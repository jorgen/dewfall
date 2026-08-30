"""Reading the IJRR Ford Campus dataset: scans, poses, images, and the join key.

This is the half of the importer Python is genuinely good at -- pulling file formats apart -- and
it is deliberately the only half that looks at a point individually. Everything here hands back
whole NumPy arrays; the per-point work (Morton ordering, the octree, LOD, compression) happens
inside the dew library. ford_import.py does the orchestration.

Layout on disk:

    <root>/SCANS/Scan####.mat            one Velodyne sweep, MATLAB v5, zlib-compressed elements
    <root>/IMAGES/Cam<c>/image####.ppm   per-camera frames, P6, 1616 x 616
    <root>/Timestamp.log                 framecount -> camera timestamp

A scan carries its own vehicle pose and, per camera, the pixel coordinate of every point that
camera saw. That last part is why colouring is a LOOKUP rather than a projection: there are no
intrinsics, no extrinsics and no calibration anywhere in this program.

The one thing a scan does not tell you is WHICH image. SCAN.image_index is zero in every file in
the release; SCAN.timestamp_camera is real, and Timestamp.log maps it to a frame number.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np
import scipy.io

# The join key: scan number in the high 12 bits, point index within the scan in the low 20. Pass 1
# carries it as an ordinary u32 attribute; pass 2 joins colour onto it.
#
# make_scan_key does NOT mask the scan id, so an id above 4095 would silently alias another scan.
# Both passes check rather than mask, because that aliasing is invisible in the output.
KEY_ATTRIBUTE = "scan_key"
MAX_SCAN_ID = (1 << 12) - 1
MAX_POINT_INDEX = (1 << 20) - 1

# Velodyne reach (~110 m) plus how far the vehicle travels between sampled poses. Pads the dataset
# box, and gives each scan a true lower bound on where its points can land.
BOUNDS_PADDING = 160.0


def make_scan_key(scan_id, point_index):
    """Pack (scan, index) into the u32 key. `point_index` may be an array."""
    scan_bits = np.uint32(scan_id) << np.uint32(20)
    return (np.asarray(point_index, dtype=np.uint32) & np.uint32(MAX_POINT_INDEX)) | scan_bits


def scan_key_scan_id(key):
    return np.asarray(key, dtype=np.uint32) >> np.uint32(20)


def scan_key_point_index(key):
    return np.asarray(key, dtype=np.uint32) & np.uint32(MAX_POINT_INDEX)


def scan_id_from_path(path):
    """The run of trailing digits ending at the last '.' -- 'Scan0075.mat' -> 75.

    Deliberately the file's own number rather than an ordinal in the sorted list, so keys stay
    stable when only part of the dataset is imported -- including when a scan turns out to be
    unreadable and is skipped, which the release guarantees will happen.
    """
    end = path.rfind(".")
    if end < 0:
        end = len(path)
    begin = end
    while begin > 0 and "0" <= path[begin - 1] <= "9":
        begin -= 1
    return int(path[begin:end]) if begin < end else 0


@dataclass
class Camera:
    """One camera's view of a scan: which points it saw, and where they landed."""

    point_index: np.ndarray  # int32, 0-based into Scan.xyz
    pixel_u: np.ndarray  # float64, native-frame column (spans 1616)
    pixel_v: np.ndarray  # float64, native-frame row (spans 1..1232, twice the stored height)


@dataclass
class Scan:
    path: str
    xyz: np.ndarray  # (N, 3) float64, VEHICLE frame
    pose: np.ndarray  # (6,) x y z roll pitch yaw -- SCAN.X_wv
    timestamp_camera: float
    cameras: list = field(default_factory=list)

    @property
    def point_count(self):
        return int(self.xyz.shape[0])

    @property
    def scan_id(self):
        return scan_id_from_path(self.path)


def read_scan(path, with_cameras=True):
    """Decode one SCANS/*.mat. Raises ValueError on anything malformed.

    `with_cameras=False` skips building the per-camera tables, which pass 1 never looks at.
    """
    mat = scipy.io.loadmat(path, squeeze_me=True, struct_as_record=False)
    root = mat.get("SCAN")
    if root is None:
        names = [k for k in mat if not k.startswith("__")]
        if not names:
            raise ValueError(f"{path}: no variables")
        root = mat[names[0]]

    xyz = np.asarray(getattr(root, "XYZ", np.empty(0)))
    if xyz.ndim != 2 or xyz.shape[0] != 3:
        raise ValueError(f"{path}: SCAN.XYZ is missing or not 3 x N")
    # MATLAB is column-major and the array arrives as (3, N): column i is point i.
    points = np.ascontiguousarray(xyz.T, dtype=np.float64)
    count = points.shape[0]

    pose = np.zeros(6, dtype=np.float64)
    raw_pose = np.asarray(getattr(root, "X_wv", np.empty(0))).ravel()
    if raw_pose.size >= 6:
        pose[:] = raw_pose[:6]

    stamp = np.asarray(getattr(root, "timestamp_camera", 0.0)).ravel()
    scan = Scan(path=path, xyz=points, pose=pose, timestamp_camera=float(stamp[0]) if stamp.size else 0.0)
    if not with_cameras:
        return scan

    cameras = np.asarray(getattr(root, "Cam", np.empty(0)), dtype=object)
    for raw in np.atleast_1d(cameras).ravel():
        if not hasattr(raw, "points_index"):
            continue
        index = np.atleast_1d(np.asarray(raw.points_index)).ravel()
        if index.size == 0:
            continue
        pixels = np.asarray(raw.pixels)
        if pixels.ndim != 2 or pixels.shape[0] != 2 or pixels.shape[1] != index.size:
            raise ValueError(f"{path}: Cam.pixels is not 2 x len(points_index)")
        # 1-based -> 0-based once, here, and bounds-checked: an out-of-range index would scatter
        # colour onto the wrong point, and no later stage could detect that.
        zero_based = index.astype(np.int64) - 1
        if zero_based.min() < 0 or zero_based.max() >= count:
            raise ValueError(f"{path}: Cam.points_index out of range")
        scan.cameras.append(
            Camera(
                point_index=zero_based.astype(np.int32),
                pixel_u=np.ascontiguousarray(pixels[0], dtype=np.float64),
                pixel_v=np.ascontiguousarray(pixels[1], dtype=np.float64),
            )
        )
    return scan


def rotation_from_pose(pose):
    """The 3x3 world-from-vehicle rotation: Rx(roll) @ Ry(pitch) @ Rz(yaw - pi/2).

    The yaw offset is not documented anywhere in the release. It was established empirically, by
    maximising occupied voxels per point at 0.3 m over every Euler order, sign and quarter turn.
    Only the yaw term is actually pinned down by the data: the xyz and yxz orders tie exactly, so
    the roll/pitch order is a guess that happens not to matter at this dataset's roll and pitch.
    """
    roll, pitch = float(pose[3]), float(pose[4])
    yaw = float(pose[5]) - math.pi / 2.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cp * cy, -cp * sy, sp],
            [sr * sp * cy + cr * sy, -sr * sp * sy + cr * cy, -sr * cp],
            [-cr * sp * cy + sr * sy, cr * sp * sy + sr * cy, cr * cp],
        ],
        dtype=np.float64,
    )


def world_points(scan):
    """(N, 3) world coordinates.

    SCAN.XYZ is in the VEHICLE frame: every scan's points sit within a couple of metres of the
    origin until its own pose places them, so skipping this piles a 450 m loop into one heap.
    """
    return scan.xyz @ rotation_from_pose(scan.pose).T + scan.pose[:3]


def read_ppm(path):
    """(H, W, 3) uint8. Pillow reads the release's P6 files directly."""
    from PIL import Image

    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"), dtype=np.uint8)


class Dataset:
    """A dataset root: the scan list, the camera count, and the frame timestamp table."""

    def __init__(self, root):
        self.root = root
        scans_dir = os.path.join(root, "SCANS")
        if not os.path.isdir(scans_dir):
            raise ValueError(f"{root} has no SCANS directory")
        self.scan_paths = sorted(
            os.path.join(scans_dir, name) for name in os.listdir(scans_dir) if name.lower().endswith(".mat")
        )
        if not self.scan_paths:
            raise ValueError(f"{scans_dir} contains no .mat files")

        self.camera_count = 0
        while self.camera_count < 16 and os.path.isdir(os.path.join(root, "IMAGES", f"Cam{self.camera_count}")):
            self.camera_count += 1

        # Timestamp.log: "framecount curr_timestamp_sync ...", one header row, ascending.
        frames, stamps = [], []
        with open(os.path.join(root, "Timestamp.log"), "r", errors="replace") as handle:
            for line_number, line in enumerate(handle):
                if line_number == 0:
                    continue
                parts = line.split()
                if len(parts) < 2:
                    continue
                try:
                    frame, sync = int(parts[0]), float(parts[1])
                except ValueError:
                    continue
                if frame > 0 and sync > 0.0:
                    frames.append(frame)
                    stamps.append(sync)
        if not stamps:
            raise ValueError(f"{root}/Timestamp.log has no usable rows")
        self.frame_numbers = np.asarray(frames, dtype=np.int32)
        self.frame_timestamps = np.asarray(stamps, dtype=np.float64)

    def frame_for_timestamp(self, timestamp):
        """Nearest frame by camera timestamp. Returns (frame_number, delta)."""
        best = int(np.searchsorted(self.frame_timestamps, timestamp))
        if best == self.frame_timestamps.size:
            best -= 1
        if best > 0 and abs(self.frame_timestamps[best - 1] - timestamp) < abs(self.frame_timestamps[best] - timestamp):
            best -= 1
        return int(self.frame_numbers[best]), abs(float(self.frame_timestamps[best]) - timestamp)

    def image_path(self, camera, frame):
        return os.path.join(self.root, "IMAGES", f"Cam{camera}", f"image{frame:04d}.ppm")


def trajectory_bounds(dataset, stride=32, limit=0):
    """The dataset's world box, from POSES alone -- no points are read.

    Sampling every stride-th scan and padding by the sensor's reach is enough, and the box only has
    to CONTAIN the data: the converter quantizes against it, so slack costs a little resolution
    while a box that is too small silently drops points.
    """
    paths = dataset.scan_paths[: limit or None]
    low = np.full(3, np.inf)
    high = np.full(3, -np.inf)
    sampled = 0
    for path in paths[:: max(stride, 1)]:
        try:
            pose = read_scan(path, with_cameras=False).pose[:3]
        except Exception:
            continue  # the release contains truncated scans; one costs its own pose and nothing else
        low = np.minimum(low, pose)
        high = np.maximum(high, pose)
        sampled += 1
    if sampled == 0:
        raise ValueError("no readable scans to derive bounds from")
    return low - BOUNDS_PADDING, high + BOUNDS_PADDING, sampled
