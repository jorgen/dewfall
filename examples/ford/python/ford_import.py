#!/usr/bin/env python3
"""Import the IJRR Ford Campus dataset into a .dew dataset, from Python.

    python ford_import.py inspect <dataset-root> [--scan N]
    python ford_import.py convert <dataset-root> <output.dew> [--scans N] [--scale M] [--stride N]
    python ford_import.py colour  <dataset-root> <dataset.dew> [--scans N]

The same two-pass shape as the C++ example in ../cpp, and deliberately the same API calls in the
same order -- this is the demonstration that an importer can be orchestrated entirely from Python:

  PASS 1 (convert) ingests the geometry and carries a `scan_key` attribute, so that
  PASS 2 (colour)  can join colour onto the finished dataset without reconverting anything.

WHAT PYTHON DOES AND DOES NOT DO
--------------------------------
Python decides WHAT to convert and hands over whole NumPy arrays. It never touches a point one at
a time: the pose transform and quantization are single vectorised expressions per scan, and
everything after that -- Morton ordering, the octree, LOD, compression, the key join -- happens
inside the library.

The cost of the callback model is the GIL. The converter calls pre_init/init/convert_data on its
own worker threads, and each takes the GIL to enter Python, so scan decoding does NOT run in
parallel the way the C++ importer's does. Expect this to be read-bound where the C++ version is
not. Nothing about the DATASET differs -- the output is byte-comparable in structure -- so this is
a fair demonstration of the API and an unfair one of the throughput.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import numpy as np

import ford_dataset as ford

try:
    import dew
except ImportError:  # pragma: no cover - the usual first-run problem
    sys.exit(
        "cannot import dew.\n"
        "Build it with -DDEW_BUILD_PYTHON=ON and point PYTHONPATH at the package, e.g.\n"
        "  PYTHONPATH=<build>/bindings/python python ford_import.py ..."
    )


# Deliberately inflated over the true 16 bytes of i32x3 + u32. This number is what the converter's
# 1 GB read/sort budget divides by to decide how many inputs to keep in flight, and 3817 scans at
# their real size would let it start far more than it can hold.
APPROXIMATE_POINT_SIZE_BYTES = 96
APPROXIMATE_FILE_SIZE_BYTES = 6_500_000
APPROXIMATE_POINTS_PER_SCAN = 77_000


def _fail(message):
    print(f"error: {message}", file=sys.stderr)
    return 1


# ---------------------------------------------------------------------------------------------
# inspect


def inspect(root, scan_ordinal):
    dataset = ford.Dataset(root)
    print(f"{root}\n  {len(dataset.scan_paths)} scans, {dataset.camera_count} cameras, "
          f"{dataset.frame_timestamps.size} image timestamps")

    path = dataset.scan_paths[scan_ordinal if 0 <= scan_ordinal < len(dataset.scan_paths) else 0]
    scan = ford.read_scan(path)
    world = ford.world_points(scan)
    print(f"\n  {os.path.basename(path)}  (scan id {scan.scan_id})")
    print(f"    {scan.point_count} points, {len(scan.cameras)} cameras with pixel tables")
    print(f"    pose      x {scan.pose[0]:.2f}  y {scan.pose[1]:.2f}  z {scan.pose[2]:.2f}"
          f"   roll {scan.pose[3]:.4f}  pitch {scan.pose[4]:.4f}  yaw {scan.pose[5]:.4f}")
    print(f"    vehicle   min {np.round(scan.xyz.min(axis=0), 2)}  max {np.round(scan.xyz.max(axis=0), 2)}")
    print(f"    world     min {np.round(world.min(axis=0), 2)}  max {np.round(world.max(axis=0), 2)}")

    frame, delta = dataset.frame_for_timestamp(scan.timestamp_camera)
    print(f"    nearest image frame {frame} ({delta / 1000.0:.1f} ms away)")
    for index, camera in enumerate(scan.cameras):
        image_file = dataset.image_path(index, frame)
        exists = "present" if os.path.isfile(image_file) else "MISSING"
        print(f"      cam{index}: {camera.point_index.size} points  -> {os.path.basename(image_file)} ({exists})")
    return 0


# ---------------------------------------------------------------------------------------------
# pass 1: convert


class _ConvertState:
    """Everything the three converter callbacks share.

    The scan cache exists because pre_init and init both need the decoded scan -- pre_init for the
    point count and the pose, init for the points themselves. The C++ importer simply decodes
    twice, which is cheap there and is not here. The cache is BOUNDED because pre_init runs ahead
    of init on the converter's pool, so an unbounded one would hold the whole dataset; on a miss
    init just decodes again.
    """

    def __init__(self, scale, offset, world_min, world_max, cache_limit=32):
        self.scale = scale
        self.offset = np.asarray(offset, dtype=np.float64)
        self.world_min = np.asarray(world_min, dtype=np.float64)
        self.world_max = np.asarray(world_max, dtype=np.float64)
        self.cache = {}
        self.cache_limit = cache_limit
        self.lock = threading.Lock()
        self.scans_started = 0
        self.points = 0
        self.failures = []

    def take(self, path):
        with self.lock:
            return self.cache.pop(path, None)

    def offer(self, path, scan):
        with self.lock:
            if len(self.cache) < self.cache_limit:
                self.cache[path] = scan


class _FileState:
    """Per-input state: the quantized points, ready to be sliced into the converter's buffers."""

    __slots__ = ("xyz", "keys", "emitted")

    def __init__(self, xyz, keys):
        self.xyz = xyz
        self.keys = keys
        self.emitted = 0


def convert(root, output, scan_limit, scale, stride, read_cache_mb, decoded_cache_mb):
    dataset = ford.Dataset(root)
    paths = dataset.scan_paths[:scan_limit] if scan_limit else dataset.scan_paths
    print(f"{root}\n  {len(paths)} scans to import")

    # Every scan id must fit the key's 12 bits, and overflow is silent -- check before doing work.
    for path in paths:
        if ford.scan_id_from_path(path) > ford.MAX_SCAN_ID:
            return _fail(f"{path}: scan id exceeds {ford.MAX_SCAN_ID}, which the key packing cannot hold")

    started = time.monotonic()
    world_min, world_max, sampled = ford.trajectory_bounds(dataset, stride=stride, limit=scan_limit)
    print(f"  trajectory from {sampled} sampled poses in {time.monotonic() - started:.1f}s")
    span = world_max - world_min
    print(f"  world box [{world_min[0]:.1f} {world_min[1]:.1f} {world_min[2]:.1f}]"
          f" .. [{world_max[0]:.1f} {world_max[1]:.1f} {world_max[2]:.1f}]"
          f"  ({span[0]:.0f} x {span[1]:.0f} x {span[2]:.0f} m)")

    # Anchor the offset at the box minimum so every quantized value is small and positive: 700 m at
    # 1 mm is 700k, which keeps the compressor's delta coding on small numbers.
    state = _ConvertState(scale=scale, offset=world_min, world_min=world_min, world_max=world_max)
    inverse_scale = 1.0 / scale

    converter = dew.Converter(output, dew.ConverterOpenFileSemantics.truncate)
    converter.set_read_cache_bytes(read_cache_mb << 20)
    converter.set_decompressed_cache_bytes(decoded_cache_mb << 20)
    # Pass 2 amends this dataset, and only a MUTABLE one can be amended.
    converter.set_mutable(1)
    # Mandatory here: the default LOD keep-list is rgb/intensity/classification, so without this the
    # key would not survive LOD and only leaf nodes could ever be coloured.
    converter.set_lod_all_attributes(1)

    def pre_init(filename):
        info = dew.ConverterFilePreInitInfo()
        info.approximate_point_size_bytes = APPROXIMATE_POINT_SIZE_BYTES
        info.input_file_size_bytes = APPROXIMATE_FILE_SIZE_BYTES
        info.scale = [scale] * 3
        info.found_scale = 1
        info.approximate_point_count = APPROXIMATE_POINTS_PER_SCAN
        try:
            scan = ford.read_scan(filename, with_cameras=False)
        except Exception:
            return info  # init will fail on it too and report properly
        state.offer(filename, scan)
        info.approximate_point_count = scan.point_count
        info.found_point_count = 1
        # LOAD-BEARING, not an optimisation. The converter dispatches inputs in rising min-morton
        # order and derives the done-morton watermark from this; without it the watermark sits at
        # zero for the whole run and every collapse and LOD pass is deferred into one terminal
        # pass -- which on the full dataset meant 62 minutes on a fraction of one core.
        #
        # It must be a TRUE lower bound on every axis: too low only slows the watermark, too high
        # silently DROPS points.
        info.aabb_min = list(scan.pose[:3] - ford.BOUNDS_PADDING)
        info.found_aabb_min = 1
        return info

    def init(filename, header, attributes):
        scan = state.take(filename) or ford.read_scan(filename, with_cameras=False)

        # The EXACT count, never an estimate: declaring 77000 and then emitting the true 77276
        # wedges the conversion.
        header.point_count = scan.point_count
        header.scale = [scale] * 3
        header.offset = list(state.offset)
        # The dataset-wide box for every input. A per-scan box would be tighter but would mean
        # decoding the scan just to measure it.
        header.min = list(state.world_min)
        header.max = list(state.world_max)

        # xyz first and as i32x3 -- the library refuses anything else in that slot. scan_key second,
        # as an ordinary attribute; nothing in the library knows it is special.
        attributes.add_attribute(dew.ATTRIBUTE_XYZ, dew.Type.i32, dew.Components.components_3)
        attributes.add_attribute(ford.KEY_ATTRIBUTE, dew.Type.u32, dew.Components.components_1)

        # The whole per-point cost of this importer, as two vectorised expressions. np.rint before
        # the cast matters: a plain cast truncates toward zero, which biases negative coordinates.
        world = ford.world_points(scan)
        quantized = np.rint((world - state.offset) * inverse_scale).astype(np.int32)
        keys = ford.make_scan_key(scan.scan_id, np.arange(scan.point_count, dtype=np.uint32))

        with state.lock:
            state.scans_started += 1
            state.points += scan.point_count
        return _FileState(np.ascontiguousarray(quantized), np.ascontiguousarray(keys))

    def convert_data(file_state, header, attributes, buffers, max_points):
        xyz_out, key_out = buffers
        begin = file_state.emitted
        end = min(begin + min(max_points, xyz_out.shape[0]), file_state.xyz.shape[0])
        count = end - begin
        if count == 0:
            return 0, True
        xyz_out[:count] = file_state.xyz[begin:end]
        key_out[:count, 0] = file_state.keys[begin:end]
        file_state.emitted = end
        return count, end >= file_state.xyz.shape[0]

    converter.set_file_converter_callbacks(pre_init=pre_init, init=init, convert_data=convert_data)

    warnings, errors = [], []
    last_report = [0.0]

    def on_progress(fraction):
        now = time.monotonic()
        if now - last_report[0] > 0.5:
            last_report[0] = now
            with state.lock:
                scans, points = state.scans_started, state.points
            print(f"\r  {time.monotonic() - started:7.1f}s  {fraction:6.1%}  {scans} scans, {points} points   ",
                  end="", flush=True)

    converter.set_runtime_callbacks(
        progress=on_progress,
        warning=warnings.append,
        error=lambda e: errors.append(f"{e.code}: {e.message}"),
    )

    converter.add_data_file(list(paths))
    converter.wait_idle()
    elapsed = time.monotonic() - started
    print(f"\r  {state.scans_started} scans, {state.points} points in {elapsed:.1f}s "
          f"({state.points / max(elapsed, 1e-9):,.0f} points/s)".ljust(78))

    for warning in warnings[:10]:
        print(f"  warning: {warning}")
    if len(warnings) > 10:
        print(f"  ... and {len(warnings) - 10} more warnings")
    if errors:
        for message in errors[:10]:
            print(f"  error: {message}", file=sys.stderr)
        return 1

    stats_path = output + ".stats.json"
    if converter.write_stats(stats_path):
        print(f"  run statistics: {stats_path}")
    print(f"\ncolour it with:  python ford_import.py colour {root} {output}")
    return 0


# ---------------------------------------------------------------------------------------------
# pass 2: colour


def _colour_one_scan(scan, images):
    """(rgb, have, out_of_range) for one scan, from the per-camera pixel tables.

    A point seen by several cameras -- 44k of a scan's 77k are -- takes the colour from the camera
    whose pixel is NEAREST THE IMAGE CENTRE, where lens distortion and exposure are best behaved.
    """
    rgb = np.zeros((scan.point_count, 3), dtype=np.uint8)
    have = np.zeros(scan.point_count, dtype=bool)
    best = np.full(scan.point_count, np.inf, dtype=np.float32)
    out_of_range = 0

    for index, camera in enumerate(scan.cameras):
        image = images[index] if index < len(images) else None
        if image is None:
            continue
        height, width = image.shape[:2]
        # The entire coordinate mapping. The cameras are natively 1232 x 1616 and the released
        # per-camera images are 1616 x 616 -- the native frame transposed with its long axis halved
        # -- while Cam.pixels is in the NATIVE frame. So u passes through and v is halved. The -1 is
        # MATLAB's 1-based pixel origin.
        x = np.rint(camera.pixel_u - 1.0).astype(np.int64)
        y = np.rint((camera.pixel_v - 1.0) * 0.5).astype(np.int64)
        inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        out_of_range += int((~inside).sum())
        if not inside.any():
            continue

        point = camera.point_index[inside]
        x, y = x[inside], y[inside]
        dx = (x - width * 0.5).astype(np.float32)
        dy = (y - height * 0.5).astype(np.float32)
        score = dx * dx + dy * dy

        # Strictly better wins, so on an exact tie the earlier camera keeps the point.
        improves = score < best[point]
        point, x, y, score = point[improves], x[improves], y[improves], score[improves]
        rgb[point] = image[y, x]
        best[point] = score
        have[point] = True

    return rgb, have, out_of_range


def colour(root, dataset_path, scan_limit):
    dataset = ford.Dataset(root)
    paths = dataset.scan_paths[:scan_limit] if scan_limit else dataset.scan_paths

    converter = dew.Converter(dataset_path, dew.ConverterOpenFileSemantics.open_existing)
    # Amending needs a MUTABLE dataset: a finalized one has trees that have been banded and, in
    # destination mode, evicted, and an amend has to read every unit it touches. Pass 1 left it
    # mutable; a fresh process has to say so again after reopening.
    converter.set_mutable(1)
    warnings, errors = [], []
    converter.set_runtime_callbacks(warning=warnings.append,
                                    error=lambda e: errors.append(f"{e.code}: {e.message}"))

    if not converter.add_attribute(dew.ATTRIBUTE_RGB, ford.KEY_ATTRIBUTE, dew.Type.u8, dew.Components.components_3):
        return _fail(f"could not declare {dew.ATTRIBUTE_RGB} on {dataset_path}")

    print(f"{root}\n  sampling colour for {dataset_path}")
    started = time.monotonic()
    scans_read = scans_failed = scans_without_image = images_loaded = 0
    points_seen = points_coloured = pixels_out_of_range = 0
    cached_frame, cached_images = -1, []

    for ordinal, path in enumerate(paths):
        try:
            scan = ford.read_scan(path)
        except Exception:
            # The release contains truncated scans. Skipping one costs that scan's colour and
            # nothing else -- its points keep the zeroed rgb a point with no table entry gets.
            scans_failed += 1
            continue
        scans_read += 1

        scan_id = ford.scan_id_from_path(path)
        if scan_id > ford.MAX_SCAN_ID or scan.point_count > ford.MAX_POINT_INDEX + 1:
            return _fail(f"{path}: scan id {scan_id} / {scan.point_count} points do not fit the key packing")

        frame, _ = dataset.frame_for_timestamp(scan.timestamp_camera)
        if frame != cached_frame:
            cached_images = []
            for camera_index in range(dataset.camera_count):
                image_file = dataset.image_path(camera_index, frame)
                try:
                    cached_images.append(ford.read_ppm(image_file))
                    images_loaded += 1
                except Exception:
                    cached_images.append(None)
            cached_frame = frame
        if not any(image is not None for image in cached_images):
            scans_without_image += 1
            continue

        rgb, have, out_of_range = _colour_one_scan(scan, cached_images)
        points_seen += scan.point_count
        pixels_out_of_range += out_of_range

        index = np.nonzero(have)[0]
        if index.size == 0:
            continue
        points_coloured += int(index.size)
        # uint64 keys regardless of the key attribute's own width -- that is the C signature, and
        # the u32 value is simply widened.
        keys = ford.make_scan_key(scan_id, index.astype(np.uint32)).astype(np.uint64)
        if not converter.add_data_for_attribute(dew.ATTRIBUTE_RGB, np.ascontiguousarray(keys),
                                                np.ascontiguousarray(rgb[index])):
            return _fail(f"add_data_for_attribute failed on {path}")

        if ordinal % 25 == 0:
            print(f"\r  {time.monotonic() - started:7.1f}s  {ordinal + 1}/{len(paths)} scans, "
                  f"{points_coloured} points coloured, {images_loaded} images read   ", end="", flush=True)

    sample_seconds = time.monotonic() - started
    print(f"\r  {scans_read} scans read ({scans_failed} unreadable, {scans_without_image} without a "
          f"matching image), {images_loaded} images".ljust(78))
    percent = 100.0 * points_coloured / points_seen if points_seen else 0.0
    print(f"  {points_coloured} of {points_seen} points coloured ({percent:.1f}%), "
          f"{pixels_out_of_range} pixels out of range")
    print(f"  sampled in {sample_seconds:.1f}s")

    # THE pass. One read and one write of one attribute per unit -- the unit's existing blobs are
    # not read, not recompressed and not moved.
    print(f"\ncommitting {dew.ATTRIBUTE_RGB} to every node ...")
    commit_started = time.monotonic()
    converter.commit_attributes()
    commit_seconds = time.monotonic() - commit_started
    if errors:
        for message in errors[:10]:
            print(f"  error: {message}", file=sys.stderr)
        return _fail("the amend failed")
    converter.finalize()
    print(f"  committed in {commit_seconds:.1f}s\n\ninspect it with:  dew info {dataset_path}")
    return 0


# ---------------------------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["inspect", "convert", "colour", "color"])
    parser.add_argument("root", help="the IJRR dataset root (the directory holding SCANS/)")
    parser.add_argument("output", nargs="?", help="the .dew dataset to write (convert) or amend (colour)")
    parser.add_argument("--scan", type=int, default=0, help="inspect: which scan, by ordinal")
    parser.add_argument("--scans", type=int, default=0, help="stop after this many scans (0 = all)")
    parser.add_argument("--scale", type=float, default=0.001, help="quantization step in metres")
    parser.add_argument("--stride", type=int, default=32, help="convert: pose sampling stride for the world box")
    parser.add_argument("--read-cache-mb", type=int, default=1024)
    parser.add_argument("--decoded-cache-mb", type=int, default=1024)
    args = parser.parse_args()

    if args.command == "inspect":
        return inspect(args.root, args.scan)
    if not args.output:
        return _fail(f"{args.command} needs an output dataset path")
    if args.command == "convert":
        return convert(args.root, args.output, args.scans, args.scale, args.stride,
                       args.read_cache_mb, args.decoded_cache_mb)
    return colour(args.root, args.output, args.scans)


if __name__ == "__main__":
    sys.exit(main())
