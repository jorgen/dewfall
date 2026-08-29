/* The layout of an IJRR Ford Campus Vision and Lidar dataset directory, and the typed view of one
 * scan that the importer works against.
 *
 *   <root>/SCANS/Scan####.mat          one lidar sweep each
 *   <root>/IMAGES/Cam{0..4}/image####.ppm
 *   <root>/IMAGES/FULL/image####.ppm   the five stacked vertically (1616 x 3080)
 *   <root>/Timestamp.log               framecount -> camera timestamp
 *
 * WHAT MAKES THIS DATASET CHEAP TO COLOUR. Each scan carries, per camera, the pixel coordinates of
 * every point that camera saw -- SCAN.Cam(i).pixels -- alongside a 1-based index into SCAN.XYZ. So
 * there is no projection to derive, no intrinsics or extrinsics to apply, and no calibration to
 * guess: colouring is a lookup. The five cameras' index sets together cover every point of the
 * scan exactly once or more, so no point is left uncoloured for want of a view.
 *
 * The one thing the scan does NOT tell you is which image. SCAN.image_index is zero in every file
 * in the dataset and is useless; SCAN.timestamp_camera is real, and Timestamp.log maps it to a
 * frame number. See frame_for_timestamp.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mat_v5.hpp"

namespace ford
{

// One camera's contribution to a scan: which points it saw, and where they landed in its image.
struct scan_camera_t
{
  std::vector<int32_t> point_index; // 0-based here; 1-based in the file
  std::vector<double> pixel_u;      // spans the image's long (1616) axis
  std::vector<double> pixel_v;      // spans 1..1232, i.e. twice the stored image's 616 short axis
};

struct scan_t
{
  std::string path;
  std::vector<double> xyz;       // 3 * point_count, interleaved x,y,z (as stored)
  size_t point_count = 0;
  double timestamp_camera = 0.0;
  double timestamp_laser = 0.0;
  double pose[6] = {};           // SCAN.X_wv: vehicle pose, x y z roll pitch yaw
  std::vector<scan_camera_t> cameras;

  [[nodiscard]] const double *point(size_t i) const
  {
    return xyz.data() + i * 3;
  }
};

// Convert an already-parsed .mat into the view above. Kept separate from mat_read_file so the
// generic reader stays free of anything Ford-specific.
bool scan_from_mat(const mat_file_t &file, const std::string &path, scan_t &out, std::string &error);

bool scan_read(const std::string &path, scan_t &out, std::string &error);

struct dataset_t
{
  std::string root;
  std::vector<std::string> scan_paths;    // sorted; Scan0075..Scan3891 in the released set
  std::vector<int32_t> frame_numbers;     // Timestamp.log column 1
  std::vector<double> frame_timestamps;   // Timestamp.log column 2 (curr_timestamp_sync)
  int32_t camera_count = 0;

  // Nearest frame by camera timestamp. Returns -1 if there are no timestamps. `delta_us` receives
  // the match's absolute error, which is worth checking: the cameras run at about 8 Hz, so a match
  // worse than ~60 ms means the scan has no contemporaneous image and should be left uncoloured
  // rather than coloured from the wrong frame.
  [[nodiscard]] int32_t frame_for_timestamp(double timestamp, double &delta_us) const;

  [[nodiscard]] std::string image_path(int32_t camera, int32_t frame) const;
};

bool dataset_open(const std::string &root, dataset_t &out, std::string &error);

} // namespace ford
