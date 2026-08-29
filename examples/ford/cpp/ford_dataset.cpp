#include "ford_dataset.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

#include <fmt/format.h>

namespace ford
{
namespace
{
const mat_array_t *numeric_field(const mat_struct_t &element, const char *name)
{
  const auto *value = element.find(name);
  if (!value || value->kind != mat_value_t::kind_t::numeric)
    return nullptr;
  return &value->numeric;
}
} // namespace

bool scan_from_mat(const mat_file_t &file, const std::string &path, scan_t &out, std::string &error)
{
  const mat_value_t *scan = file.find("SCAN");
  if (!scan)
    scan = file.variables.empty() ? nullptr : &file.variables.front();
  if (!scan || scan->kind != mat_value_t::kind_t::structure || scan->elements.empty())
  {
    error = fmt::format("{}: expected a SCAN struct", path);
    return false;
  }
  const mat_struct_t &root = scan->elements.front();

  const mat_array_t *xyz = numeric_field(root, "XYZ");
  if (!xyz || xyz->rows() != 3)
  {
    error = fmt::format("{}: SCAN.XYZ is missing or not 3 x N", path);
    return false;
  }
  out.path = path;
  out.point_count = size_t(xyz->cols());
  out.xyz = xyz->data; // already column-major 3 x N, i.e. interleaved xyz per point

  if (const auto *t = numeric_field(root, "timestamp_camera"); t && !t->empty())
    out.timestamp_camera = t->data.front();
  if (const auto *t = numeric_field(root, "timestamp_laser"); t && !t->empty())
    out.timestamp_laser = t->data.front();
  if (const auto *p = numeric_field(root, "X_wv"); p && p->count() >= 6)
    for (int i = 0; i < 6; i++)
      out.pose[i] = p->data[size_t(i)];

  const mat_value_t *cams = root.find("Cam");
  if (!cams || cams->kind != mat_value_t::kind_t::structure)
  {
    error = fmt::format("{}: SCAN.Cam is missing", path);
    return false;
  }
  out.cameras.clear();
  out.cameras.reserve(cams->elements.size());
  for (const auto &element : cams->elements)
  {
    const mat_array_t *index = numeric_field(element, "points_index");
    const mat_array_t *pixels = numeric_field(element, "pixels");
    if (!index || !pixels || pixels->rows() != 2)
    {
      error = fmt::format("{}: SCAN.Cam element lacks points_index or a 2 x N pixels", path);
      return false;
    }
    const size_t n = index->count();
    if (size_t(pixels->cols()) != n)
    {
      error = fmt::format("{}: SCAN.Cam has {} indices but {} pixel pairs", path, n, pixels->cols());
      return false;
    }
    scan_camera_t camera;
    camera.point_index.resize(n);
    camera.pixel_u.resize(n);
    camera.pixel_v.resize(n);
    for (size_t i = 0; i < n; i++)
    {
      // MATLAB indices are 1-based. Storing them 0-based here means exactly one place in the
      // program has to know that, instead of every use site.
      const double raw = index->data[i];
      const int32_t zero_based = int32_t(raw) - 1;
      if (zero_based < 0 || size_t(zero_based) >= out.point_count)
      {
        error = fmt::format("{}: points_index {} is outside SCAN.XYZ (0..{})", path, raw, out.point_count);
        return false;
      }
      camera.point_index[i] = zero_based;
      camera.pixel_u[i] = pixels->data[i * 2 + 0];
      camera.pixel_v[i] = pixels->data[i * 2 + 1];
    }
    out.cameras.push_back(std::move(camera));
  }
  return true;
}

bool scan_read(const std::string &path, scan_t &out, std::string &error)
{
  mat_file_t file;
  if (!mat_read_file(path, file, error))
    return false;
  return scan_from_mat(file, path, out, error);
}

int32_t dataset_t::frame_for_timestamp(double timestamp, double &delta_us) const
{
  delta_us = 0.0;
  if (frame_timestamps.empty())
    return -1;
  // Timestamp.log is in ascending order, so a binary search finds the insertion point and the
  // nearest frame is one of the two neighbours.
  const auto it = std::lower_bound(frame_timestamps.begin(), frame_timestamps.end(), timestamp);
  size_t best = size_t(it - frame_timestamps.begin());
  if (best == frame_timestamps.size())
    best--;
  if (best > 0 && std::fabs(frame_timestamps[best - 1] - timestamp) < std::fabs(frame_timestamps[best] - timestamp))
    best--;
  delta_us = std::fabs(frame_timestamps[best] - timestamp);
  return frame_numbers[best];
}

std::string dataset_t::image_path(int32_t camera, int32_t frame) const
{
  return fmt::format("{}/IMAGES/Cam{}/image{:04d}.ppm", root, camera, frame);
}

bool dataset_open(const std::string &root, dataset_t &out, std::string &error)
{
  namespace fs = std::filesystem;
  out.root = root;

  std::error_code ec;
  const fs::path scans = fs::path(root) / "SCANS";
  if (!fs::is_directory(scans, ec))
  {
    error = fmt::format("{} has no SCANS directory", root);
    return false;
  }
  for (const auto &entry : fs::directory_iterator(scans, ec))
  {
    if (entry.is_regular_file(ec) && entry.path().extension() == ".mat")
      out.scan_paths.push_back(entry.path().string());
  }
  std::sort(out.scan_paths.begin(), out.scan_paths.end());
  if (out.scan_paths.empty())
  {
    error = fmt::format("{}/SCANS contains no .mat files", root);
    return false;
  }

  for (int32_t c = 0; c < 16; c++)
  {
    if (!fs::is_directory(fs::path(root) / "IMAGES" / fmt::format("Cam{}", c), ec))
      break;
    out.camera_count = c + 1;
  }

  const fs::path timestamps = fs::path(root) / "Timestamp.log";
  FILE *f = fopen(timestamps.string().c_str(), "rb");
  if (!f)
  {
    error = fmt::format("cannot open {}", timestamps.string());
    return false;
  }
  char line[512];
  bool first = true;
  while (fgets(line, sizeof(line), f))
  {
    if (first)
    {
      first = false; // the header row: "framecount curr_timestamp_sync ..."
      continue;
    }
    char *cursor = line;
    const long frame = strtol(cursor, &cursor, 10);
    const double sync = strtod(cursor, &cursor);
    if (frame <= 0 || sync <= 0.0)
      continue;
    out.frame_numbers.push_back(int32_t(frame));
    out.frame_timestamps.push_back(sync);
  }
  fclose(f);
  if (out.frame_timestamps.empty())
  {
    error = fmt::format("{} has no usable rows", timestamps.string());
    return false;
  }
  return true;
}

} // namespace ford
