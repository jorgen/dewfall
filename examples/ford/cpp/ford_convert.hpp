/* Pass 1: ingest the Ford scans into a .dew dataset, carrying a join key. */
#pragma once

#include <cstdint>
#include <string>

struct dew_converter_t;

namespace ford
{
struct dataset_t;

// The attribute the colour pass will join on. An ordinary attribute in every respect -- the library
// has no notion of a "key"; it is a key only because pass 2 names it as one.
constexpr const char *k_key_attribute = "scan_key";

// scan number in the high 12 bits, index within the scan in the low 20.
//
// A u32 rather than the obvious u64 (scan << 32 | index), and the reason is size: the key is stored
// per point, and after the morton sort a scan's points are scattered across the octree, so the key
// column compresses poorly. Over the full 294 M-point dataset that is 1.2 GB at u32 and 2.4 GB at
// u64 -- the difference between a join key that costs something and one that dominates the dataset.
//
// The cost of that choice is the two limits below, which is why they are checked rather than
// assumed.
constexpr uint32_t k_max_scan_id = (1u << 12) - 1;
constexpr uint32_t k_max_point_index = (1u << 20) - 1;

constexpr uint32_t make_scan_key(uint32_t scan_id, uint32_t point_index)
{
  return (scan_id << 20) | (point_index & k_max_point_index);
}
constexpr uint32_t scan_key_scan_id(uint32_t key)
{
  return key >> 20;
}
constexpr uint32_t scan_key_point_index(uint32_t key)
{
  return key & k_max_point_index;
}

constexpr uint64_t k_approximate_points_per_scan = 77000;
constexpr uint64_t k_approximate_scan_bytes = 6500000;
// Velodyne range plus the vehicle's travel between two sampled poses. See ford_trajectory_bounds.
constexpr double k_bounds_padding = 160.0;

// Sample every `stride`-th scan's pose and return the trajectory box, padded to bracket the points.
// The scan number encoded in a path like "Scan0075.mat"; 0 if there is none.
uint32_t scan_id_from_path(const std::string &path);

bool ford_trajectory_bounds(const dataset_t &dataset, size_t stride, double (&world_min)[3], double (&world_max)[3], size_t &sampled, std::string &error);

void ford_configure_import(const dataset_t &dataset, double scale, const double (&world_min)[3], const double (&world_max)[3]);
void ford_install_callbacks(dew_converter_t *converter);
// scans_failed counts inputs whose .mat could not be decoded -- the Ford release contains at
// least one truncated scan, and one bad file out of 3817 is not a reason to lose the other 3816.
void ford_import_progress(size_t &scans_done, size_t &points_done, size_t &scans_failed);

} // namespace ford
