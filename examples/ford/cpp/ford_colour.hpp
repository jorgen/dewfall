/* Pass 2: give the converted dataset colour, joined on the key pass 1 stored. */
#pragma once

#include <cstddef>
#include <string>

struct dew_converter_t;

namespace ford
{
struct dataset_t;

struct colour_stats_t
{
  size_t scans_read = 0;
  size_t scans_failed = 0;
  size_t scans_without_image = 0;
  size_t points_coloured = 0;
  size_t points_seen = 0;
  size_t pixels_out_of_range = 0;
  size_t images_loaded = 0;
};

// Walk the scans, sample each point's colour out of the camera image that saw it, and hand the
// (key, rgb) pairs to the converter. Does NOT commit -- the caller decides when, because a commit
// is a whole pass over the dataset.
bool ford_colour_scans(const dataset_t &dataset, dew_converter_t *converter, size_t scan_limit, colour_stats_t &stats, std::string &error);

} // namespace ford
