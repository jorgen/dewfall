/* Pass 2 of the importer: colour, joined onto the key pass 1 stored.
 *
 * Nothing here touches geometry. The dataset already exists -- converted, sorted into a morton
 * octree, LOD generated -- and this adds one attribute to it without reconverting any of that. The
 * only thing pass 1 had to do to make this possible was carry `scan_key`.
 *
 * WHY THIS IS A LOOKUP AND NOT A PROJECTION. Each scan carries, per camera, the pixel coordinates of
 * every point that camera saw (SCAN.Cam(i).pixels) alongside a 1-based index into SCAN.XYZ. So there
 * are no intrinsics, no extrinsics and no calibration involved: read the pixel, read the image,
 * done. The five cameras' index sets together cover every point in the scan.
 *
 * LOD NEEDS NO SPECIAL CASE, which is the part worth understanding. An LOD node holds a SUBSET of
 * its descendants' points, and its scan_key buffer holds those points' keys -- so joining the LOD
 * nodes against the same table gives each sampled point its own true colour. No re-LOD, no
 * provenance metadata. The key attribute IS the provenance.
 */

#include "ford_colour.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

#include <fmt/format.h>

#include <dew/converter/converter.h>
#include <dew/core/default_attribute_names.h>

#include "ford_convert.hpp"
#include "ford_dataset.hpp"
#include "ppm.hpp"

namespace ford
{
namespace
{

// Where a Ford pixel coordinate lands in the stored PPM.
//
// The cameras are native 1232 x 1616; the released per-camera images are 1616 x 616, i.e. the native
// frame transposed with its long axis halved. SCAN.Cam.pixels is in the NATIVE frame: the first row
// spans the 1616 axis and the second spans 1..1232. So u passes through and v is halved.
//
// Established empirically -- the release ships no documentation of it. Under this mapping 100% of
// points land inside the image and the colour difference between 3D-nearest-neighbour points is
// about 30 against a random-pair baseline of about 190. Getting the axes wrong instead puts most
// points outside the image entirely, which is why the out-of-range count is reported: it is the
// canary for this assumption.
bool map_pixel(double u, double v, const image_t &image, int32_t &x, int32_t &y)
{
  x = int32_t(std::lround(u - 1.0));       // MATLAB pixel coordinates are 1-based
  y = int32_t(std::lround((v - 1.0) * 0.5));
  return image.contains(x, y);
}

// The five images for one frame. Consecutive scans usually share a frame (3817 scans against 3060
// frames), so holding the last one turns most scans' image loading into nothing at all.
struct frame_cache_t
{
  int32_t frame = -1;
  std::vector<image_t> images;
  bool complete = false;
};

bool load_frame(const dataset_t &dataset, int32_t frame, frame_cache_t &cache, size_t &images_loaded)
{
  if (cache.frame == frame)
    return cache.complete;
  cache.frame = frame;
  cache.complete = false;
  cache.images.assign(size_t(dataset.camera_count), image_t{});
  std::string message;
  for (int32_t c = 0; c < dataset.camera_count; c++)
  {
    if (!ppm_read(dataset.image_path(c, frame), cache.images[size_t(c)], message))
      return false; // a frame is all-or-nothing; a partial one would colour some cameras and not others
    images_loaded++;
  }
  cache.complete = true;
  return true;
}

} // namespace

bool ford_colour_scans(const dataset_t &dataset, dew_converter_t *converter, size_t scan_limit, colour_stats_t &stats, std::string &error)
{
  const size_t count = scan_limit && scan_limit < dataset.scan_paths.size() ? scan_limit : dataset.scan_paths.size();
  frame_cache_t cache;

  // Flushed per scan rather than accumulated across all of them: add_data_for_attribute COPIES what
  // it is given, so these can stay scan-sized. (The converter still holds the whole table until
  // commit_attributes -- see the note in the colour command about what that costs.)
  std::vector<uint64_t> keys;
  std::vector<uint8_t> values;

  // Per-scan scratch, sized once at the first scan and reused.
  std::vector<uint8_t> have;
  std::vector<uint8_t> rgb;
  std::vector<float> best;

  const auto started = std::chrono::steady_clock::now();
  for (size_t s = 0; s < count; s++)
  {
    scan_t scan;
    std::string message;
    if (!scan_read(dataset.scan_paths[s], scan, message))
    {
      // The release contains at least one truncated scan. Skipping it costs that scan's colour and
      // nothing else -- its points keep the zeroed rgb that a point with no table entry gets.
      stats.scans_failed++;
      continue;
    }
    stats.scans_read++;
    const uint32_t scan_id = scan_id_from_path(dataset.scan_paths[s]);
    if (scan_id > k_max_scan_id || scan.point_count > k_max_point_index + 1)
    {
      error = fmt::format("{}: scan id {} / {} points do not fit the key packing", dataset.scan_paths[s], scan_id, scan.point_count);
      return false;
    }

    double delta_us = 0.0;
    const int32_t frame = dataset.frame_for_timestamp(scan.timestamp_camera, delta_us);
    if (frame < 0 || !load_frame(dataset, frame, cache, stats.images_loaded))
    {
      stats.scans_without_image++;
      continue;
    }

    have.assign(scan.point_count, 0);
    rgb.assign(scan.point_count * 3, 0);
    best.assign(scan.point_count, 1e30f);

    for (size_t c = 0; c < scan.cameras.size() && c < cache.images.size(); c++)
    {
      const auto &camera = scan.cameras[c];
      const image_t &image = cache.images[c];
      const float centre_x = float(image.width) * 0.5f;
      const float centre_y = float(image.height) * 0.5f;
      for (size_t i = 0; i < camera.point_index.size(); i++)
      {
        int32_t x = 0, y = 0;
        if (!map_pixel(camera.pixel_u[i], camera.pixel_v[i], image, x, y))
        {
          stats.pixels_out_of_range++;
          continue;
        }
        const size_t point = size_t(camera.point_index[i]);
        // 44k of a scan's 77k points are seen by more than one camera, so something has to break the
        // tie. Distance from the image centre: a Ladybug's lenses are worst at the edges, and unlike
        // "first camera wins" this does not depend on the order the cameras happen to be stored in.
        const float dx = float(x) - centre_x;
        const float dy = float(y) - centre_y;
        const float score = dx * dx + dy * dy;
        if (score >= best[point])
          continue;
        best[point] = score;
        const uint8_t *pixel = image.at(x, y);
        rgb[point * 3 + 0] = pixel[0];
        rgb[point * 3 + 1] = pixel[1];
        rgb[point * 3 + 2] = pixel[2];
        have[point] = 1;
      }
    }

    keys.clear();
    values.clear();
    keys.reserve(scan.point_count);
    values.reserve(scan.point_count * 3);
    for (size_t p = 0; p < scan.point_count; p++)
    {
      stats.points_seen++;
      if (!have[p])
        continue;
      keys.push_back(make_scan_key(scan_id, uint32_t(p)));
      values.push_back(rgb[p * 3 + 0]);
      values.push_back(rgb[p * 3 + 1]);
      values.push_back(rgb[p * 3 + 2]);
      stats.points_coloured++;
    }
    if (!keys.empty())
    {
      if (dew_converter_add_data_for_attribute(converter, DEW_ATTRIBUTE_RGB, uint32_t(strlen(DEW_ATTRIBUTE_RGB)), keys.data(), values.data(), keys.size()) == 0)
      {
        error = "the converter refused the colour values (was the attribute declared?)";
        return false;
      }
    }

    if ((s % 25) == 0 || s + 1 == count)
    {
      const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      fmt::print("\r  {:6.1f}s  {}/{} scans, {} points coloured, {} images read   ", seconds, s + 1, count, stats.points_coloured, stats.images_loaded);
    }
  }
  fmt::print("\n");
  return true;
}

} // namespace ford
