/* Pass 1 of the importer: ingest the geometry, carrying a join key.
 *
 * Nothing about colour happens here. The scans are converted once, with `scan_key` declared as an
 * ordinary attribute, and colour is joined onto that key afterwards (pass 2). That split is the
 * point of the exercise: the expensive part -- reading 25 GB of scans, transforming, sorting into a
 * morton octree and generating LOD -- happens once, and every later attribute is a cheap amend.
 */

#include "ford_convert.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>

#include <fmt/format.h>

#include <dew/converter/converter.h>
#include <dew/core/default_attribute_names.h>

#include "ford_dataset.hpp"

namespace ford
{
namespace
{

// The converter's file callbacks are plain C function pointers with no user context of their own --
// only `init` gets to hand back a per-file void*. Everything the callbacks need that is per-RUN
// rather than per-file therefore lives here. Written once before conversion starts and only read
// afterwards, from several converter threads at once.
struct import_context_t
{
  const dataset_t *dataset = nullptr;
  double offset[3] = {};
  double scale = 0.0;
  double world_min[3] = {};
  double world_max[3] = {};
};
import_context_t g_context;

const bool g_trace = std::getenv("DEW_FORD_TRACE") != nullptr;
std::mutex g_progress_mutex;
size_t g_scans_done = 0;
size_t g_points_done = 0;
size_t g_scans_failed = 0;

// Per-file state. Holds the decoded scan only while it is being drained -- see convert_data.
struct scan_state_t
{
  std::string path;
  uint32_t scan_id = 0;
  scan_t scan;
  bool loaded = false;
  size_t emitted = 0;
};

// SCAN.XYZ is in the VEHICLE frame -- every scan's centroid sits within a couple of metres of the
// origin while the vehicle drives a 450 m loop -- so each scan has to be placed by its pose or the
// whole dataset piles up on itself.
//
// The convention is R = Rx(roll) * Ry(pitch) * Rz(yaw - pi/2), which the release does not document.
// It was found by maximising map crispness (occupied voxels per point, at 0.3 m) over every Euler
// order, sign and quarter-turn offset, across a window of scans spanning a 64-degree turn: this
// scores 0.120 against 0.165 for translation alone and 0.199 for the worst candidate. The -pi/2 is
// the interesting part -- the pose's yaw is measured 90 degrees off the world frame's x axis.
//
// Roll and pitch are both about 0.02 rad here, so their ORDER relative to each other is not
// determined by the data (xyz and yxz tie exactly). Only the yaw term is pinned down.
void pose_rotation(const double (&pose)[6], double (&r)[9])
{
  const double roll = pose[3];
  const double pitch = pose[4];
  const double yaw = pose[5] - 1.5707963267948966;
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  // Rx * Ry * Rz, row-major.
  r[0] = cp * cy;
  r[1] = -cp * sy;
  r[2] = sp;
  r[3] = sr * sp * cy + cr * sy;
  r[4] = -sr * sp * sy + cr * cy;
  r[5] = -sr * cp;
  r[6] = -cr * sp * cy + sr * sy;
  r[7] = cr * sp * sy + sr * cy;
  r[8] = cr * cp;
}

void transform_point(const double (&r)[9], const double (&t)[3], const double *in, double *out)
{
  out[0] = r[0] * in[0] + r[1] * in[1] + r[2] * in[2] + t[0];
  out[1] = r[3] * in[0] + r[4] * in[1] + r[5] * in[2] + t[1];
  out[2] = r[6] * in[0] + r[7] * in[1] + r[8] * in[2] + t[2];
}

} // namespace

// Parse the scan number out of "Scan0075.mat". Using the FILE's number rather than an ordinal in
// our sorted list means the key is stable no matter which subset of scans is imported -- a key
// minted by a 100-scan trial run still identifies the same point in a full import.
uint32_t scan_id_from_path(const std::string &path)
{
  size_t end = path.find_last_of('.');
  if (end == std::string::npos)
    end = path.size();
  size_t begin = end;
  while (begin > 0 && path[begin - 1] >= '0' && path[begin - 1] <= '9')
    begin--;
  return begin == end ? 0u : uint32_t(strtoul(path.substr(begin, end - begin).c_str(), nullptr, 10));
}

namespace
{
dew_converter_file_pre_init_info_t ford_pre_init(const char *filename, size_t filename_size, dew_error_t **)
{
  dew_converter_file_pre_init_info_t info{};
  info.approximate_point_count = k_approximate_points_per_scan;
  info.found_point_count = 1;

  // REPORTING aabb_min IS NOT OPTIONAL, and leaving it out is not merely a missed optimisation.
  //
  // The converter dispatches inputs in rising min-morton order and derives the done-morton watermark
  // from it -- everything strictly below the watermark is final, which is what lets LOD passes
  // conclude, subtrees finalize, leaf collapse retire its chunks and checkpoints commit. The registry
  // is explicit about the alternative (input_data_source_registry.cpp): "A file without a found
  // pre-init min has input_order 0 -> boundary 0 -> no progress until it's done: conservative."
  //
  // With 3812 inputs and no min, the watermark sat at zero for the whole conversion. Nothing was ever
  // final, so all of the collapse and LOD work piled into a single terminal pass: 190 s of ingest
  // followed by 62 minutes of one-and-a-half cores, 11 GB of resident memory, and not one checkpoint.
  //
  // The min corner has to be a TRUE lower bound on every point of the scan, on every axis, or the
  // watermark overclaims and points below it are dropped. The scan's own extent reaches about 115 m
  // from the sensor, so the pose minus the (larger) bounds padding is comfortably safe.
  scan_t scan;
  std::string message;
  if (scan_read(std::string(filename, filename_size), scan, message))
  {
    for (int i = 0; i < 3; i++)
      info.aabb_min[i] = scan.pose[i] - k_bounds_padding;
    info.found_aabb_min = 1;
    info.approximate_point_count = scan.point_count;
    // The scan is decoded a second time in init, where the points are actually wanted. That is a
    // deliberate trade: pre_init runs on the converter's thread pool, so this pass is parallel and
    // costs wall-clock roughly once, and it buys a watermark that advances.
  }
  // 16 bytes on the wire (i32x3 + u32) but reported higher, because this number is what the
  // converter's 1 GB read/sort budget divides by to decide how many inputs to have in flight. The
  // true cost of an input here includes decoding a 7 MB MATLAB payload, and letting 800 of those
  // run at once would be the memory blowup the budget exists to prevent.
  info.approximate_point_size_bytes = 96;
  info.input_file_size_bytes = k_approximate_scan_bytes;
  info.scale[0] = info.scale[1] = info.scale[2] = g_context.scale;
  info.found_scale = 1;
  return info;
}

void ford_init(const char *filename, size_t filename_size, dew_converter_header_t *header, dew_attributes_t *attributes, void **user_ptr, dew_error_t **error)
{
  auto *state = new scan_state_t();
  state->path.assign(filename, filename_size);
  state->scan_id = scan_id_from_path(state->path);

  // Decoded HERE rather than lazily in convert_data, because header.point_count must be the file's
  // EXACT count -- the converter sizes an input's bookkeeping from it, and declaring 77000 while
  // emitting the true 77276 wedged the conversion. A scan's count is only knowable by decoding it,
  // so the scan is held from here until it has been drained; pre_init's inflated
  // approximate_point_size_bytes is what bounds how many are held at once.
  std::string message;
  if (!scan_read(state->path, state->scan, message))
  {
    *error = dew_error_create();
    dew_error_set_info(*error, 1, message.c_str(), message.size());
    {
      std::lock_guard<std::mutex> lock(g_progress_mutex);
      g_scans_failed++;
    }
    delete state;
    *user_ptr = nullptr;
    return;
  }
  state->loaded = true;
  if (g_trace)
    fmt::print(stderr, "[ford] init  {} points={}\n", state->path, state->scan.point_count);

  header->point_count = state->scan.point_count;
  for (int i = 0; i < 3; i++)
  {
    header->offset[i] = g_context.offset[i];
    header->scale[i] = g_context.scale;
    // The DATASET-wide box, for every input. It only has to bracket the file's data, and a
    // per-scan box would mean decoding the scan here just to measure it -- then either holding it
    // (hundreds of megabytes across the inputs in flight) or decoding it a second time in
    // convert_data. The octree is sized from the union of these anyway.
    header->min[i] = g_context.world_min[i];
    header->max[i] = g_context.world_max[i];
  }

  dew_attributes_add_attribute(attributes, DEW_ATTRIBUTE_XYZ, uint32_t(strlen(DEW_ATTRIBUTE_XYZ)), dew_type_i32, dew_components_3);
  dew_attributes_add_attribute(attributes, k_key_attribute, uint32_t(strlen(k_key_attribute)), dew_type_u32, dew_components_1);
  *user_ptr = state;
}

void ford_convert_data(void *user_ptr, const dew_converter_header_t *, const dew_attribute_t *, uint32_t, uint32_t max_points, dew_blob_t *buffers, uint32_t buffer_count, uint32_t *points_read,
                       uint8_t *done, dew_error_t **error)
{
  auto *state = static_cast<scan_state_t *>(user_ptr);
  *points_read = 0;
  *done = 0;
  if (!state || !state->loaded)
  {
    *done = 1;
    return;
  }
  (void)error;

  const size_t remaining = state->scan.point_count - state->emitted;
  const uint32_t n = uint32_t(remaining < max_points ? remaining : max_points);

  double rotation[9];
  double translation[3] = {state->scan.pose[0], state->scan.pose[1], state->scan.pose[2]};
  pose_rotation(state->scan.pose, rotation);

  auto *xyz = static_cast<int32_t *>(buffers[0].data);
  auto *key = buffer_count >= 2 ? static_cast<uint32_t *>(buffers[1].data) : nullptr;
  const double inv_scale = 1.0 / g_context.scale;
  for (uint32_t i = 0; i < n; i++)
  {
    const size_t index = state->emitted + i;
    double world[3];
    transform_point(rotation, translation, state->scan.point(index), world);
    for (int a = 0; a < 3; a++)
    {
      // std::lround, not a cast: a cast truncates toward zero, which biases every negative
      // coordinate half a quantum toward the origin.
      const double quantized = (world[a] - g_context.offset[a]) * inv_scale;
      xyz[i * 3 + a] = int32_t(std::lround(quantized));
    }
    if (key)
      key[i] = make_scan_key(state->scan_id, uint32_t(index));
  }

  if (g_trace)
    fmt::print(stderr, "[ford] chunk {} n={} emitted={}/{} max_points={}\n", state->path, n, state->emitted + n, state->scan.point_count, max_points);
  state->emitted += n;
  *points_read = n;
  if (state->emitted >= state->scan.point_count)
  {
    *done = 1;
    // Freed as soon as it is drained rather than waiting for destroy_user_ptr, so a decoded scan
    // stops counting against memory the moment its points are in the converter's hands.
    state->scan = scan_t{};
    state->loaded = false;
    std::lock_guard<std::mutex> lock(g_progress_mutex);
    g_scans_done++;
    g_points_done += state->emitted;
  }
}

void ford_destroy_user_ptr(void *user_ptr)
{
  delete static_cast<scan_state_t *>(user_ptr);
}

} // namespace

bool ford_trajectory_bounds(const dataset_t &dataset, size_t stride, double (&world_min)[3], double (&world_max)[3], size_t &sampled, std::string &error)
{
  for (int a = 0; a < 3; a++)
  {
    world_min[a] = 1e300;
    world_max[a] = -1e300;
  }
  sampled = 0;
  if (stride == 0)
    stride = 1;
  size_t unreadable = 0;
  for (size_t i = 0; i < dataset.scan_paths.size(); i += stride)
  {
    scan_t scan;
    if (!scan_read(dataset.scan_paths[i], scan, error))
    {
      // SKIPPED, not fatal. The Ford release contains several truncated scans, and one of them
      // landing on the sampling stride must not stop the import before it starts -- which is exactly
      // what it did the first time this was run over the whole dataset. The bounds only have to
      // BRACKET the data, and the padding below already covers far more than one missing pose.
      unreadable++;
      continue;
    }
    for (int a = 0; a < 3; a++)
    {
      world_min[a] = std::fmin(world_min[a], scan.pose[a]);
      world_max[a] = std::fmax(world_max[a], scan.pose[a]);
    }
    sampled++;
  }
  if (!sampled)
  {
    error = fmt::format("no scans could be read for the trajectory ({} unreadable)", unreadable);
    return false;
  }
  error.clear();
  // The box has to BRACKET the data, and the data is the sensor's reach around a trajectory we only
  // sampled. The padding covers both: the velodyne's ~110 m range, plus however far the vehicle
  // travelled between two samples. Being generous costs a little octree depth and nothing else;
  // being tight enough to clip a point would be a silent error.
  for (int a = 0; a < 3; a++)
  {
    world_min[a] -= k_bounds_padding;
    world_max[a] += k_bounds_padding;
  }
  return true;
}

void ford_configure_import(const dataset_t &dataset, double scale, const double (&world_min)[3], const double (&world_max)[3])
{
  g_context.dataset = &dataset;
  g_context.scale = scale;
  for (int a = 0; a < 3; a++)
  {
    g_context.world_min[a] = world_min[a];
    g_context.world_max[a] = world_max[a];
    // Quantized coordinates are (world - offset) / scale as int32. Anchoring the offset at the box
    // minimum keeps every value positive and as small as the data allows: a 700 m extent at 1 mm is
    // 700k, six orders of magnitude inside int32's range, which leaves the delta coding in the
    // compressor working on small numbers.
    g_context.offset[a] = world_min[a];
  }
  g_scans_done = 0;
  g_points_done = 0;
  g_scans_failed = 0;
}

void ford_install_callbacks(dew_converter_t *converter)
{
  dew_converter_file_convert_callbacks_t callbacks{};
  callbacks.pre_init = ford_pre_init;
  callbacks.init = ford_init;
  callbacks.convert_data = ford_convert_data;
  callbacks.destroy_user_ptr = ford_destroy_user_ptr;
  dew_converter_set_file_converter_callbacks(converter, callbacks);
}

void ford_import_progress(size_t &scans_done, size_t &points_done, size_t &scans_failed)
{
  std::lock_guard<std::mutex> lock(g_progress_mutex);
  scans_done = g_scans_done;
  points_done = g_points_done;
  scans_failed = g_scans_failed;
}

} // namespace ford
