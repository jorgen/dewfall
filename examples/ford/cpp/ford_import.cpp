/* Import the IJRR Ford Campus Vision and Lidar dataset into a .dew dataset.
 *
 *     ford_import inspect <dataset-root> [--scan N]
 *
 * The importer is built in two passes, which is the whole point of the exercise:
 *
 *   1. INGEST the geometry, declaring a join key (scan id + point index) as an ordinary attribute.
 *   2. AMEND colour onto it later, joined on that key, without reconverting anything.
 *
 * That shape is what makes the same program expressible from Python: neither pass does per-point
 * work in the host language. Pass 1 fills the converter's own buffers; pass 2 hands over whole
 * key/value arrays. See examples/ford/python for the other half.
 *
 * `inspect` is the reconnaissance mode -- it reads one scan and prints what the file actually
 * contains, which is how the colour mapping below was established in the first place.
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <atomic>
#include <chrono>
#include <thread>

#include <fmt/format.h>

#include <dew/converter/converter.h>
#include <dew/core/default_attribute_names.h>

#include "ford_colour.hpp"
#include "ford_convert.hpp"
#include "ford_dataset.hpp"
#include "ppm.hpp"

namespace
{

// Runtime callbacks. They fire on converter threads, so they only touch atomics and stderr.
std::atomic<double> g_progress{0.0};
std::atomic<int> g_errors{0};

void on_progress(void *, float fraction)
{
  g_progress.store(double(fraction), std::memory_order_relaxed);
}

void on_warning(void *, const char *message)
{
  fmt::print(stderr, "\n  warning: {}\n", message ? message : "");
}

void on_error(void *, const struct dew_error_t *error)
{
  g_errors.fetch_add(1, std::memory_order_relaxed);
  int code = 0;
  const char *message = nullptr;
  size_t length = 0;
  if (error)
    dew_error_get_info(error, &code, &message, &length);
  fmt::print(stderr, "\n  error ({}): {}\n", code, std::string(message ? message : "", length));
}

int usage()
{
  fmt::print(stderr,
             "usage:\n"
             "  ford_import inspect <dataset-root> [--scan N]\n"
             "  ford_import convert <dataset-root> <output.dew> [--scans N] [--scale M] [--stride N] [--finalize]\n"
             "  ford_import colour  <dataset-root> <dataset.dew> [--scans N]\n");
  return 2;
}

// Where a Ford pixel coordinate lands in the stored PPM.
//
// The cameras are native 1232 x 1616 and the released per-camera PPMs are 1616 x 616, so the stored
// image is the native one transposed with its long axis halved. SCAN.Cam.pixels reflects the NATIVE
// frame: the first row spans the 1616 axis, the second spans 1..1232. Hence u maps straight through
// and v is halved.
//
// Established empirically rather than from documentation, which the release does not include: under
// this mapping the colour difference between 3D-nearest-neighbour points is ~30 against a random-
// pair baseline of ~190, and 100% of points land inside the image. Getting the axes wrong instead
// puts most points out of range.
struct pixel_mapping_t
{
  int32_t x = 0;
  int32_t y = 0;
  bool valid = false;
};

pixel_mapping_t map_pixel(double u, double v, const ford::image_t &image)
{
  pixel_mapping_t out;
  const double x = u - 1.0;       // MATLAB pixel coordinates are 1-based
  const double y = (v - 1.0) * 0.5;
  out.x = int32_t(x + 0.5);
  out.y = int32_t(y + 0.5);
  out.valid = image.contains(out.x, out.y);
  return out;
}

int inspect(const std::string &root, int scan_ordinal)
{
  std::string error;
  ford::dataset_t dataset;
  if (!ford::dataset_open(root, dataset, error))
  {
    fmt::print(stderr, "error: {}\n", error);
    return 1;
  }
  fmt::print("{}\n  {} scans, {} cameras, {} image timestamps\n", root, dataset.scan_paths.size(), dataset.camera_count, dataset.frame_timestamps.size());

  if (scan_ordinal < 0 || size_t(scan_ordinal) >= dataset.scan_paths.size())
    scan_ordinal = 0;
  const std::string &path = dataset.scan_paths[size_t(scan_ordinal)];

  ford::scan_t scan;
  if (!ford::scan_read(path, scan, error))
  {
    fmt::print(stderr, "error: {}\n", error);
    return 1;
  }

  double min_xyz[3] = {1e300, 1e300, 1e300};
  double max_xyz[3] = {-1e300, -1e300, -1e300};
  for (size_t i = 0; i < scan.point_count; i++)
  {
    const double *p = scan.point(i);
    for (int a = 0; a < 3; a++)
    {
      min_xyz[a] = p[a] < min_xyz[a] ? p[a] : min_xyz[a];
      max_xyz[a] = p[a] > max_xyz[a] ? p[a] : max_xyz[a];
    }
  }

  fmt::print("\n{}\n", path);
  fmt::print("  points          {}\n", scan.point_count);
  fmt::print("  extent          [{:.2f} {:.2f} {:.2f}] .. [{:.2f} {:.2f} {:.2f}]\n", min_xyz[0], min_xyz[1], min_xyz[2], max_xyz[0], max_xyz[1], max_xyz[2]);
  fmt::print("  pose (X_wv)     [{:.2f} {:.2f} {:.2f}] rpy [{:.4f} {:.4f} {:.4f}]\n", scan.pose[0], scan.pose[1], scan.pose[2], scan.pose[3], scan.pose[4], scan.pose[5]);
  fmt::print("  timestamp_laser {:.0f}\n", scan.timestamp_laser);
  fmt::print("  timestamp_camera{:.0f}\n", scan.timestamp_camera);

  double delta_us = 0.0;
  const int32_t frame = dataset.frame_for_timestamp(scan.timestamp_camera, delta_us);
  fmt::print("  -> frame {} ({:.1f} ms away)\n", frame, delta_us / 1000.0);

  // Every point should be seen by at least one camera. Counting the union is the cheap check that
  // the 1-based index conversion is right: an off-by-one shows up immediately as a gap.
  std::vector<uint8_t> covered(scan.point_count, 0);
  fmt::print("\n  camera   points   u range          v range          in image   coverage\n");
  for (size_t c = 0; c < scan.cameras.size(); c++)
  {
    const auto &camera = scan.cameras[c];
    ford::image_t image;
    const std::string image_file = dataset.image_path(int32_t(c), frame);
    const bool have_image = ford::ppm_read(image_file, image, error);

    double u_lo = 1e300, u_hi = -1e300, v_lo = 1e300, v_hi = -1e300;
    size_t inside = 0;
    for (size_t i = 0; i < camera.point_index.size(); i++)
    {
      u_lo = camera.pixel_u[i] < u_lo ? camera.pixel_u[i] : u_lo;
      u_hi = camera.pixel_u[i] > u_hi ? camera.pixel_u[i] : u_hi;
      v_lo = camera.pixel_v[i] < v_lo ? camera.pixel_v[i] : v_lo;
      v_hi = camera.pixel_v[i] > v_hi ? camera.pixel_v[i] : v_hi;
      covered[size_t(camera.point_index[i])] = 1;
      if (have_image && map_pixel(camera.pixel_u[i], camera.pixel_v[i], image).valid)
        inside++;
    }
    const size_t n = camera.point_index.size();
    fmt::print("  Cam{}   {:8}   [{:7.1f} {:7.1f}]  [{:7.1f} {:7.1f}]   {:5.1f}%   {}\n", c, n, u_lo, u_hi, v_lo, v_hi, n ? 100.0 * double(inside) / double(n) : 0.0,
               have_image ? fmt::format("{}x{}", image.width, image.height) : fmt::format("no image ({})", error));
  }
  size_t seen = 0;
  for (auto flag : covered)
    seen += flag;
  fmt::print("\n  points seen by at least one camera: {} / {} ({:.1f}%)\n", seen, scan.point_count, scan.point_count ? 100.0 * double(seen) / double(scan.point_count) : 0.0);
  return 0;
}


int convert(const std::string &root, const std::string &output, int scan_limit, double scale, int stride, bool finalize)
{
  std::string error;
  ford::dataset_t dataset;
  if (!ford::dataset_open(root, dataset, error))
  {
    fmt::print(stderr, "error: {}\n", error);
    return 1;
  }
  if (scan_limit > 0 && size_t(scan_limit) < dataset.scan_paths.size())
    dataset.scan_paths.resize(size_t(scan_limit));

  // The key packs the scan number into 12 bits, so a dataset numbered past 4095 needs a wider key.
  // Checked rather than assumed, because the failure mode is silent: an overflowing scan id would
  // alias onto another scan's points and the colour pass would quietly paint the wrong ones.
  for (const auto &path : dataset.scan_paths)
  {
    const uint32_t id = ford::scan_id_from_path(path);
    if (id > ford::k_max_scan_id)
    {
      fmt::print(stderr, "error: scan id {} (from {}) exceeds the {} that scan_key reserves 12 bits for\n", id, path, ford::k_max_scan_id);
      return 1;
    }
  }

  fmt::print("{}\n  {} scans to import\n", root, dataset.scan_paths.size());

  // Pass 0: where is the data? Sampling the trajectory rather than reading every scan, because the
  // box only has to bracket, and the padding covers what a sample misses.
  const auto probe_start = std::chrono::steady_clock::now();
  double world_min[3], world_max[3];
  size_t sampled = 0;
  if (!ford::ford_trajectory_bounds(dataset, size_t(stride < 1 ? 1 : stride), world_min, world_max, sampled, error))
  {
    fmt::print(stderr, "error: {}\n", error);
    return 1;
  }
  const double probe_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - probe_start).count();
  fmt::print("  trajectory from {} sampled poses in {:.1f}s\n", sampled, probe_s);
  fmt::print("  world box [{:.1f} {:.1f} {:.1f}] .. [{:.1f} {:.1f} {:.1f}]  ({:.0f} x {:.0f} x {:.0f} m)\n", world_min[0], world_min[1], world_min[2], world_max[0], world_max[1], world_max[2],
             world_max[0] - world_min[0], world_max[1] - world_min[1], world_max[2] - world_min[2]);

  ford::ford_configure_import(dataset, scale, world_min, world_max);

  dew_error_t *create_error = nullptr;
  auto *converter = dew_converter_create(output.c_str(), output.size(), dew_open_file_semantics_truncate, &create_error);
  if (!converter)
  {
    int code = 0;
    const char *message = nullptr;
    size_t length = 0;
    if (create_error)
      dew_error_get_info(create_error, &code, &message, &length);
    fmt::print(stderr, "error: cannot create {}: {}\n", output, std::string(message ? message : "", length));
    return 1;
  }

  // MUTABLE, so the dataset stays amendable: pass 2 adds colour to it without reconverting. It is
  // sealed by dew_converter_finalize once nothing more will be added.
  dew_converter_set_mutable(converter, 1);
  // Without this the key does not survive LOD -- the default keep-list is rgb/intensity/
  // classification, so coarse nodes would drop scan_key and only the leaves could be coloured.
  dew_converter_set_lod_all_attributes(converter, 1);
  ford::ford_install_callbacks(converter);

  std::vector<std::string> names = dataset.scan_paths;
  std::vector<dew_converter_str_buffer> buffers;
  buffers.reserve(names.size());
  for (const auto &name : names)
    buffers.push_back({name.c_str(), uint32_t(name.size())});

  const auto start = std::chrono::steady_clock::now();
  fmt::print("\nconverting to {} at {:g} m ...\n", output, scale);
  dew_converter_add_data_file(converter, buffers.data(), uint32_t(buffers.size()));
  dew_converter_runtime_callbacks_t runtime{};
  runtime.progress = on_progress;
  runtime.warning = on_warning;
  runtime.error = on_error;
  dew_converter_set_runtime_callbacks(converter, runtime, nullptr);

  // wait_idle blocks, so the progress line is driven from a watcher. It is also the only thing that
  // would notice a stall: a conversion that stops advancing shows a frozen scan count rather than
  // simply never returning.
  std::atomic<bool> finished{false};
  std::thread watcher([&] {
    size_t scans = 0, points = 0, failed = 0;
    while (!finished.load(std::memory_order_relaxed))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ford::ford_import_progress(scans, points, failed);
      const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      fmt::print("\r  {:6.1f}s  {} scans read, {} points, {:.0f}% converted   ", seconds, scans, points, 100.0 * g_progress.load(std::memory_order_relaxed));
    }
  });
  dew_converter_wait_idle(converter);
  finished.store(true, std::memory_order_relaxed);
  watcher.join();
  fmt::print("\n");

  size_t scans_done = 0, points_done = 0, scans_failed = 0;
  ford::ford_import_progress(scans_done, points_done, scans_failed);
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  // A per-file decode failure latches the converter's error status, but the points that DID land are
  // perfectly good -- so only an import that produced nothing is fatal. The Ford release ships at
  // least one truncated scan, and refusing the other 3816 over it would be absurd.
  const bool failed = scans_done == 0;
  // NOT finalized by default. finalize is what ENDS a dataset's amendable life -- it seals the trees
  // so they can be banded and evicted, and an amend has to read every unit it touches. Pass 2 is
  // what seals this dataset; --finalize is for an import that is not going to be amended.
  if (!failed && finalize)
    dew_converter_finalize(converter);

  dew_converter_stats_t stats{};
  const bool have_stats = dew_converter_get_compression_stats(converter, &stats);
  dew_converter_destroy(converter);

  if (failed)
  {
    fmt::print(stderr, "\nerror: conversion failed\n");
    return 1;
  }
  fmt::print("  {} scans, {} points in {:.1f}s ({:.0f} points/s)\n", scans_done, points_done, elapsed, elapsed > 0 ? double(points_done) / elapsed : 0.0);
  if (have_stats)
  {
    uint64_t raw = 0, packed = 0;
    for (uint32_t i = 0; i < stats.attribute_count; i++)
    {
      raw += stats.attributes[i].uncompressed_bytes;
      packed += stats.attributes[i].compressed_bytes;
      fmt::print("    {:<12} {:9.2f} MB -> {:8.2f} MB\n", stats.attributes[i].name, double(stats.attributes[i].uncompressed_bytes) / 1e6, double(stats.attributes[i].compressed_bytes) / 1e6);
    }
    fmt::print("    {:<12} {:9.2f} MB -> {:8.2f} MB\n", "total", double(raw) / 1e6, double(packed) / 1e6);
  }
  fmt::print("\ninspect it with:  dew info {}\n", output);
  return 0;
}

int colour(const std::string &root, const std::string &dataset_path, int scan_limit)
{
  std::string error;
  ford::dataset_t dataset;
  if (!ford::dataset_open(root, dataset, error))
  {
    fmt::print(stderr, "error: {}\n", error);
    return 1;
  }

  dew_error_t *create_error = nullptr;
  auto *converter = dew_converter_create(dataset_path.c_str(), dataset_path.size(), dew_open_file_semantics_open_existing, &create_error);
  if (!converter)
  {
    int code = 0;
    const char *message = nullptr;
    size_t length = 0;
    if (create_error)
      dew_error_get_info(create_error, &code, &message, &length);
    fmt::print(stderr, "error: cannot open {}: {}\n", dataset_path, std::string(message ? message : "", length));
    return 1;
  }

  // Amending requires a MUTABLE dataset: a finalized one has trees that have been banded and, in
  // destination mode, evicted, and an amend has to read every unit it touches. Pass 1 left it
  // mutable; saying so again is what a fresh process has to do after reopening.
  dew_converter_set_mutable(converter, 1);
  dew_converter_runtime_callbacks_t runtime{};
  runtime.warning = on_warning;
  runtime.error = on_error;
  dew_converter_set_runtime_callbacks(converter, runtime, nullptr);

  if (dew_converter_add_attribute(converter, DEW_ATTRIBUTE_RGB, uint32_t(strlen(DEW_ATTRIBUTE_RGB)), ford::k_key_attribute, uint32_t(strlen(ford::k_key_attribute)), dew_type_u8,
                                  dew_components_3) == 0)
  {
    fmt::print(stderr, "error: could not declare {} on {}\n", DEW_ATTRIBUTE_RGB, dataset_path);
    dew_converter_destroy(converter);
    return 1;
  }

  fmt::print("{}\n  sampling colour for {}\n", root, dataset_path);
  ford::colour_stats_t stats;
  const auto started = std::chrono::steady_clock::now();
  if (!ford::ford_colour_scans(dataset, converter, size_t(scan_limit > 0 ? scan_limit : 0), stats, error))
  {
    fmt::print(stderr, "error: {}\n", error);
    dew_converter_destroy(converter);
    return 1;
  }
  const double sample_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

  fmt::print("  {} scans read ({} unreadable, {} without a matching image), {} images\n", stats.scans_read, stats.scans_failed, stats.scans_without_image, stats.images_loaded);
  fmt::print("  {} of {} points coloured ({:.1f}%), {} pixels out of range\n", stats.points_coloured, stats.points_seen, stats.points_seen ? 100.0 * double(stats.points_coloured) / double(stats.points_seen) : 0.0,
             stats.pixels_out_of_range);
  fmt::print("  sampled in {:.1f}s\n", sample_s);

  // THE pass. One read and one write of one attribute per unit -- the unit's existing blobs are not
  // read, not recompressed and not moved.
  fmt::print("\ncommitting {} to every node ...\n", DEW_ATTRIBUTE_RGB);
  const auto commit_started = std::chrono::steady_clock::now();
  dew_converter_commit_attributes(converter);
  const double commit_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - commit_started).count();
  const bool failed = dew_converter_status(converter) == dew_conversion_status_error;
  if (!failed)
    dew_converter_finalize(converter);
  dew_converter_destroy(converter);
  if (failed)
  {
    fmt::print(stderr, "error: the amend failed\n");
    return 1;
  }
  fmt::print("  committed in {:.1f}s\n\ninspect it with:  dew info {}\n", commit_s, dataset_path);
  return 0;
}
} // namespace

int main(int argc, char **argv)
{
  if (argc < 3)
    return usage();
  // Unbuffered: an import is long-running and its progress is worth seeing as it happens,
  // including when the output is piped to a file.
  setvbuf(stdout, nullptr, _IONBF, 0);
  const std::string command = argv[1];
  const std::string root = argv[2];
  int scan_ordinal = 0;
  int scan_limit = 0;
  int stride = 32;
  double scale = 0.001;
  bool finalize = false;
  std::string output;
  int first_option = 3;
  if (command == "convert" || command == "colour")
  {
    if (argc < 4)
      return usage();
    output = argv[3];
    first_option = 4;
  }
  for (int i = first_option; i < argc; i++)
  {
    if (strcmp(argv[i], "--scan") == 0 && i + 1 < argc)
      scan_ordinal = atoi(argv[++i]);
    else if (strcmp(argv[i], "--scans") == 0 && i + 1 < argc)
      scan_limit = atoi(argv[++i]);
    else if (strcmp(argv[i], "--stride") == 0 && i + 1 < argc)
      stride = atoi(argv[++i]);
    else if (strcmp(argv[i], "--scale") == 0 && i + 1 < argc)
      scale = atof(argv[++i]);
    else if (strcmp(argv[i], "--finalize") == 0)
      finalize = true;
    else
      return usage();
  }
  if (command == "inspect")
    return inspect(root, scan_ordinal);
  if (command == "convert")
  {
    if (output.empty())
      return usage();
    return convert(root, output, scan_limit, scale, stride, finalize);
  }
  if (command == "colour")
  {
    if (output.empty())
      return usage();
    return colour(root, output, scan_limit);
  }
  return usage();
}
