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

#include <fmt/format.h>

#include "ford_dataset.hpp"
#include "ppm.hpp"

namespace
{

int usage()
{
  fmt::print(stderr, "usage: ford_import inspect <dataset-root> [--scan N]\n");
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
  for (int i = 3; i < argc; i++)
  {
    if (strcmp(argv[i], "--scan") == 0 && i + 1 < argc)
      scan_ordinal = atoi(argv[++i]);
    else
      return usage();
  }
  if (command == "inspect")
    return inspect(root, scan_ordinal);
  return usage();
}
