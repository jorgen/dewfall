/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2026  Jørgen Lind
**
** This program is free software: you can redistribute it and/or modify
** it under the terms of the GNU Affero General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU Affero General Public License for more details.
**
** You should have received a copy of the GNU Affero General Public License
** along with this program.  If not, see <https://www.gnu.org/licenses/>.
************************************************************************/

// The renderer, driven with a recording consumer instead of OpenGL.
//
// Until now the render path had no automated test at all, and "it compiles" was the only signal --
// which is why every change to it has had to be checked by eye, and why moving the streaming source
// off the write pipeline has stayed parked. Nothing about the renderer actually needs a GPU, though:
// it is callback-driven by design and hands the consumer buffer lifecycle events plus a list of draw
// groups. A consumer that records instead of drawing is a complete, honest test harness.
//
// What these pin down:
//
//   * a camera pointed at data eventually produces point draw groups, and every buffer they name was
//     created and initialized first -- a draw group referencing an unknown handle is what a real
//     consumer would crash on;
//   * the buffer lifecycle balances. Every created buffer is destroyed by teardown; a leak here is a
//     GPU leak in the real consumer, invisible until a session runs long;
//   * frustum culling is real: pointing the camera away drops the point groups;
//   * moving the camera closer refines rather than merely redrawing -- more points get uploaded.
//
// The dataset is synthetic and built by the test. It goes to a temp .dew file rather than mem://,
// which would have been the natural choice: vio's create_io_manager makes a FRESH memory_io_manager_t
// per call and ignores the name, so a converter and a reader on the same mem:// URL get two different
// stores. Worth knowing before reaching for it again.

#include <doctest/doctest.h>

#include <dew/converter/converter.h>
#include <dew/converter/converter_data_source.h>
#include <dew/core/default_attribute_names.h>
#include <dew/core/error.h>
#include <dew/render/camera.h>
#include <dew/render/renderer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{

// ---------------------------------------------------------------------------------------------
// Synthetic clouds. Two shapes, because they exercise different halves of the renderer:
//
//   uniform   -- a filled cube. Every leaf is compact, so the octree is balanced and the streaming
//                path is all that runs.
//   unbalanced -- a sparse cloud over a huge volume PLUS one dense blob. The sparse points make
//                leaves that cover an enormous morton range while staying under the point limit, and
//                the blob forces deep subdivision on one branch. That combination is what promotes a
//                leaf to a VIRTUAL source, so it is the only way to reach that code from a test.
constexpr double k_scale = 0.001;

struct source_t
{
  std::vector<int32_t> xyz;
  std::vector<uint16_t> intensity;
  int32_t max_coord = 0;
};

source_t g_source;
uint32_t g_emitted = 0;

void add_point(source_t &out, int32_t x, int32_t y, int32_t z)
{
  out.xyz.push_back(x);
  out.xyz.push_back(y);
  out.xyz.push_back(z);
  out.intensity.push_back(uint16_t((x + y + z) & 0xFFFF));
  out.max_coord = std::max({out.max_coord, x, y, z});
}

// A filled cube: balanced octree, compact leaves, no virtual promotion.
source_t make_uniform_source()
{
  constexpr int32_t side = 40;
  source_t out;
  for (int32_t z = 0; z < side; z++)
    for (int32_t y = 0; y < side; y++)
      for (int32_t x = 0; x < side; x++)
        add_point(out, x, y, z);
  return out;
}

// Deliberately lopsided. A leaf is promoted to a virtual source only when it is a leaf, holds more
// than virtual_min_points (256), and its points span more than lod_quantize_full_detail_level (9)
// morton levels -- i.e. it covers a wide region without ever exceeding the point limit. Scattering a
// few thousand points across 2^18 grid units gives exactly that; the dense blob then forces the
// opposite extreme on one branch so the tree really is unbalanced rather than merely sparse.
source_t make_unbalanced_source()
{
  constexpr int32_t span = 1 << 18;
  source_t out;
  // Sparse background. A cheap deterministic hash rather than <random>, so the dataset is identical
  // on every platform and a failure is reproducible.
  uint32_t state = 12345;
  auto next = [&state]() {
    state = state * 1664525u + 1013904223u;
    return state;
  };
  for (int i = 0; i < 12000; i++)
    add_point(out, int32_t(next() % span), int32_t(next() % span), int32_t(next() % span));

  // Dense blob in one corner: forces that branch to subdivide far past everything else.
  constexpr int32_t blob_side = 40;
  constexpr int32_t blob_origin = span / 2;
  for (int32_t z = 0; z < blob_side; z++)
    for (int32_t y = 0; y < blob_side; y++)
      for (int32_t x = 0; x < blob_side; x++)
        add_point(out, blob_origin + x, blob_origin + y, blob_origin + z);
  return out;
}

uint32_t source_point_count()
{
  return uint32_t(g_source.intensity.size());
}

dew_converter_file_pre_init_info_t pre_init(const char *, size_t, dew_error_t **)
{
  dew_converter_file_pre_init_info_t info{};
  info.approximate_point_count = source_point_count();
  info.found_point_count = 1;
  info.approximate_point_size_bytes = 16;
  info.scale[0] = info.scale[1] = info.scale[2] = k_scale;
  info.found_scale = 1;
  return info;
}

void init(const char *, size_t, dew_converter_header_t *header, dew_attributes_t *attributes, void **, dew_error_t **)
{
  header->point_count = source_point_count();
  for (int i = 0; i < 3; i++)
  {
    header->offset[i] = 0.0;
    header->scale[i] = k_scale;
    header->min[i] = 0.0;
    header->max[i] = double(g_source.max_coord) * k_scale;
  }
  dew_attributes_add_attribute(attributes, DEW_ATTRIBUTE_XYZ, uint32_t(strlen(DEW_ATTRIBUTE_XYZ)), dew_type_i32, dew_components_3);
  dew_attributes_add_attribute(attributes, DEW_ATTRIBUTE_INTENSITY, uint32_t(strlen(DEW_ATTRIBUTE_INTENSITY)), dew_type_u16, dew_components_1);
  g_emitted = 0;
}

void convert_data(void *, const dew_converter_header_t *, const dew_attribute_t *, uint32_t, uint32_t max_points, dew_blob_t *buffers, uint32_t buffer_count, uint32_t *points_read, uint8_t *done,
                  dew_error_t **)
{
  const uint32_t total = source_point_count();
  const uint32_t remaining = total - g_emitted;
  const uint32_t n = remaining < max_points ? remaining : max_points;
  if (buffer_count >= 1)
    memcpy(buffers[0].data, g_source.xyz.data() + size_t(g_emitted) * 3, size_t(n) * 3 * sizeof(int32_t));
  if (buffer_count >= 2)
    memcpy(buffers[1].data, g_source.intensity.data() + g_emitted, size_t(n) * sizeof(uint16_t));
  g_emitted += n;
  *points_read = n;
  *done = g_emitted >= total ? 1 : 0;
}

bool build_dataset(const char *path, source_t source, uint32_t node_point_limit)
{
  g_source = std::move(source);
  std::remove(path);
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, uint32_t(strlen(path)), dew_open_file_semantics_truncate, &error);
  if (!converter)
  {
    if (error)
      dew_error_destroy(error);
    return false;
  }
  dew_converter_file_convert_callbacks_t callbacks{};
  callbacks.pre_init = pre_init;
  callbacks.init = init;
  callbacks.convert_data = convert_data;
  dew_converter_set_file_converter_callbacks(converter, callbacks);
  // Force a real octree. With the default limit everything lands in one node and there is no LOD
  // structure for the walker to descend, which would make every assertion below trivially true.
  dew_converter_set_node_point_limit(converter, node_point_limit);

  dew_converter_str_buffer name{"synthetic", 9};
  dew_converter_add_data_file(converter, &name, 1);
  dew_converter_wait_idle(converter);
  const bool ok = dew_converter_status(converter) == dew_conversion_status_completed;
  dew_converter_destroy(converter);
  return ok;
}

// ---------------------------------------------------------------------------------------------
// The consumer under test: records instead of drawing.
struct recording_consumer_t
{
  struct buffer_record_t
  {
    dew_buffer_type_t type = dew_buffer_type_vertex;
    int initialized_bytes = 0;
    int initialize_calls = 0;
    int modify_calls = 0;
    bool destroyed = false;
  };

  // Handles are dense small integers rather than pointers, so a stale handle shows up as a lookup
  // miss instead of a use-after-free that happens to still be mapped.
  std::unordered_map<uintptr_t, buffer_record_t> buffers;
  uintptr_t next_handle = 1;

  uint32_t create_calls = 0;
  uint32_t initialize_calls = 0;
  uint32_t modify_calls = 0;
  uint32_t destroy_calls = 0;
  uint64_t initialized_bytes = 0;

  [[nodiscard]] size_t live_buffers() const
  {
    size_t live = 0;
    for (const auto &entry : buffers)
      if (!entry.second.destroyed)
        live++;
    return live;
  }

  [[nodiscard]] bool known(void *user_ptr) const
  {
    auto it = buffers.find(reinterpret_cast<uintptr_t>(user_ptr));
    return it != buffers.end() && !it->second.destroyed;
  }

  [[nodiscard]] bool initialized(void *user_ptr) const
  {
    auto it = buffers.find(reinterpret_cast<uintptr_t>(user_ptr));
    return it != buffers.end() && it->second.initialize_calls > 0;
  }

  static recording_consumer_t *self(void *user_ptr) { return static_cast<recording_consumer_t *>(user_ptr); }

  static void on_create(dew_renderer_t *, void *user_ptr, dew_buffer_type_t type, void **buffer_user_ptr)
  {
    auto *consumer = self(user_ptr);
    const uintptr_t handle = consumer->next_handle++;
    consumer->buffers[handle] = buffer_record_t{type, 0, 0, 0, false};
    consumer->create_calls++;
    *buffer_user_ptr = reinterpret_cast<void *>(handle);
  }

  static void on_initialize(dew_renderer_t *, void *user_ptr, dew_buffer_t *, void *buffer_user_ptr, dew_type_t, dew_components_t, int buffer_size, void *)
  {
    auto *consumer = self(user_ptr);
    auto it = consumer->buffers.find(reinterpret_cast<uintptr_t>(buffer_user_ptr));
    REQUIRE(it != consumer->buffers.end()); // initialize before create would be a lifecycle bug
    it->second.initialized_bytes = buffer_size;
    it->second.initialize_calls++;
    consumer->initialize_calls++;
    consumer->initialized_bytes += uint64_t(buffer_size < 0 ? 0 : buffer_size);
  }

  static void on_modify(dew_renderer_t *, void *user_ptr, dew_buffer_t *, void *buffer_user_ptr, int, int, void *)
  {
    auto *consumer = self(user_ptr);
    auto it = consumer->buffers.find(reinterpret_cast<uintptr_t>(buffer_user_ptr));
    REQUIRE(it != consumer->buffers.end());
    it->second.modify_calls++;
    consumer->modify_calls++;
  }

  static void on_destroy(dew_renderer_t *, void *user_ptr, void *buffer_user_ptr)
  {
    auto *consumer = self(user_ptr);
    auto it = consumer->buffers.find(reinterpret_cast<uintptr_t>(buffer_user_ptr));
    REQUIRE(it != consumer->buffers.end()); // destroying something never created
    REQUIRE(!it->second.destroyed);         // double destroy
    it->second.destroyed = true;
    consumer->destroy_calls++;
  }

  dew_renderer_callbacks_t callbacks()
  {
    dew_renderer_callbacks_t out{};
    out.create_buffer = &recording_consumer_t::on_create;
    out.initialize_buffer = &recording_consumer_t::on_initialize;
    out.modify_buffer = &recording_consumer_t::on_modify;
    out.destroy_buffer = &recording_consumer_t::on_destroy;
    return out;
  }
};

// What one frame produced, flattened for assertions.
struct frame_summary_t
{
  int group_count = 0;
  int point_group_count = 0;
  uint64_t point_draw_size = 0;
  std::vector<dew_draw_type_t> types;
};

bool is_point_group(dew_draw_type_t type)
{
  return type == dew_dyn_points_1 || type == dew_dyn_points_3 || type == dew_dyn_points_crossfade || type == dew_flat_points;
}

frame_summary_t run_frame(dew_renderer_t *renderer, dew_camera_t *camera, recording_consumer_t &consumer)
{
  frame_summary_t summary;
  dew_frame_t frame = dew_renderer_frame(renderer, camera);
  summary.group_count = frame.to_render_size;
  for (int i = 0; i < frame.to_render_size; i++)
  {
    const auto &group = frame.to_render[i];
    summary.types.push_back(group.draw_type);
    // The contract a real consumer depends on: every handle in a draw group is one the renderer
    // asked us to create and then initialize. A group naming an unknown or destroyed handle is
    // precisely what would crash a GL backend.
    for (int b = 0; b < group.buffers_size; b++)
    {
      if (!group.buffers[b].user_ptr)
        continue;
      REQUIRE(consumer.known(group.buffers[b].user_ptr));
      REQUIRE(consumer.initialized(group.buffers[b].user_ptr));
    }
    if (is_point_group(group.draw_type) && group.draw_size > 0)
    {
      summary.point_group_count++;
      summary.point_draw_size += uint64_t(group.draw_size);
    }
  }
  return summary;
}

// Every wait below is bounded by the WALL CLOCK, never by a frame count.
//
// A frame with nothing to do costs microseconds: add_to_frame walks the tree, schedules IO and
// returns, while the work it is waiting for happens elsewhere -- the blob read on the storage loop,
// the morton decode on a convert_pool worker, and for anything below the root an extra async hop to
// load the sub-tree. So "400 frames" was never a deadline, it was however fast this machine spins a
// near-empty loop, and the spin steals the CPU from the very threads it is waiting on. Under ASan
// (slower decode, fewer vCPUs) that window expired before the first node landed and the test read an
// empty pipeline as a renderer bug. Pausing a millisecond per frame hands those threads the CPU and
// makes the bound real time. The predicates are unchanged, so a genuine regression still fails -- it
// just takes the timeout to say so.
constexpr auto k_wait_timeout = std::chrono::seconds(30);
constexpr auto k_frame_pause = std::chrono::milliseconds(1);

// Streaming is asynchronous: the first frames walk the tree and issue loads, and points appear over
// the following ones. So "render until it settles" is the only honest way to observe a steady state.
frame_summary_t render_until_points(dew_renderer_t *renderer, dew_camera_t *camera, recording_consumer_t &consumer)
{
  const auto deadline = std::chrono::steady_clock::now() + k_wait_timeout;
  frame_summary_t last;
  for (int i = 0;; i++)
  {
    last = run_frame(renderer, camera, consumer);
    if (last.point_draw_size > 0 && i > 8)
      return last;
    if (std::chrono::steady_clock::now() >= deadline)
      return last;
    std::this_thread::sleep_for(k_frame_pause);
  }
}

frame_summary_t render_frames(dew_renderer_t *renderer, dew_camera_t *camera, recording_consumer_t &consumer, int frames)
{
  frame_summary_t last;
  for (int i = 0; i < frames; i++)
    last = run_frame(renderer, camera, consumer);
  return last;
}

// Render until `predicate` holds, or give up.
//
// Everything about this pipeline is asynchronous -- reads land on the storage loop, decodes on a
// worker, uploads on the next frame, and departed nodes fade out over several more. Asserting after a
// FIXED number of frames therefore passes or fails on timing, which is exactly what an earlier version
// of these tests did: same binary, two runs, two answers. Waiting on the condition instead makes the
// bound a timeout rather than a guess, and a genuine regression still fails because the predicate
// never comes true.
template <typename Predicate> frame_summary_t render_until(dew_renderer_t *renderer, dew_camera_t *camera, recording_consumer_t &consumer, Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + k_wait_timeout;
  frame_summary_t last;
  for (;;)
  {
    last = run_frame(renderer, camera, consumer);
    if (predicate(last))
      return last;
    if (std::chrono::steady_clock::now() >= deadline)
      return last;
    std::this_thread::sleep_for(k_frame_pause);
  }
}

const char *k_path = "render_callback_test.dew";
const char *k_unbalanced_path = "render_callback_unbalanced.dew";

// Everything a test needs, torn down in the order a real consumer must use: data source first (it
// releases its GPU buffers), then the renderer.
struct render_fixture_t
{
  explicit render_fixture_t(const char *path = k_path, source_t source = make_uniform_source(), uint32_t node_point_limit = 2000)
  {
    REQUIRE(build_dataset(path, std::move(source), node_point_limit));
    renderer = dew_renderer_create();
    REQUIRE(renderer);
    dew_renderer_set_callback(renderer, consumer.callbacks(), &consumer);

    // dew_error_t is opaque, so the handle has to be allocated rather than declared.
    dew_error_t *error = dew_error_create();
    data_source = dew_converter_data_source_create(path, uint32_t(strlen(path)), error, renderer);
    REQUIRE(data_source);
    dew_error_destroy(error);

    dew_converter_data_source_set_viewport(data_source, 800, 600);
    // Deliberately does NOT set a rendered attribute: the data source is supposed to pick one, and
    // this dataset (xyz + intensity) is exactly the shape that used to defeat its selection.
    dew_renderer_add_data_source(renderer, dew_converter_data_source_get(data_source));

    camera = dew_camera_create();
    REQUIRE(camera);
    dew_camera_set_perspective(camera, 45, 800, 600, 0.01, 10000);

    dew_converter_data_source_get_tight_aabb(data_source, aabb_min, aabb_max);
    for (int i = 0; i < 3; i++)
      center[i] = (aabb_min[i] + aabb_max[i]) * 0.5;
    extent = std::max({aabb_max[0] - aabb_min[0], aabb_max[1] - aabb_min[1], aabb_max[2] - aabb_min[2]});
    if (extent <= 0)
      extent = 1.0;
  }

  ~render_fixture_t()
  {
    if (camera)
      dew_camera_destroy(camera);
    if (data_source)
      dew_converter_data_source_destroy(data_source);
    if (renderer)
      dew_renderer_destroy(renderer);
  }

  void look_from(double dx, double dy, double dz, double distance_factor)
  {
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double d = extent * distance_factor / (length > 0 ? length : 1.0);
    const double eye[3] = {center[0] + dx * d, center[1] + dy * d, center[2] + dz * d};
    const double up[3] = {0, 0, 1};
    dew_camera_look_at(camera, eye, center, up);
  }

  recording_consumer_t consumer;
  dew_renderer_t *renderer = nullptr;
  dew_converter_data_source_t *data_source = nullptr;
  dew_camera_t *camera = nullptr;
  double aabb_min[3] = {0, 0, 0};
  double aabb_max[3] = {0, 0, 0};
  double center[3] = {0, 0, 0};
  double extent = 1.0;
};

} // namespace

TEST_CASE("render: a camera on the data produces point draw groups over consistent buffers")
{
  render_fixture_t fixture;
  fixture.look_from(1, 1, 1, 2.0);

  auto summary = render_until_points(fixture.renderer, fixture.camera, fixture.consumer);

  // The headline: the renderer asked us to draw actual points. run_frame has already asserted that
  // every buffer handle in every group was created and initialized first.
  MESSAGE("point groups: ", summary.point_group_count, "  points drawn: ", summary.point_draw_size, "  buffers created: ", fixture.consumer.create_calls, "  uploaded bytes: ",
          fixture.consumer.initialized_bytes);
  REQUIRE(summary.point_group_count > 0);
  REQUIRE(summary.point_draw_size > 0);
  REQUIRE(fixture.consumer.create_calls > 0);
  REQUIRE(fixture.consumer.initialize_calls > 0);
  REQUIRE(fixture.consumer.initialized_bytes > 0);

  // Points are streamed per node, so a cube this size must arrive as more than one buffer -- a single
  // one would mean the octree was never subdivided and the walk is not doing its job.
  REQUIRE(fixture.consumer.create_calls > 1);
}

TEST_CASE("render: pointing the camera away drops the point groups")
{
  // Frustum culling, observed rather than assumed. If the walker ignored the frustum this would draw
  // exactly as much with the data behind the camera as in front of it.
  render_fixture_t fixture;
  fixture.look_from(1, 1, 1, 2.0);
  auto facing = render_until_points(fixture.renderer, fixture.camera, fixture.consumer);
  REQUIRE(facing.point_draw_size > 0);

  // Same position, looking the other way.
  const double d = fixture.extent * 2.0 / std::sqrt(3.0);
  const double eye[3] = {fixture.center[0] + d, fixture.center[1] + d, fixture.center[2] + d};
  const double away[3] = {eye[0] + d, eye[1] + d, eye[2] + d};
  const double up[3] = {0, 0, 1};
  dew_camera_look_at(fixture.camera, eye, away, up);

  // Wait for the fade-out rather than assuming a frame count: departed nodes are crossfaded, so the
  // count decays over several frames.
  auto behind = render_until(fixture.renderer, fixture.camera, fixture.consumer, [](const frame_summary_t &s) { return s.point_draw_size == 0; });
  MESSAGE("facing: ", facing.point_draw_size, "  looking away: ", behind.point_draw_size);
  REQUIRE(behind.point_draw_size == 0);
}

TEST_CASE("render: moving closer refines the cloud")
{
  // The streaming contract: a nearer camera means a finer LOD frontier, which means more uploaded
  // points -- not merely the same buffers redrawn.
  // Measured on the FRONTIER (points actually drawn), not on consumer.initialized_bytes. That
  // counter is cumulative for the whole session and re-counts every re-upload, so a node that fades
  // out and back in inflates it without the frontier changing at all -- it never measured "how much
  // the far view needed". Under ASan the far view churned enough to reach 1088144 bytes on its own,
  // the close view then added nothing new, and the test failed with 1088144 > 1088144. Points drawn
  // cannot be inflated that way, and it is what the contract above actually claims.
  render_fixture_t fixture;
  fixture.look_from(1, 1, 1, 6.0);
  render_until_points(fixture.renderer, fixture.camera, fixture.consumer);
  // Let the far view reach a steady state: stop once the frontier stops growing for a stretch of
  // real time. Frames, again, are not a clock -- 30 quiet frames is microseconds here and seconds
  // under a sanitizer, which is why the two builds settled at wildly different points.
  uint64_t settled = 0;
  auto quiet_since = std::chrono::steady_clock::now();
  auto far_away = render_until(fixture.renderer, fixture.camera, fixture.consumer, [&](const frame_summary_t &s) {
    if (s.point_draw_size != settled)
    {
      settled = s.point_draw_size;
      quiet_since = std::chrono::steady_clock::now();
      return false;
    }
    return s.point_draw_size > 0 && std::chrono::steady_clock::now() - quiet_since >= std::chrono::milliseconds(500);
  });
  REQUIRE(far_away.point_draw_size > 0);
  const uint64_t points_when_far = far_away.point_draw_size;

  fixture.look_from(1, 1, 1, 0.9);
  auto close_up = render_until(fixture.renderer, fixture.camera, fixture.consumer,
                               [&](const frame_summary_t &s) { return s.point_draw_size > points_when_far; });

  // A finer frontier: the closer view draws strictly more points than the far one.
  MESSAGE("points drawn far: ", points_when_far, "  after moving close: ", close_up.point_draw_size,
          "  (uploaded bytes far/close: ", fixture.consumer.initialized_bytes, ")");
  REQUIRE(close_up.point_draw_size > points_when_far);
}

TEST_CASE("render: buffer lifecycle balances across a full session")
{
  render_fixture_t fixture;
  fixture.look_from(1, 1, 1, 1.5);
  render_until_points(fixture.renderer, fixture.camera, fixture.consumer);
  render_frames(fixture.renderer, fixture.camera, fixture.consumer, 40);

  REQUIRE(fixture.consumer.create_calls > 0);
  REQUIRE(fixture.consumer.live_buffers() > 0);

  // Tear the data source down explicitly, which is what releases its buffers, and then check nothing
  // it created is still outstanding.
  dew_converter_data_source_destroy(fixture.data_source);
  fixture.data_source = nullptr;

  MESSAGE("buffers created: ", fixture.consumer.create_calls, "  destroyed: ", fixture.consumer.destroy_calls);
  REQUIRE(fixture.consumer.destroy_calls > 0);
  REQUIRE(fixture.consumer.destroy_calls == fixture.consumer.create_calls);
  REQUIRE(fixture.consumer.live_buffers() == 0);
}

TEST_CASE("render: a frame with no data source produces no draw groups")
{
  // The floor case, and the one that says an empty frame is empty rather than stale: a renderer with
  // nothing registered must not emit last frame's groups.
  recording_consumer_t consumer;
  dew_renderer_t *renderer = dew_renderer_create();
  REQUIRE(renderer);
  dew_renderer_set_callback(renderer, consumer.callbacks(), &consumer);

  dew_camera_t *camera = dew_camera_create();
  dew_camera_set_perspective(camera, 45, 800, 600, 0.01, 10000);
  const double eye[3] = {1, 1, 1};
  const double center[3] = {0, 0, 0};
  const double up[3] = {0, 0, 1};
  dew_camera_look_at(camera, eye, center, up);

  auto summary = run_frame(renderer, camera, consumer);
  REQUIRE(summary.group_count == 0);
  REQUIRE(consumer.create_calls == 0);

  dew_camera_destroy(camera);
  dew_renderer_destroy(renderer);
}

TEST_CASE("render: an unbalanced octree promotes leaves to virtual sources")
{
  // Virtual subnodes are part of the render interface and had no coverage at all. They exist for the
  // "dense patch" inversion: a LEAF that covers a huge region (so it is never subdivided) but holds
  // enough points to be worth drawing coarsely. A uniform cube never produces one -- every leaf is
  // compact -- which is why this needs its own deliberately lopsided dataset.
  render_fixture_t fixture(k_unbalanced_path, make_unbalanced_source(), 4000);
  fixture.look_from(1, 1, 1, 2.0);

  uint32_t promoted = 0;
  uint64_t gpu_bytes = 0;
  uint64_t resident_cpu_bytes = 0;
  uint32_t nodes_drawn = 0;
  // Settle rather than stop at the first promotion: promotions are capped per frame, so the first one
  // says almost nothing about whether the rest of the tree is reached.
  uint32_t last_drawn = 0;
  int quiet = 0;
  render_until(fixture.renderer, fixture.camera, fixture.consumer, [&](const frame_summary_t &) {
    dew_converter_data_source_get_virtual_stats(fixture.data_source, &promoted, &gpu_bytes, &resident_cpu_bytes, &nodes_drawn);
    if (nodes_drawn == last_drawn && promoted > 0 && nodes_drawn > 0)
      return ++quiet >= 40;
    last_drawn = nodes_drawn;
    quiet = 0;
    return false;
  });

  MESSAGE("virtual promoted: ", promoted, "  nodes drawn: ", nodes_drawn, "  gpu: ", gpu_bytes, "  resident cpu: ", resident_cpu_bytes);
  REQUIRE(promoted > 0);
  // Promotion alone proves only that a leaf QUALIFIED. Drawn virtual nodes prove the rest of the path
  // ran: split_octants, materialize on the convert pool, upload, and emit.
  REQUIRE(nodes_drawn > 0);
  REQUIRE(gpu_bytes > 0);
  REQUIRE(resident_cpu_bytes > 0);
}

TEST_CASE("render: virtual subtrees can be switched off")
{
  // The A/B toggle is public API (dew_converter_data_source_set_enable_virtual_subtrees) and the way a
  // host backs out of the feature. If it did nothing, the previous test would still pass.
  render_fixture_t fixture(k_unbalanced_path, make_unbalanced_source(), 4000);
  dew_converter_data_source_set_enable_virtual_subtrees(fixture.data_source, 0);
  REQUIRE(dew_converter_data_source_get_enable_virtual_subtrees(fixture.data_source) == 0);

  fixture.look_from(1, 1, 1, 2.0);
  auto summary = render_until_points(fixture.renderer, fixture.camera, fixture.consumer);
  render_frames(fixture.renderer, fixture.camera, fixture.consumer, 60);

  uint32_t promoted = 0;
  uint64_t gpu_bytes = 0;
  uint64_t resident_cpu_bytes = 0;
  uint32_t nodes_drawn = 0;
  dew_converter_data_source_get_virtual_stats(fixture.data_source, &promoted, &gpu_bytes, &resident_cpu_bytes, &nodes_drawn);

  MESSAGE("with virtual off -- promoted: ", promoted, "  nodes drawn: ", nodes_drawn, "  points: ", summary.point_draw_size);
  REQUIRE(promoted == 0);
  REQUIRE(nodes_drawn == 0);
  // And the dataset still renders: switching the feature off must not cost the points, it just draws
  // the leaf's own monolith instead of a virtual cut.
  REQUIRE(summary.point_draw_size > 0);
}

TEST_CASE("render: the bounding-box data source emits its own line groups")
{
  // A second data source over the same octree, and a different draw type. Worth its own test because
  // it is the one place a consumer sees dew_node_bbox_lines, and because it shares the renderer's
  // buffer callbacks with the point source -- a handle collision between the two would surface here.
  render_fixture_t fixture;
  dew_converter_data_source_set_show_bounding_boxes(fixture.data_source, 1);
  dew_renderer_add_data_source(fixture.renderer, dew_converter_data_source_get_bbox_data_source(fixture.data_source));
  fixture.look_from(1, 1, 1, 2.0);

  // Wait for BOTH: the boxes come straight from the walker, but the points need their IO to land, so
  // stopping at the first frame with a box catches a frame that has no points yet.
  auto summary = render_until(fixture.renderer, fixture.camera, fixture.consumer, [](const frame_summary_t &s) {
    return s.point_group_count > 0 && std::find(s.types.begin(), s.types.end(), dew_node_bbox_lines) != s.types.end();
  });

  const int bbox_groups = int(std::count(summary.types.begin(), summary.types.end(), dew_node_bbox_lines));
  MESSAGE("bbox groups: ", bbox_groups, "  total groups: ", summary.group_count);
  REQUIRE(bbox_groups > 0);
  // The points are still drawn alongside them; the boxes are an overlay, not a replacement.
  REQUIRE(summary.point_group_count > 0);
}

TEST_CASE("render: points_rendered agrees with the emitted draw groups")
{
  // dew_converter_data_source_get_points_rendered is what a host puts on screen as a statistic. It is
  // computed inside the emit rather than derived from the draw groups, so the two can drift apart
  // without anything else noticing.
  render_fixture_t fixture;
  fixture.look_from(1, 1, 1, 2.0);
  auto summary = render_until_points(fixture.renderer, fixture.camera, fixture.consumer);

  // Re-render one frame and compare THAT frame's groups against the counter it just produced.
  summary = run_frame(fixture.renderer, fixture.camera, fixture.consumer);
  const uint64_t reported = dew_converter_data_source_get_points_rendered(fixture.data_source);

  MESSAGE("points_rendered: ", reported, "  summed draw_size: ", summary.point_draw_size);
  REQUIRE(reported > 0);
  REQUIRE(reported == summary.point_draw_size);
}

TEST_CASE("render: removing the data source stops the draw groups")
{
  // dew_renderer_remove_data_source is how a host swaps datasets. If removal left the source
  // registered, the next frame would keep drawing data the host believes it has detached -- and the
  // buffers would outlive the handle it used to reach them.
  render_fixture_t fixture;
  fixture.look_from(1, 1, 1, 2.0);
  auto before = render_until_points(fixture.renderer, fixture.camera, fixture.consumer);
  REQUIRE(before.point_group_count > 0);

  dew_renderer_remove_data_source(fixture.renderer, dew_converter_data_source_get(fixture.data_source));

  auto after = run_frame(fixture.renderer, fixture.camera, fixture.consumer);
  MESSAGE("groups before removal: ", before.group_count, "  after: ", after.group_count);
  REQUIRE(after.group_count == 0);
  REQUIRE(after.point_draw_size == 0);
}
