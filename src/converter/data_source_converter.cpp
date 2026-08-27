/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2022  Jørgen Lind
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
#include "data_source_converter.hpp"
#include "data_source.hpp"
#include "input_header.hpp"
#include "lod_quantize.hpp" // make_lod_random_offsets (same scheme as the converter)
#include "native_node_data_loader.hpp"
#include "virtual_tree.hpp" // build_resident_source, make_virtual_root, process_virtual_trees, emit_virtual_draws
#ifdef __EMSCRIPTEN__
#include "worker_node_data_loader.hpp" // decode-worker loader (used when the web app installs a worker pool)
#include <emscripten/heap.h>           // emscripten_get_heap_size/_max (heap-pressure brake probe)
#endif
#include <dew/core/format.h>
#include <dew/converter/converter_data_source.h>

#include <vio/objstore/create_object_store.h> // apply_connection_override, clear_*_config_override

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <fmt/printf.h>

#include "renderer.hpp"

using namespace dew;
using namespace dew::converter;
using namespace dew::core;

// Estimate a leaf's resident_source_t CPU footprint from its data_handler (matches build_resident_source's
// cpu_bytes), so in-flight async builds count against the CPU-resident budget before they finalize (bug #2).
static size_t estimate_resident_cpu(const dyn_points_data_handler_t &h)
{
  size_t bytes = h.data_info[0].size + size_t(h.header.point_count) * 3u * sizeof(float);
  if (h.data_info[1].data)
    bytes += h.data_info[1].size;
  return bytes;
}

// CPU bytes a bare salvage handler pins (morton codes + attribute blob) on an unpromoted promotion-candidate
// leaf. Unlike estimate_resident_cpu this excludes the future r32x3 decode -- it hasn't happened yet. A
// PROMOTED node's handler is the same shared_ptr as resident->data_handler and already inside
// resident->cpu_bytes, so it must never be charged through this.
static size_t salvage_handler_bytes(const dyn_points_data_handler_t &h)
{
  size_t bytes = h.data_info[0].size;
  if (h.data_info[1].data)
    bytes += h.data_info[1].size;
  return bytes;
}

// Heap-pressure probe: current wasm heap size and its link-time ceiling. Native has no comparable cheap,
// portable probe -- report 0/0, which compute_brake_level maps to `none` (the budget knob still applies).
static void probe_heap(uint64_t &heap_bytes, uint64_t &heap_max)
{
#ifdef __EMSCRIPTEN__
  heap_bytes = emscripten_get_heap_size();
  heap_max = emscripten_get_heap_max();
#else
  heap_bytes = 0;
  heap_max = 0;
#endif
}

// The one place the read-cache cap is computed from (budget, brake level), so a budget change under a
// latched brake and a brake transition can never disagree about the cap.
static uint64_t braked_read_cache_bytes(const derived_budgets_t &d, brake_level_t level)
{
  if (level == brake_level_t::critical)
    return 0;
  if (level == brake_level_t::high)
    return d.read_cache_bytes / 4;
  return d.read_cache_bytes;
}

dew_converter_data_source_t::dew_converter_data_source_t(const std::string &a_url, render::callback_manager_t &a_callbacks)
  : url(a_url)
  , processor(a_url, file_existence_requirement_t::exist, error)
  , callbacks(a_callbacks)
{
  if (error.code != 0)
  {
    return;
  }
  data_source.user_ptr = this;
  data_source.add_to_frame = [](dew_frame_camera_t *camera, dew_to_render_t *to_render, void *user_ptr)
  {
    auto *thiz = static_cast<dew_converter_data_source_t *>(user_ptr);
    thiz->add_to_frame(camera, to_render);
  };

  bbox_data_source = std::make_unique<node_bbox_data_source_t>(callbacks);

#ifdef __EMSCRIPTEN__
  // On the web, route decode through a pool of Web Workers when the app installed one (globalThis
  // .__dewDecodePool); otherwise fall back to decoding inline on the main thread.
  if (decode_worker_pool_available())
    node_loader = std::make_unique<worker_node_data_loader_t>(processor.storage_handler().reader());
  else
    node_loader = std::make_unique<native_node_data_loader_t>(processor.storage_handler().reader());
#else
  node_loader = std::make_unique<native_node_data_loader_t>(processor.storage_handler().reader());
#endif

  // Apply the default memory budget's derived cache sizes (the storage_handler ctor defaults predate the
  // budget knob; the render data source's decompressed cache in particular is derived smaller because the
  // render path never populates it).
  processor.storage_handler().set_read_cache_size(derived_budgets.read_cache_bytes);
  processor.storage_handler().set_decompressed_cache_size(derived_budgets.decompressed_cache_bytes);

  // Read compression stats for attribute normalization
  attribute_stats = processor.storage_handler().get_compression_stats();

  // Index 0 is the positions, so index 1 is the first attribute worth colouring by -- which exists as
  // soon as there are TWO entries, not three. The old `> 2` skipped selection entirely on a dataset
  // with exactly one non-position attribute, leaving current_attribute_name empty; the walker then
  // asked for the attribute "" on every node and the render path fell over on the null buffer that
  // produced. render_callback_tests covers the two-attribute case for that reason.
  if (processor.attrib_name_registry_count() > 1)
  {
    char buffer[256];
    auto str_size = processor.attrib_name_registry_get(1, buffer, sizeof(buffer));
    next_attribute_name.assign(buffer, str_size);
  }

  last_frame_time = std::chrono::high_resolution_clock::now();
}

dew_converter_data_source_t::~dew_converter_data_source_t()
{
  // Runs before members destruct (esp. convert_pool, which is declared after render_list and would otherwise
  // drain queued jobs against already-freed nodes). destroy_render_node spin-waits each in-flight convert /
  // materialize job, tears down virtual subtrees, and frees all GPU buffers -- fixing both the native
  // shutdown use-after-free and the GPU-buffer leak on data-source destroy/reload.
  for (auto &np : render_list)
    if (np)
      destroy_render_node(*np, callbacks, node_loader.get(), &virtual_gpu_used);
  render_list.clear();
  // Nodes deferred from the non-blocking eviction path. At shutdown a spin-wait is acceptable, so the
  // blocking destroy_render_node drains any still-in-flight job before the convert_pool is torn down.
  for (auto &np : pending_destroy)
    if (np)
      destroy_render_node(*np, callbacks, node_loader.get(), &virtual_gpu_used);
  pending_destroy.clear();
}

void dew_converter_data_source_t::add_to_frame(dew_frame_camera_t *c_camera, dew_to_render_t *to_render)
{
  using clock = std::chrono::high_resolution_clock;
  auto t_start = clock::now();

  float delta_ms = std::chrono::duration<float, std::milli>(t_start - last_frame_time).count();
  last_frame_time = t_start;
  // Clamp to avoid large jumps (e.g., first frame or debugger pause)
  if (delta_ms > 1000.0f)
    delta_ms = 16.0f;

  const render::frame_camera_cpp_t camera = render::cast_to_frame_camera_cpp(*c_camera);
  bool new_attribute = false;
  double frac_threshold;
  int frame_viewport_height;
  double frame_render_density_px;
  io_limits_t io_limits;
  brake_level_t frame_brake;
  size_t frame_cpu_resident_budget; // snapshot: set_memory_budget writes the member under the mutex
  {
    std::unique_lock<std::mutex> lock(mutex);
    new_attribute = current_attribute_name != next_attribute_name;
    current_attribute_name = next_attribute_name;
    frac_threshold = screen_fraction_threshold;
    frame_viewport_height = viewport_height;
    frame_render_density_px = render_density_px;

    // Heap-pressure brake. The level only ever rises within a run (on wasm the heap never shrinks, so
    // pressure that latched once is real until reload); the one-shot cache shrinks fire on each upward
    // transition, the per-frame cap tightening below re-applies every frame at the current level.
    uint64_t heap_bytes = 0, heap_max = 0;
    if (heap_probe_override)
      heap_probe_override(heap_bytes, heap_max);
    else
      probe_heap(heap_bytes, heap_max);
    heap_bytes_last = heap_bytes;
    heap_max_last = heap_max;
    const auto probe_level = compute_brake_level(heap_bytes, heap_max);
    if (probe_level > brake_level)
    {
      brake_level = probe_level;
      fmt::print(stderr, "[membrake] heap {} MB of {} MB ceiling -> {} (tightening io/cache caps)\n",
                 heap_bytes / (1024 * 1024), heap_max / (1024 * 1024), brake_level == brake_level_t::critical ? "critical" : "high");
      processor.storage_handler().set_read_cache_size(braked_read_cache_bytes(derived_budgets, brake_level));
    }
    frame_brake = brake_level;
    frame_cpu_resident_budget = cpu_resident_budget;

    io_limits.max_concurrent_io = std::min(max_in_flight_io, derived_budgets.io_clamp);
    io_limits.max_new_io_per_frame = max_new_io_per_frame;
    io_limits.max_upload_bytes = upload_budget_per_frame;
    io_limits.gpu_memory_budget = gpu_memory_budget;
    io_limits.decoded_backlog_cap = derived_budgets.decoded_backlog_cap;
    // The brake never relaxes (the wasm heap cannot shrink), so every level must stay livable as a
    // PERMANENT state: high halves the caps, critical quarters them and trickles new IO at 1/frame --
    // never zero, which would brick streaming for the rest of the session. malloc reuses freed space
    // inside the grown heap, so a small backlog window cannot push the heap past its ceiling.
    if (frame_brake >= brake_level_t::high)
    {
      io_limits.decoded_backlog_cap /= 2;
      io_limits.max_concurrent_io = std::max(1, io_limits.max_concurrent_io / 2);
    }
    if (frame_brake == brake_level_t::critical)
    {
      io_limits.decoded_backlog_cap /= 2;
      io_limits.max_concurrent_io = std::max(1, io_limits.max_concurrent_io / 2);
      io_limits.max_new_io_per_frame = 1;
    }
  }

  // Handle attribute change
  if (new_attribute)
  {
    if (debug_transitions)
      fmt::print(stderr, "[transition-debug] === ATTRIBUTE CHANGE to '{}' ===\n", current_attribute_name);
    handle_attribute_change(render_list, callbacks, node_loader.get());

    current_attr_min = 0.0;
    current_attr_max = 1.0;
    for (auto &attr : attribute_stats.per_attribute)
    {
      if (attr.name == current_attribute_name && attr.min_value <= attr.max_value)
      {
        current_attr_min = attr.min_value;
        current_attr_max = attr.max_value;
        break;
      }
    }
  }

  // Phase 1: Tree walk
  glm::dvec3 camera_position = glm::dvec3(camera.inverse_view[3]);
  lod_params_t lod_params;
  lod_params.camera_position = camera_position;
  lod_params.projection = camera.projection;
  lod_params.screen_fraction_threshold = frac_threshold;

  if (cached_walker_attribute_source != current_attribute_name)
  {
    cached_walker_attribute_names = {std::string("xyz"), current_attribute_name};
    cached_walker_attribute_source = current_attribute_name;
  }
  frustum_tree_walker_t walker(camera.view_projection, lod_params, cached_walker_attribute_names);
  walker.m_previously_subdivided = std::move(previously_subdivided);
  walker.m_debug = debug_transitions;
  processor.walk_tree(walker);
  previously_subdivided.clear();
  for (auto &[parent, child] : walker.m_new_nodes.parent_child_edges)
    previously_subdivided.insert(parent);
  auto &walker_subsets = walker.m_new_nodes.point_subsets;
  std::sort(walker_subsets.begin(), walker_subsets.end(), render_node_less_than);
  frame_timings.walker_node_count = int(walker_subsets.size());
  frame_timings.walker_trees_to_load = int(walker.m_trees_requested); // NOT .size(): walk_tree moved the vector away
  {
    uint64_t total_pts = 0;
    for (auto &s : walker_subsets)
      total_pts += s.point_count.data;
    frame_timings.walker_total_points = total_pts;
  }
  auto t_after_tree_walk = clock::now();

  // Phase 2: Build render list
  // Non-blocking eviction: retire nodes deferred from earlier frames whose worker jobs have now finished,
  // and re-park the ones still decoding. This keeps the main thread from ever spin-waiting on a convert.
  if (!pending_destroy.empty())
  {
    render_list_t still_busy;
    still_busy.reserve(pending_destroy.size());
    for (auto &np : pending_destroy)
    {
      if (np && node_is_busy(*np))
        still_busy.push_back(std::move(np));
      else if (np)
        destroy_render_node(*np, callbacks, node_loader.get(), &virtual_gpu_used);
    }
    pending_destroy = std::move(still_busy);
  }

  render_list = build_render_list(walker_subsets, std::move(render_list),
      fade_duration_ms, callbacks, node_loader.get(), &virtual_gpu_used, pending_destroy);
  frame_timings.render_list_size = int(render_list.size());
  auto t_after_build = clock::now();

  // Phase 3: IO + upload (single pass for distances, completions, scheduling, upload)
  auto tree_config = processor.tree_config();
  // Departed-but-busy nodes parked in pending_destroy (including ones build_render_list just parked) still
  // hold decoded/decoding CPU buffers in the same heap; pre-charge them against the backlog cap. An
  // uploaded node's decoded buffers were already reaped -- only pre-upload states pin CPU.
  for (auto &np : pending_destroy)
    if (np && (np->io_state == render_node_io_state::converting ||
               (np->io_state == render_node_io_state::loaded && np->gpu_state == render_node_gpu_state::none)))
      io_limits.deferred_backlog_bytes += estimate_node_cpu_bytes(np->walker_data);
  auto io_stats = process_io_and_upload(render_list, camera_position, tree_config,
      callbacks, node_loader.get(), convert_pool, camera, io_limits,
      current_attr_min, current_attr_max, enable_virtual_subtrees, virtual_gpu_used, &cpu_reap_queue);
  decoded_backlog_bytes_last.store(io_stats.backlog_bytes, std::memory_order_relaxed);
  // Free this frame's dead decoded CPU buffers on a worker (their dtor cascade is ~140 render-thread samples).
  if (!cpu_reap_queue.empty())
  {
    convert_pool.enqueue_detached([reaped = std::move(cpu_reap_queue)]() {});
    cpu_reap_queue.clear();
  }
  frame_timings.io_in_flight = io_stats.io_in_flight;
  frame_timings.scan_classify_ms = io_stats.scan_classify_ms;
  frame_timings.schedule_io_ms = io_stats.schedule_io_ms;
  frame_timings.normalize_ms = io_stats.normalize_ms;
  frame_timings.gpu_upload_ms = io_stats.gpu_upload_ms;
  frame_timings.uploads_this_frame = io_stats.uploads_done;
  auto t_after_io_upload = clock::now();

  // Phase 4: Update fades
  update_fades(render_list, delta_ms, fade_duration_ms);
  auto t_after_fade = clock::now();

  // Phase 4.5: Virtual subnodes. Promote uploaded spanning leaves (is_leaf + should_subdivide) whose morton
  // data was salvaged into resident_handler, then walk each virtual octree (materialize/upload/evict). A live
  // cut sets draw_suppressed so emit_draws skips the leaf's own monolith.
  if (enable_virtual_subtrees)
  {
    if (virtual_lod_random_offsets.empty())
      virtual_lod_random_offsets = make_lod_random_offsets();
    // Promote EVERY spanning leaf reached by the walker (not just should_subdivide ones): the whole point is
    // that a far/small spanning leaf is drawn full-res (the dense-patch inversion) and needs a coarse LOD.
    // A leaf is worth promoting only if it has a coarser representation to offer (maskWidth = lod_span-9 > 0);
    // compact leaves keep the cheap monolith. Cap promotions/frame so the one-time decodes don't hitch.
    // Under heap pressure stop starting new promotions: each one pins a resident source (codes + full
    // re-decode + attrs) in the already-tight CPU heap. Existing cuts keep rendering.
    int builds_left = frame_brake >= brake_level_t::high ? 0 : int(virtual_max_promotions_per_frame);
    bool recovery_fired = false; // a recovery reload was kicked this frame -> keep the dirty-driven host ticking
    for (auto &np : render_list)
    {
      auto &node = *np;
      // Finalize an async resident build (R11): the decode ran on convert_pool; wire up the virtual root now.
      if (node.resident_building && node.resident_ready.load(std::memory_order_acquire))
      {
        node.resident_building = false;
        node.resident = std::move(node.pending_resident);
        // Keep resident_handler: it is the SAME shared_ptr as resident->data_handler (a free extra ref while
        // promoted), so un-promotion (A-B toggle off, or a receded cut) can re-promote instantly without a
        // reload. Released only on CPU eviction (R5) or node teardown.
        if (node.resident)
        {
          node.virtual_root = make_virtual_root(*node.resident, node.walker_data.tight_aabb, node.walker_data.aabb);
          node.is_virtual_source = true;
          // resident_cpu_used is recomputed authoritatively below (counts residents + in-flight builds), so no
          // increment here -- the estimate charged at kick-off already persisted across the build.
        }
        continue;
      }
      // R5 recovery: a leaf un-promoted by the CPU budget kept its (still-uploaded) monolith but genuinely
      // freed its salvage handler, and the salvage lift only runs on a fresh upload -- so it can never
      // re-promote through the normal path. Once budget headroom returns, free the monolith and reset io so
      // the IO scan reloads it: the additive octree's coarser ancestors cover the leaf during the reload
      // exactly like the initial load, the fresh upload re-lifts the handler, and promotion proceeds. The
      // hysteresis must include THIS leaf's own resident estimate (mortons + r32x3 decode + attrs, computable
      // from walker formats without the handler): gating on the aggregate alone lets a leaf whose resident
      // exceeds budget/4 defeat the 3/4 band and ping-pong evict<->reload forever. An oversized leaf that can
      // never fit the band simply stays on its monolith -- the stable, intended fallback. fade_out nodes are
      // departing: destroying their monolith mid-crossfade would pop the region off screen (and the reload
      // would never run; build_render_list destroys non-uploaded fade-outs).
      //
      // The estimate reads node.point_count, which a convert_pool worker writes (convert_node_data)
      // right up until it publishes convert_done. So compute it INSIDE the state guard, never before
      // it: io_state == loaded is only set by the frame that observed convert_done with acquire, so
      // by then the worker's write happens-before this read. Hoisted out, as it used to be, it read
      // the field of every node still converting and raced the worker -- eight TSan reports.
      if (node.salvage_lost && !node.resident_handler && !node.resident_building && node.walker_data.frustum_visible &&
          node.fade_state != render_node_fade_state::fade_out &&
          node.gpu_state == render_node_gpu_state::uploaded && node.io_state == render_node_io_state::loaded &&
          builds_left > 0)
      {
        const size_t resident_estimate =
          size_t(node.point_count) * (size_t(size_for_format(node.walker_data.format[0].type, node.walker_data.format[0].components)) + 3u * sizeof(float) +
                                      (node.walker_data.locations[1].size > 0 ? size_t(size_for_format(node.walker_data.format[1].type, node.walker_data.format[1].components)) : 0u));
        if (resident_cpu_used + resident_estimate <= frame_cpu_resident_budget * 3 / 4)
        {
          --builds_left;             // a reload leads to a resident build soon; count it against the ramp
          node.salvage_lost = false; // one-shot; re-armed only by another R5 eviction
          recovery_fired = true;     // the reload starts NEXT frame's IO scan -> tick the host until it kicks in
          for (auto &b : node.gpu_buffers)
            if (b.user_ptr)
              callbacks.do_destroy_buffer(b);
          if (node.params_buffer.user_ptr)
            callbacks.do_destroy_buffer(node.params_buffer);
          node.gpu_state = render_node_gpu_state::none;
          node.io_state = render_node_io_state::none;
          node.gpu_memory_size = 0;
          continue;
        }
      }
      if (node.is_virtual_source || node.resident_building || !node.resident_handler || node.gpu_state != render_node_gpu_state::uploaded)
        continue;
      node.salvage_lost = false; // handler present again (fresh upload lifted it) -> recovery no longer pending
      if (node.point_count <= virtual_min_points || !node.walker_data.is_leaf)
      {
        node.resident_handler.reset(); // too small / not a leaf -> never promotes; drop the salvaged CPU dup
        continue;
      }
      if (node.resident_handler->header.lod_span <= lod_quantize_full_detail_level)
      {
        node.resident_handler.reset(); // compact leaf: maskWidth(lod_span)==0, no coarser LOD to offer
        continue;
      }
      // Latched heap-pressure brake: no promotion is coming this session, so do NOT suppress the monolith --
      // suppressing without a cut would pin the region at ancestor-level detail forever. (A leaf suppressed
      // just before the latch is un-suppressed here the next frame.)
      if (frame_brake >= brake_level_t::high)
      {
        node.draw_suppressed = false;
        continue;
      }
      // This IS a promotable spanning leaf -> never draw its full-res monolith. Suppress it NOW (phase 4.5, before
      // emit_draws): the octree is additive so the coarser ancestor nodes already cover the leaf's footprint while
      // its virtual cut materializes, then the cut refines it coarse->fine. Set before the per-frame build cap so a
      // leaf still waiting its turn to promote isn't flashed at full res either. (Un-promotion / A-B-off / departure
      // reset draw_suppressed elsewhere; a leaf that stops being promotable falls back to its monolith there.)
      node.draw_suppressed = true;
      if (builds_left <= 0 || resident_cpu_used >= frame_cpu_resident_budget)
        continue; // ramp over frames; and don't promote while over the CPU-resident budget (R5)
      --builds_left;
      // Kick the resident decode onto convert_pool (R11) -- keeps the ~1-2ms/leaf morton decode off the render
      // thread. The job captures &node (stable: build_render_list moves the unique_ptr, not the object) + a ref
      // to the handler; ~data_source / destroy_render_node spin-waits resident_ready before freeing the node.
      node.resident_building = true;
      node.resident_ready.store(false, std::memory_order_relaxed);
      resident_cpu_used += estimate_resident_cpu(*node.resident_handler); // charge now so later nodes this frame see it (bug #2)
      render_node_t *np_raw = &node;
      auto handler = node.resident_handler;
      tree_config_t tc = tree_config;
      convert_pool.enqueue([np_raw, handler, tc] {
        np_raw->pending_resident = build_resident_source(handler, tc);
        np_raw->resident_ready.store(true, std::memory_order_release);
      });
    }
    virtual_frame_t vf;
    vf.camera = &camera;
    vf.camera_position = camera_position;
    vf.tree_config = &tree_config;
    vf.lod_params = &lod_params;
    vf.callbacks = &callbacks;
    vf.convert_pool = &convert_pool;
    vf.lod_random_offsets = &virtual_lod_random_offsets;
    vf.gpu_memory_budget = gpu_memory_budget;
    vf.gpu_memory_used = &virtual_gpu_used;
    vf.real_gpu_used = io_stats.gpu_memory_used; // monolith GPU total this frame -> shared budget gate
    vf.virtual_min_points = virtual_min_points;
    vf.frame_index = ++virtual_frame_counter;
    vf.delta_ms = delta_ms;
    vf.fade_duration_ms = fade_duration_ms;
    vf.viewport_height = frame_viewport_height;
    // Floor the virtual subdivision at the full-detail level. A promoted leaf's virtual tree exists precisely to
    // refine its own points independent of what other real geometry is on screen, so the floor must be a fixed
    // property of the leaf (level 9, below which maskWidth is 0 -> every point already drawn), NOT the finest
    // real-node lod currently in frustum. The old min-over-real-nodes clamp collapsed at extreme zoom: the octree
    // is additive, so zooming into a single promoted leaf leaves only its own COARSER ancestors frustum-visible
    // (the leaf itself is is_virtual_source, excluded); min_real_lod then rose above the virtual root's level
    // (= leaf_lod = lod_span), the subdivide gate v.level > floor went false at the root, and the whole cut
    // collapsed to one coarse whole-leaf node that never refined again. should_subdivide's screen-space test
    // still gates real per-frame depth; split_octants' level<=0 guard remains the crash backstop from 5ab5436,
    // so the min-over-real clamp is no longer needed for safety.
    vf.subdivide_floor_lod = lod_quantize_full_detail_level;
    vf.render_density_px = frame_render_density_px;
    vf.attr_min = current_attr_min;
    vf.attr_max = current_attr_max;
    process_virtual_trees(render_list, vf);
    // Keep the on-demand renderer ticking while any resident build / materialize / fade is pending, so async
    // virtual work completes even when the camera is idle (e.g. right after flipping the A/B toggle). A fired
    // R5 recovery counts too: its reload only enters the IO scan NEXT frame, and on the dirty-driven wasm host
    // nothing else would schedule that frame.
    virtual_animating = vf.any_animating || recovery_fired;
    for (auto &np : render_list)
      if (np->resident_building)
      {
        virtual_animating = true;
        break;
      }

    // R5: CPU-resident budget. Recompute the pinned total (finalized residents + in-flight async builds, whose
    // pending_resident is already allocated -- bug #2), then un-promote farthest-first until under budget; the
    // evicted leaf falls back to its monolith (reload it if R3 had freed it).
    resident_cpu_used = 0;
    for (auto &np : render_list)
    {
      if (np->is_virtual_source && np->resident)
        resident_cpu_used += np->resident->cpu_bytes;
      else if (np->resident_building && np->resident_handler)
        resident_cpu_used += estimate_resident_cpu(*np->resident_handler);
      else if (np->resident_handler)
        // Bare salvage copy on an unpromoted candidate leaf: real heap bytes that previously went uncounted.
        // (A promoted node's handler is the same shared_ptr inside resident->cpu_bytes -- branch 1 covers it.)
        resident_cpu_used += salvage_handler_bytes(*np->resident_handler);
    }
    // Under critical heap pressure enforce down to a quarter of the budget: un-promotions genuinely free
    // resident + handler bytes, the most effective relief the renderer has.
    const size_t resident_target = frame_brake == brake_level_t::critical ? frame_cpu_resident_budget / 4 : frame_cpu_resident_budget;
    while (resident_cpu_used > resident_target)
    {
      // Tier 1: drop bare salvage handlers from unpromoted candidates farthest-first. Cheap (no virtual
      // teardown), and salvage_lost re-arms the R5-recovery reload once headroom returns.
      render_node_t *bare = nullptr;
      for (auto &np : render_list)
        if (np->resident_handler && !np->is_virtual_source && !np->resident_building && (!bare || np->cached_distance > bare->cached_distance))
          bare = np.get();
      if (bare)
      {
        resident_cpu_used -= std::min(resident_cpu_used, salvage_handler_bytes(*bare->resident_handler));
        bare->resident_handler.reset();
        bare->draw_suppressed = false; // no cut is coming; the monolith must draw again
        bare->salvage_lost = true;
        continue;
      }
      render_node_t *farthest = nullptr;
      for (auto &np : render_list)
        // Skip a subtree still materializing on the convert pool: destroy_virtual_subtree would spin-wait the
        // main thread. Leaving it over-budget for a frame (until its job finishes) is the non-blocking choice.
        if (np->is_virtual_source && np->resident && !(np->virtual_root && virtual_subtree_has_inflight(*np->virtual_root)) && (!farthest || np->cached_distance > farthest->cached_distance))
          farthest = np.get();
      if (!farthest)
        break;
      resident_cpu_used -= farthest->resident->cpu_bytes;
      destroy_virtual_subtree(farthest->virtual_root, callbacks, &virtual_gpu_used);
      farthest->resident.reset();
      farthest->resident_handler.reset(); // genuinely free the data_handler CPU (unlike a toggle-off, which keeps it)
      farthest->is_virtual_source = false;
      farthest->draw_suppressed = false;
      // This leaf is proven-promotable but now has no salvage handler, and the lift only runs on a fresh
      // upload. Mark it so the promoter can free+reload its monolith to re-acquire the handler once budget
      // headroom returns — otherwise it is stranded on its full-res monolith forever (zoom-out repro).
      farthest->salvage_lost = true;
      if (farthest->monolith_freed) // R3 freed it -> reload the monolith so the node is drawable again
      {
        farthest->monolith_freed = false;
        farthest->io_state = render_node_io_state::none;
      }
    }
    resident_cpu_published.store(resident_cpu_used, std::memory_order_relaxed);
  }

  // Collect bounding boxes and tight AABB, count stats
  {
    std::vector<node_bbox_t> loose_boxes;
    std::vector<node_bbox_t> tight_boxes;
    int loading = 0, converting = 0, uploaded = 0, fading_in = 0, fading_out = 0;
    for (auto &np : render_list)
    {
      if (np->fade_state != render_node_fade_state::fade_out)
      {
        tight_aabb_accumulator.min = glm::min(tight_aabb_accumulator.min, np->walker_data.tight_aabb.min);
        tight_aabb_accumulator.max = glm::max(tight_aabb_accumulator.max, np->walker_data.tight_aabb.max);
      }
      if (show_bounding_boxes && np->fade_state != render_node_fade_state::fade_out)
      {
        loose_boxes.push_back({np->walker_data.aabb.min, np->walker_data.aabb.max});
        tight_boxes.push_back({np->walker_data.tight_aabb.min, np->walker_data.tight_aabb.max});
      }
      if (np->io_state == render_node_io_state::loading || np->io_state == render_node_io_state::loaded)
        loading++;
      if (np->io_state == render_node_io_state::converting)
        converting++;
      if (np->gpu_state == render_node_gpu_state::uploaded)
        uploaded++;
      if (np->fade_state == render_node_fade_state::fade_in)
        fading_in++;
      if (np->fade_state == render_node_fade_state::fade_out)
        fading_out++;
    }
    if (show_bounding_boxes)
      bbox_data_source->update_boxes(loose_boxes, tight_boxes);
    frame_timings.nodes_loading = loading;
    frame_timings.nodes_converting = converting;
    frame_timings.nodes_uploaded = uploaded;
    frame_timings.nodes_fading_in = fading_in;
    frame_timings.nodes_fading_out = fading_out;
  }

  // Phase 5: Emit draws
  uint64_t pts_rendered = 0;
  frame_timings.nodes_drawn = emit_draws(render_list, callbacks, camera, tree_config, to_render, fade_duration_ms, frame_viewport_height, frame_render_density_px, pts_rendered);
  if (enable_virtual_subtrees)
  {
    int vdrawn = emit_virtual_draws(render_list, callbacks, camera, tree_config, to_render, virtual_frame_counter, frame_viewport_height, frame_render_density_px, fade_duration_ms, pts_rendered);
    frame_timings.nodes_drawn += vdrawn;
    virtual_nodes_drawn_last = vdrawn;
    int promoted = 0;
    for (auto &np : render_list)
      if (np->is_virtual_source)
        promoted++;
    virtual_promoted_last = promoted;
    // Report virtual-subnode activity only when it changes, so it's a verifiable signal without per-frame spam.
    if (promoted != last_virtual_promoted)
    {
      fmt::print(stderr, "[virtual] promoted spanning leaves = {} (gpu {} KB)\n", promoted, virtual_gpu_used / 1024);
      last_virtual_promoted = promoted;
    }
  }
  else
  {
    virtual_promoted_last = 0;
    virtual_nodes_drawn_last = 0;
    virtual_animating = false;
  }
  points_rendered_last_frame = pts_rendered;
  auto t_after_emit = clock::now();

  auto t_end = clock::now();

  auto to_ms = [](auto duration) { return std::chrono::duration<double, std::milli>(duration).count(); };
  frame_timings.tree_walk_ms = to_ms(t_after_tree_walk - t_start);
  frame_timings.build_render_list_ms = to_ms(t_after_build - t_after_tree_walk);
  // scan_classify_ms, schedule_io_ms, normalize_ms, gpu_upload_ms set from io_stats above
  frame_timings.fade_ms = to_ms(t_after_fade - t_after_io_upload);
  frame_timings.emit_ms = to_ms(t_after_emit - t_after_fade);
  frame_timings.total_ms = to_ms(t_end - t_start);
}

struct dew_converter_data_source_t *dew_converter_data_source_create_with_connection(const char *url, uint32_t url_len, const char *connection, uint32_t connection_len, dew_error_t *error, struct dew_renderer_t *renderer)
{
  if (!error)
    return nullptr;
  std::string url_str(url, url_len);

  // Install the connection string (credentials/endpoint/region) for the dataset URL's provider before the
  // storage backend is created inside the data source's processor. A no-op for local (file/dir/mem) URLs.
  bool applied = false;
  if (connection && connection_len > 0)
  {
    auto result = vio::objstore::apply_connection_override(url_str, std::string_view(connection, connection_len));
    if (!result)
    {
      error->code = result.error().code != 0 ? result.error().code : -1;
      error->msg = result.error().msg;
      return nullptr;
    }
    applied = true;
  }

  auto ret = std::make_unique<dew_converter_data_source_t>(url_str, renderer->callbacks);

  // The override was consumed when the backend was created (its bucket/prefix are baked in), so clear it
  // now; leaving it set would leak into a later create with a different URL in the same process.
  if (applied)
  {
    vio::objstore::clear_s3_config_override();
#ifndef __EMSCRIPTEN__
    vio::objstore::clear_azure_config_override();
#endif
  }

  if (ret->error.code != 0)
  {
    *error = ret->error;
    return nullptr;
  }
  return ret.release();
}

struct dew_converter_data_source_t *dew_converter_data_source_create(const char *url, uint32_t url_len, dew_error_t *error, struct dew_renderer_t *renderer)
{
  return dew_converter_data_source_create_with_connection(url, url_len, nullptr, 0, error, renderer);
}

void dew_converter_data_source_destroy(struct dew_converter_data_source_t *converter_data_source)
{
  delete converter_data_source;
}

struct dew_data_source_t dew_converter_data_source_get(struct dew_converter_data_source_t *converter_data_source)
{
  return converter_data_source->data_source;
}

void dew_converter_data_source_request_aabb(struct dew_converter_data_source_t *converter_data_source, dew_converter_data_source_request_aabb_callback_t callback, void *user_ptr)
{
  auto callback_cpp = [callback, user_ptr](double aabb_min[3], double aabb_max[3]) { callback(aabb_min, aabb_max, user_ptr); };

  converter_data_source->processor.request_aabb(callback_cpp);
}

uint32_t dew_converter_data_attribute_count(struct dew_converter_data_source_t *converter_data_source)
{
  return converter_data_source->processor.attrib_name_registry_count();
}

uint32_t dew_converter_data_get_attribute_name(struct dew_converter_data_source_t *converter_data_source, int index, char *name, uint32_t name_size)
{
  return converter_data_source->processor.attrib_name_registry_get(index, name, name_size);
}

void dew_converter_data_set_rendered_attribute(struct dew_converter_data_source_t *converter_data_source, const char *name, uint32_t name_len)
{
  std::unique_lock<std::mutex> lock(converter_data_source->mutex);
  converter_data_source->next_attribute_name.assign(name, name_len);
}

void dew_converter_data_source_set_viewport(struct dew_converter_data_source_t *converter_data_source, int width, int height)
{
  std::unique_lock<std::mutex> lock(converter_data_source->mutex);
  converter_data_source->viewport_width = width;
  converter_data_source->viewport_height = height;
}

void dew_converter_data_source_set_pixel_error_threshold(struct dew_converter_data_source_t *converter_data_source, double threshold)
{
  std::unique_lock<std::mutex> lock(converter_data_source->mutex);
  converter_data_source->screen_fraction_threshold = threshold;
}

void dew_converter_data_source_set_render_density_px(struct dew_converter_data_source_t *converter_data_source, double density_px)
{
  std::unique_lock<std::mutex> lock(converter_data_source->mutex);
  converter_data_source->render_density_px = density_px > 0.0 ? density_px : 0.01;
}

void dew_converter_data_source_set_gpu_memory_budget(struct dew_converter_data_source_t *converter_data_source, size_t budget_bytes)
{
  std::unique_lock<std::mutex> lock(converter_data_source->mutex);
  converter_data_source->gpu_memory_budget = budget_bytes;
}

void dew_converter_data_source_set_upload_budget_per_frame(struct dew_converter_data_source_t *converter_data_source, size_t budget_bytes)
{
  std::unique_lock<std::mutex> lock(converter_data_source->mutex);
  converter_data_source->upload_budget_per_frame = budget_bytes;
}

void dew_converter_data_source_set_max_in_flight_io(struct dew_converter_data_source_t *converter_data_source, int max_requests)
{
  std::unique_lock<std::mutex> lock(converter_data_source->mutex);
  converter_data_source->max_in_flight_io = max_requests;
}

void dew_converter_data_source_set_memory_budget(struct dew_converter_data_source_t *cds, uint64_t total_bytes)
{
  constexpr uint64_t min_budget = 64 * 1024 * 1024;
  if (total_bytes < min_budget)
    total_bytes = min_budget;
  std::unique_lock<std::mutex> lock(cds->mutex);
  cds->total_memory_budget = total_bytes;
  cds->derived_budgets = derive_budgets(total_bytes);
  cds->cpu_resident_budget = cds->derived_budgets.cpu_resident_budget;
  // Cache caps applied while still holding the mutex so this can never interleave with a brake transition
  // (which recomputes the same cap under the same mutex). Safe: the storage caches' own locks are only ever
  // taken AFTER the data-source mutex, never before it. braked_read_cache_bytes keeps a latched brake's
  // divisor in effect, so lowering the budget under pressure tightens the cache as the user expects.
  cds->processor.storage_handler().set_read_cache_size(braked_read_cache_bytes(cds->derived_budgets, cds->brake_level));
  cds->processor.storage_handler().set_decompressed_cache_size(cds->derived_budgets.decompressed_cache_bytes);
}

uint64_t dew_converter_data_source_get_memory_budget(struct dew_converter_data_source_t *cds)
{
  std::unique_lock<std::mutex> lock(cds->mutex);
  return cds->total_memory_budget;
}

void dew_converter_data_source_get_memory_stats(struct dew_converter_data_source_t *cds,
  uint64_t *heap_bytes, uint64_t *heap_max, uint64_t *budget_bytes, uint64_t *backlog_bytes,
  uint64_t *read_cache_bytes, uint64_t *resident_cpu_bytes, uint32_t *brake_level)
{
  std::unique_lock<std::mutex> lock(cds->mutex);
  if (heap_bytes)
    *heap_bytes = cds->heap_bytes_last;
  if (heap_max)
    *heap_max = cds->heap_max_last;
  if (budget_bytes)
    *budget_bytes = cds->total_memory_budget;
  if (brake_level)
    *brake_level = uint32_t(cds->brake_level);
  lock.unlock(); // the remaining fields are atomics / have their own lock
  if (backlog_bytes)
    *backlog_bytes = cds->decoded_backlog_bytes_last.load(std::memory_order_relaxed);
  if (resident_cpu_bytes)
    *resident_cpu_bytes = cds->resident_cpu_published.load(std::memory_order_relaxed);
  if (read_cache_bytes)
    *read_cache_bytes = cds->processor.storage_handler().read_cache_current_bytes();
}

uint64_t dew_converter_data_source_get_points_rendered(struct dew_converter_data_source_t *converter_data_source)
{
  return converter_data_source->points_rendered_last_frame;
}

// True while the last frame left a node crossfade in progress. The dirty-driven host (renderer_wasm) polls
// this after a draw to re-arm the next frame so a fade-in/out keeps playing to completion even with no camera
// input or IO in flight. (The per-node LOD detail is distance-driven, not time-animated, so it is not here.)
uint8_t dew_converter_data_source_is_animating(struct dew_converter_data_source_t *cds)
{
  auto &t = cds->frame_timings;
  return ((t.nodes_fading_in + t.nodes_fading_out) > 0 || cds->virtual_animating) ? 1 : 0;
}

void dew_converter_data_source_get_frame_timings(struct dew_converter_data_source_t *cds, double *tree_walk_ms, double *buffer_reconciliation_ms, double *gpu_upload_ms, double *refine_strategy_ms, double *frontier_scheduling_ms,
                                             double *draw_emission_ms, double *eviction_ms, double *total_ms,
                                             int *registry_node_count, int *active_set_size, int *nodes_drawn,
                                             int *transitioning_count, int *nodes_evicted, int *nodes_reconcile_destroyed,
                                             int *walker_node_count, uint64_t *walker_total_points, int *walker_trees_to_load,
                                             int *io_in_flight)
{
  auto &t = cds->frame_timings;
  // Map new pipeline timings to old API parameters
  *tree_walk_ms = t.tree_walk_ms;
  *buffer_reconciliation_ms = t.build_render_list_ms;
  *gpu_upload_ms = t.gpu_upload_ms;
  *refine_strategy_ms = t.scan_classify_ms + t.schedule_io_ms;
  *frontier_scheduling_ms = t.normalize_ms;
  *draw_emission_ms = t.emit_ms;
  *eviction_ms = t.fade_ms;
  *total_ms = t.total_ms;
  if (registry_node_count) *registry_node_count = t.render_list_size;
  if (active_set_size) *active_set_size = t.nodes_uploaded;
  if (nodes_drawn) *nodes_drawn = t.nodes_drawn;
  if (transitioning_count) *transitioning_count = t.nodes_fading_in + t.nodes_fading_out;
  if (nodes_evicted) *nodes_evicted = 0;
  if (nodes_reconcile_destroyed) *nodes_reconcile_destroyed = 0;
  if (walker_node_count) *walker_node_count = t.walker_node_count;
  if (walker_total_points) *walker_total_points = t.walker_total_points;
  if (walker_trees_to_load) *walker_trees_to_load = t.walker_trees_to_load;
  if (io_in_flight) *io_in_flight = t.io_in_flight;
}

void dew_converter_data_source_set_debug_transitions(struct dew_converter_data_source_t *cds, uint8_t enabled)
{
  cds->debug_transitions = enabled;
}

void dew_converter_data_source_set_show_bounding_boxes(struct dew_converter_data_source_t *cds, uint8_t enabled)
{
  cds->show_bounding_boxes = enabled;
  cds->bbox_data_source->enabled = enabled;
}

void dew_converter_data_source_set_enable_virtual_subtrees(struct dew_converter_data_source_t *cds, uint8_t enabled)
{
  const bool on = enabled != 0;
  if (on == cds->enable_virtual_subtrees)
    return;
  cds->enable_virtual_subtrees = on;
  if (!on)
  {
    // Turn-off: tear every virtual cut down and fall the leaves back to their monoliths, so draw_suppressed
    // isn't left stale (which would blank the promoted regions) and a later turn-on re-promotes cleanly.
    for (auto &np : cds->render_list)
    {
      auto &node = *np;
      if (!node.is_virtual_source)
      {
        // A promotable leaf is suppressed the moment it's recognized -- possibly BEFORE its promotion completes
        // (per-frame build cap, in-flight resident build). Unsuppress those too or they'd stay invisible forever
        // with virtual off. (An in-flight build finalizes on the next turn-on; pending_resident just parks.)
        node.draw_suppressed = false;
        continue;
      }
      if (node.virtual_root)
        dew::converter::destroy_virtual_subtree(node.virtual_root, cds->callbacks, &cds->virtual_gpu_used);
      node.resident.reset();
      node.is_virtual_source = false;
      node.draw_suppressed = false;
      if (node.monolith_freed) // R3 freed it -> reload the monolith
      {
        node.monolith_freed = false;
        node.io_state = dew::converter::render_node_io_state::none;
      }
    }
    cds->resident_cpu_used = 0;
    cds->resident_cpu_published.store(0, std::memory_order_relaxed);
    cds->virtual_promoted_last = 0;
    cds->virtual_nodes_drawn_last = 0;
    cds->last_virtual_promoted = -1;
  }
  else
  {
    // Turn-on: a leaf loaded WHILE virtual was off never had its salvage handler lifted (want_salvage is off at
    // load time, and the decoded CPU buffers were reaped after upload) -- on both the native and worker paths it
    // could never promote. Arm the R5-recovery reload for those; leaves whose handler the turn-off path kept
    // re-promote instantly without it. (A reloaded compact/too-small leaf is rejected once and stays monolith.)
    for (auto &np : cds->render_list)
    {
      auto &node = *np;
      if (node.walker_data.is_leaf && !node.resident_handler && !node.is_virtual_source && !node.resident_building &&
          node.point_count > cds->virtual_min_points && node.gpu_state == dew::converter::render_node_gpu_state::uploaded)
        node.salvage_lost = true;
    }
  }
}

uint8_t dew_converter_data_source_get_enable_virtual_subtrees(struct dew_converter_data_source_t *cds)
{
  return cds->enable_virtual_subtrees ? 1 : 0;
}

void dew_converter_data_source_get_virtual_stats(struct dew_converter_data_source_t *cds,
  uint32_t *promoted, uint64_t *gpu_bytes, uint64_t *resident_cpu_bytes, uint32_t *nodes_drawn)
{
  if (promoted)
    *promoted = uint32_t(cds->virtual_promoted_last);
  if (gpu_bytes)
    *gpu_bytes = uint64_t(cds->virtual_gpu_used);
  if (resident_cpu_bytes)
    *resident_cpu_bytes = cds->resident_cpu_published.load(std::memory_order_relaxed);
  if (nodes_drawn)
    *nodes_drawn = uint32_t(cds->virtual_nodes_drawn_last);
}

struct dew_data_source_t dew_converter_data_source_get_bbox_data_source(struct dew_converter_data_source_t *cds)
{
  return cds->bbox_data_source->data_source;
}

void dew_converter_data_source_get_tight_aabb(struct dew_converter_data_source_t *cds, double min[3], double max[3])
{
  auto &ta = cds->tight_aabb_accumulator;
  memcpy(min, &ta.min, sizeof(double) * 3);
  memcpy(max, &ta.max, sizeof(double) * 3);
}
