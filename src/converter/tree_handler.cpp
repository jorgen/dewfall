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
#include "tree_handler.hpp"

#include "tree_build.hpp"

#include "tree_lod_generator.hpp"

#include "morton_tree_coordinate_transform.hpp"
#include "storage_handler.hpp"

#include "loop_quiesce.hpp"

#include <atomic>
#include <chrono>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

namespace dew::converter
{
using namespace dew::core;

// Awaitable wrapper for callback-based storage_handler operations.
// The callback is called on the storage handler's event loop, and posts
// the result back to the caller's event loop to resume the coroutine.
template <typename Result>
struct callback_awaitable_t
{
  struct state_t
  {
    Result result;
    std::coroutine_handle<> continuation;
    vio::event_loop_t &caller_loop;
  };

  std::shared_ptr<state_t> _state;

  explicit callback_awaitable_t(vio::event_loop_t &caller_loop)
    : _state(std::make_shared<state_t>(Result{}, std::coroutine_handle<>{}, caller_loop))
  {
  }

  bool await_ready() const noexcept { return false; }

  void await_suspend(std::coroutine_handle<> continuation) noexcept
  {
    _state->continuation = continuation;
  }

  Result await_resume() noexcept
  {
    return std::move(_state->result);
  }
};

struct write_trees_result_t
{
  std::vector<tree_id_t> tree_ids;
  std::vector<storage_location_t> locations;
  dew_error_t error;
};

struct write_tree_registry_result_t
{
  storage_location_t location;
  dew_error_t error;
};

struct write_blob_result_t
{
  dew_error_t error;
};

tree_handler_t::tree_handler_t(vio::thread_pool_t &thread_pool, storage_handler_t &file_cache, attributes_configs_t &attributes_configs, perf_stats_t &perf_stats, vio::event_pipe_t<input_data_id_t> &done_with_input)
  : _thread_pool(thread_pool)
  , _event_loop_thread()
  , _event_loop(_event_loop_thread.event_loop())
  , _initialized(false)
  , _configuration_initialized(false)
  , _pre_init_tree_config({0.001, {100000, 100000, 100000}})
  , _first_root_initialized(false)
  , _file_cache(file_cache)
  , _attributes_configs(attributes_configs)
  , _perf_stats(perf_stats)
  , _tree_lod_generator(_event_loop, _thread_pool, _tree_registry, _file_cache, _attributes_configs, _perf_stats, _serialize_trees)
  , _tree_collapse(_event_loop, _thread_pool, _tree_registry, _file_cache, _attributes_configs)
  , add_points(_event_loop, bind(&tree_handler_t::handle_add_points))
  , _generate_lod_pipe(_event_loop, bind(&tree_handler_t::handle_generate_lod))
  , _serialize_trees(_event_loop, bind(&tree_handler_t::handle_serialize_trees))
  , _checkpoint_request(_event_loop, bind(&tree_handler_t::handle_checkpoint_request))
  , _deserialize_tree(_event_loop, bind(&tree_handler_t::handle_deserialize_tree))
  , _done_with_input(done_with_input)
  , _request_aabb(_event_loop, bind(&tree_handler_t::handle_request_aabb))
  , _request_root(_event_loop, bind(&tree_handler_t::handle_request_root))
  , _request_trees_batch(_event_loop, bind(&tree_handler_t::handle_request_trees_batch))
{
  _event_loop.add_about_to_block_listener(this);
}

tree_handler_t::~tree_handler_t()
{
  // Safety net for standalone use: join the loop before _tree_registry / the pipes it touches are destroyed.
  // In the processor the ordered teardown already stopped it (stop_and_join is idempotent).
  _event_loop_thread.stop_and_join();
}

void tree_handler_t::begin_shutdown()
{
  // Flip the flag ON the tree loop and wait for it: once this task runs, every previously-queued tree-load
  // batch has already enqueued its pool task, and every later batch sees the flag and enqueues nothing. So
  // after this returns the caller may drain the thread pool without racing an enqueue.
  // See loop_quiesce.hpp for why the wait has to be shaped this way, and why it is bounded. On the
  // timeout path the flag is set here instead, which is weaker (a batch already past the check can
  // still enqueue) but is exactly the pre-barrier behaviour rather than a deadlock.
  if (!core::run_on_loop_and_wait(_event_loop, [this]() { _shutting_down.store(true, std::memory_order_release); }))
    _shutting_down.store(true, std::memory_order_release);
}

void tree_handler_t::stop_loop()
{
  _event_loop_thread.stop_and_join();
}

dew_error_t tree_handler_t::deserialize_tree_registry(std::unique_ptr<uint8_t[]> &tree_registry_buffer, uint32_t tree_registry_blobs_size)
{
  auto ret = tree_registry_deserialize(tree_registry_buffer, tree_registry_blobs_size, _tree_registry);
  if (ret.code == 0)
  {
    _initialized = true;
    _configuration_initialized = true;
    // Resume: seed the finality/LOD watermarks from the persisted registry (v2; zero for v1 files
    // -- morton192 is a min-accumulating compare, an all-zero watermark restores nothing).
    morton::morton192_t zero = {};
    if (zero < _tree_registry.lod_watermark)
    {
      _pass_watermark = _tree_registry.lod_watermark;
      _has_pass_watermark = true;
      _pending_pass_watermark = _tree_registry.lod_watermark;
      _has_pending_pass_watermark = true;
      _tree_lod_generator.restore_lod_complete_morton(_tree_registry.lod_watermark);
    }
  }
  else
  {
    _tree_registry = {};
  }
  return ret;
}

void tree_handler_t::request_root()
{
  _request_root.post_event();
#ifdef __EMSCRIPTEN__
  // Cooperative single-thread: spin the browser event loop (this runs at open, top of stack, so a
  // nested pump is safe) until the root read+deserialize chain sets _first_root_initialized.
  while (!_first_root_initialized)
  {
    vio::wasm::pump();
    emscripten_sleep(0);
  }
#else
  std::unique_lock<std::mutex> lock(_root_mutex);
  _root_cv.wait(lock, [this] { return _first_root_initialized; });
#endif
}

void tree_handler_t::set_tree_initialization_config(const tree_config_t &config)
{
  std::unique_lock<std::mutex> lock(_configuration_mutex);
  assert(!_configuration_initialized);
  _pre_init_tree_config = config;
}

void tree_handler_t::set_tree_initialization_scale(double scale)
{
  std::unique_lock<std::mutex> lock(_configuration_mutex);
  assert(!_configuration_initialized);
  _pre_init_tree_config.scale = scale;
}

void tree_handler_t::set_tree_initialization_node_point_limit(uint32_t limit)
{
  std::unique_lock<std::mutex> lock(_configuration_mutex);
  assert(!_configuration_initialized);
  _pre_init_tree_config.node_point_limit = limit;
}

void tree_handler_t::set_mutable(bool value)
{
  // BOTH configs, because which one survives depends on when this is called.
  //
  //  * Before the first file, the configuration is not yet sealed and seal_configuration() will
  //    overwrite the registry's copy wholesale from _pre_init_tree_config -- so setting only the
  //    registry silently loses the flag on a fresh dataset.
  //  * On a reopened dataset the configuration is already sealed and _pre_init_tree_config is dead,
  //    so setting only that would silently lose it there instead.
  //
  // Unlike the other tree_initialization setters this deliberately does NOT assert on a sealed
  // configuration: it changes no stored geometry, and being settable after a reopen is the point.
  {
    std::unique_lock<std::mutex> lock(_configuration_mutex);
    _pre_init_tree_config.mutable_dataset = value ? 1 : 0;
  }
  // _tree_registry belongs to the tree loop, which may be mid-checkpoint. Post rather than write
  // across threads -- same discipline as generate_lod.
  _event_loop.run_in_loop([this, value]() { _tree_registry.tree_config.mutable_dataset = value ? 1 : 0; });
}

bool tree_handler_t::is_mutable()
{
  std::unique_lock<std::mutex> lock(_configuration_mutex);
  return (_configuration_initialized ? _tree_registry.tree_config : _pre_init_tree_config).mutable_dataset != 0;
}

void tree_handler_t::set_tree_initialization_read_chunk_bytes(uint64_t bytes)
{
  std::unique_lock<std::mutex> lock(_configuration_mutex);
  assert(!_configuration_initialized);
  // The reader clamps to [node_point_limit, k_default_max_chunk_points] points; k_hard_max_chunk_points
  // bounds even deliberately huge requests (decompressed morton spikes scale with chunk size).
  _pre_init_tree_config.read_chunk_byte_target = bytes;
}

void tree_handler_t::about_to_block()
{
}

void tree_handler_t::handle_add_points(storage_header_t &&header, attributes_id_t &&attributes_id, std::vector<storage_location_t> &&storage)
{
  auto tree_start = std::chrono::steady_clock::now();
  if (!_initialized)
  {
    _initialized = true;
    seal_configuration();
    _tree_registry.root = tree_initialize(_tree_registry, _file_cache, header, attributes_id, std::move(storage));
  }
  else
  {
    _tree_registry.root = tree_add_points(_tree_registry, _file_cache, _tree_registry.root, header, attributes_id, std::move(storage));
  }
  auto tree_end = std::chrono::steady_clock::now();
  auto tree_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(tree_end - tree_start).count());
  _perf_stats.tree_build_time_us.fetch_add(tree_us, std::memory_order_relaxed);

  auto to_send = header.input_id;
  _done_with_input.post_event(std::move(to_send));
}

void tree_handler_t::generate_lod(const morton::morton192_t &max)
{
  // Hop to the tree loop: the pass target and the generator's batch state are tree-loop state, and
  // the caller (the processor advancing the watermark) may race an in-flight checkpoint chain here.
  auto copy = max;
  _generate_lod_pipe.post_event(std::move(copy));
}

void tree_handler_t::handle_generate_lod(morton::morton192_t &&max)
{
  if (std::getenv("DEW_DEBUG_CHAIN"))
    fmt::print(stderr, "[sched] handle_generate_lod target={}\n", max.data[0]);
  _perf_stats.lod_start = perf_stats_t::clock_t::now();
  _perf_stats.lod_phase.store(true, std::memory_order_release);
  // Record the pass TARGET; it is only promoted to the finality watermark when the pass completes
  // (handle_serialize_trees) -- marking against an in-flight target would finalize trees whose LOD
  // writes haven't landed yet.
  if (!_has_pending_pass_watermark || _pending_pass_watermark < max)
  {
    _pending_pass_watermark = max;
    _has_pending_pass_watermark = true;
  }
  // Collapse first: leaves of trees this pass will finalize are already immutable, and the LOD
  // level-5 sampling below then reads the small per-node units instead of whole ingest chunks.
  // on_done runs on the tree loop.
  _tree_collapse.collapse_for_pass(max, [this, max]() { _tree_lod_generator.generate_lods(_tree_registry.root, max); });
}

void tree_handler_t::set_input_registry_snapshot_provider(std::function<std::vector<uint8_t>()> provider)
{
  _input_registry_snapshot_provider = std::move(provider);
}

void tree_handler_t::set_band_sink(std::function<void(band_job_t &&)> sink, uint32_t next_band_id, bool terminal_band_committed)
{
  _band_sink = std::move(sink);
  _next_band_id = next_band_id;
  _terminal_band_emitted = terminal_band_committed;
}

void tree_handler_t::restore_uploaded_trees(const std::vector<std::pair<uint32_t, uint32_t>> &tree_and_band)
{
  for (auto &[tree_id, band_id] : tree_and_band)
  {
    if (tree_id < _tree_registry.tree_state.size())
    {
      _tree_registry.tree_state[tree_id] = uint8_t(tree_state_t::uploaded);
      _tree_registry.tree_band[tree_id] = band_id;
    }
  }
  // A tree the cache thought was banded but the bucket doesn't know about (crash between band
  // assignment and commit) must be re-banded: clear its assignment if it is merely final.
  for (uint32_t i = 0; i < uint32_t(_tree_registry.tree_state.size()); i++)
  {
    if (_tree_registry.tree_state[i] == uint8_t(tree_state_t::final) && _tree_registry.tree_band[i] != tree_band_none)
      _tree_registry.tree_band[i] = tree_band_none;
  }
}

void tree_handler_t::mark_band_uploaded(uint32_t band_id, std::vector<uint32_t> tree_ids)
{
  _event_loop.run_in_loop([this, band_id, tree_ids = std::move(tree_ids)]() {
    for (auto tree_id : tree_ids)
    {
      if (tree_id < _tree_registry.tree_state.size())
      {
        _tree_registry.tree_state[tree_id] = uint8_t(tree_state_t::uploaded);
        _tree_registry.tree_band[tree_id] = band_id;
      }
    }
  });
}

void tree_handler_t::emit_band_job()
{
  if (!_band_sink)
    return;
  // Suppressing the finality flip alone is NOT enough: job.terminal is derived from the watermark
  // rather than from any tree, and the trees-empty early-out below lets a terminal job through --
  // which writes the registry snapshot and seals the bucket. Nothing ships until finalize.
  if (_tree_registry.tree_config.mutable_dataset)
    return;
  band_job_t job;
  job.band_id = _next_band_id;
  job.watermark = _tree_registry.lod_watermark;
  morton::morton192_t terminal;
  memset(&terminal, 0xFF, sizeof(terminal));
  job.terminal = !(_tree_registry.lod_watermark < terminal);
  for (uint32_t i = 0; i < uint32_t(_tree_registry.tree_state.size()); i++)
  {
    if (_tree_registry.tree_state[i] == uint8_t(tree_state_t::final) && _tree_registry.tree_band[i] == tree_band_none)
    {
      // The tree's serialized blob location is committed (finality is marked in the same checkpoint
      // that serialized it); the uploader reads the immutable bytes straight from the cache.
      job.trees.emplace_back(i, _tree_registry.locations[i]);
      _tree_registry.tree_band[i] = job.band_id; // provisional; the bucket is authoritative on resume
    }
  }
  if (job.trees.empty() && (!job.terminal || _terminal_band_emitted))
    return;
  auto attributes = _attributes_configs.serialize();
  job.attributes_snapshot.assign(attributes.data.get(), attributes.data.get() + attributes.size);
  if (job.terminal)
    job.registry_snapshot = tree_registry_serialize(_tree_registry);
  _next_band_id++;
  if (job.terminal)
    _terminal_band_emitted = true;
  _band_sink(std::move(job));
}

tree_config_t tree_handler_t::tree_config()
{
  seal_configuration();
  std::unique_lock<std::mutex> lock(_configuration_mutex);
  return _tree_registry.tree_config;
}

void tree_handler_t::request_aabb(std::function<void(double *, double *)> function)
{
  _request_aabb.post_event(std::move(function));
}

void tree_handler_t::request_trees_async(std::vector<tree_id_t> tree_ids)
{
  if (tree_ids.empty())
    return;
  _request_trees_batch.post_event(std::move(tree_ids));
}

void tree_handler_t::handle_request_trees_batch(std::vector<tree_id_t> &&tree_ids)
{
  // During teardown (begin_shutdown) stop enqueuing new tree-load tasks: the shared thread pool is about to
  // be drained, and enqueue-after-stop aborts.
  if (_shutting_down.load(std::memory_order_acquire))
    return;
  _tree_id_requested.resize(_tree_registry.data.size());
  for (auto &tree_id : tree_ids)
  {
    if (_tree_id_requested[tree_id.data])
      continue;
    _tree_id_requested[tree_id.data] = 1;
    auto location = _tree_registry.locations[tree_id.data];
    auto req = _file_cache.read(location);
#ifdef __EMSCRIPTEN__
    // No thread pool on wasm: drive the read as a detached coroutine that suspends on the async read
    // (co_await await_on) instead of parking a pool thread on wait_for_read. req is captured by value
    // (shared_ptr) so the read_request stays alive across the suspension.
    [](tree_handler_t *self, std::shared_ptr<read_request_t> req, tree_id_t tree_id) -> vio::detached_task_t
    {
      co_await req->await_on(self->_event_loop);
      if (req->error.code != 0)
      {
        fmt::print("Error reading tree\n");
        co_return;
      }
      serialized_tree_t data;
      data.size = int(req->buffer_info.size);
      data.data = req->buffer;
      self->_deserialize_tree.post_event(tree_id_t(tree_id.data), std::move(data));
    }(this, req, tree_id);
#else
    _thread_pool.enqueue([this, req, tree_id]() {
      req->wait_for_read();
      if (req->error.code != 0)
      {
        fmt::print("Error reading tree\n");
        return;
      }
      serialized_tree_t data;
      data.size = int(req->buffer_info.size);
      data.data = req->buffer;
      this->_deserialize_tree.post_event(tree_id_t(tree_id.data), std::move(data));
    });
#endif
  }
}

void tree_handler_t::handle_serialize_trees()
{
  // This pipe is posted by the LOD generator strictly at pass COMPLETION, so the pending pass
  // target is now safe to use for finality marking. (The cache-pressure checkpoint path goes
  // through handle_checkpoint_request instead and skips this promotion.)
  if (_has_pending_pass_watermark && (!_has_pass_watermark || _pass_watermark < _pending_pass_watermark))
  {
    _pass_watermark = _pending_pass_watermark;
    _has_pass_watermark = true;
  }
  launch_serialize_chain();
}

void tree_handler_t::request_checkpoint()
{
  _checkpoint_request.post_event();
}

void tree_handler_t::handle_checkpoint_request()
{
  // No watermark promotion: this checkpoint exists to make pending remote facts durable (and
  // persist current tree/registry state), not to advance finality.
  launch_serialize_chain();
}

void tree_handler_t::launch_serialize_chain()
{
  if (_serialize_in_flight)
  {
    // Coalesce: one rerun after the in-flight chain commits covers every trigger that landed
    // meanwhile (the chain serializes current state, not a queued snapshot).
    _serialize_rerun = true;
    return;
  }
  _serialize_in_flight = true;
  [](tree_handler_t *self) -> vio::detached_task_t
  {
    co_await self->do_serialize_trees();
    self->_serialize_in_flight = false;
    {
      // Count finished chains regardless of outcome so checkpoint_and_wait never hangs on a
      // failed commit (commit success is counted separately in _commits_seen).
      std::unique_lock<std::mutex> lock(self->_band_emission_mutex);
      self->_serialize_rounds++;
    }
    self->_band_emission_cv.notify_all();
    if (self->_serialize_rerun)
    {
      self->_serialize_rerun = false;
      self->launch_serialize_chain();
    }
  }(this);
}

void tree_handler_t::checkpoint_and_wait()
{
  // NOTE: if a chain was already in flight, our request only coalesces into a rerun and the first
  // FINISHED chain may have snapshotted state older than this call; callers needing call-time
  // coverage (wait_idle's quiesce sequence) simply call twice.
  uint64_t target;
  {
    std::unique_lock<std::mutex> lock(_band_emission_mutex);
    target = _serialize_rounds + 1;
  }
  request_checkpoint();
  std::unique_lock<std::mutex> lock(_band_emission_mutex);
  _band_emission_cv.wait(lock, [&] { return _serialize_rounds >= target; });
}

vio::task_t<void> tree_handler_t::do_serialize_trees()
{
  const bool chain_debug = std::getenv("DEW_DEBUG_CHAIN") != nullptr;
  if (chain_debug)
    fmt::print(stderr, "[chain] enter\n");
  // Step 0: Mark finality BEFORE serializing, so the state lands in this checkpoint's registry
  // blob atomically with the trees' last serialization. A tree whose morton_max is strictly below
  // the pass watermark can never receive points or LOD again (inputs dispatch in rising min-morton
  // order; the LOD pass that produced this checkpoint already generated every LOD node below the
  // watermark, including the parent-tree nodes that sample this tree). The terminal pass
  // (all-0xFF watermark) finalizes everything -- the root cell's morton_max equals the sentinel,
  // so strict '<' alone would leave it building forever. Only loaded trees are marked; an
  // unloaded (lazily-not-yet-read) tree on a resumed session keeps its persisted state and is
  // finalized by a later pass or the terminal one.
  if (_has_pass_watermark)
  {
    morton::morton192_t terminal;
    memset(&terminal, 0xFF, sizeof(terminal));
    const bool terminal_pass = !(_pass_watermark < terminal);
    if (_tree_registry.lod_watermark < _pass_watermark)
      _tree_registry.lod_watermark = _pass_watermark;
    for (auto &tree : _tree_registry.data)
    {
      if (!tree)
        continue;
      auto &state = _tree_registry.tree_state[tree->id.data];
      // leaves_collapsed gates finality: bands must only ever ship per-node units, and a building
      // tree loaded after its range passed the watermark must not be flipped final by a mid-pass
      // cache-pressure checkpoint before the collapse phase has seen it.
      // A MUTABLE dataset is never sealed: finality is what makes a tree uploadable and evictable,
      // and an evicted tree cannot be amended. dew_converter_finalize clears the flag and the next
      // checkpoint seals in the ordinary way.
      if (!_tree_registry.tree_config.mutable_dataset && state == uint8_t(tree_state_t::building) && tree->leaves_collapsed &&
          (terminal_pass || tree->morton_max < _pass_watermark))
        state = uint8_t(tree_state_t::final);
    }
  }

  // Step 1: Serialize dirty trees. A finalized tree must never be dirty again -- its serialized
  // form from its finalizing checkpoint is immutable (the upload/eviction tiers depend on this).
  std::vector<tree_id_t> tree_ids;
  std::vector<serialized_tree_t> serialized_trees;
  // Blobs the trees stopped referencing (regenerated LOD nodes, split leaf chunks). Taken in the
  // same synchronous block as the tree serialization so the freed set exactly matches the tree
  // snapshots this checkpoint commits; restored on failure so no discard is ever lost.
  std::vector<std::pair<tree_t *, std::vector<storage_location_t>>> taken_discards;
  std::vector<tree_t *> serialized_tree_ptrs;
  auto restore_failed_serialize = [&]() {
    for (auto &[tree, discards] : taken_discards)
      tree->storage_map.restore_discarded(std::move(discards));
    taken_discards.clear();
    for (auto *tree : serialized_tree_ptrs)
      tree->is_dirty = true; // stays in every later checkpoint until a write succeeds
  };
  for (auto &tree : _tree_registry.data)
  {
    if (tree && tree->is_dirty)
    {
      tree_ids.emplace_back(tree->id);
      serialized_trees.emplace_back(tree_serialize(*tree));
      if (serialized_trees.back().data == nullptr)
      {
        fmt::print(stderr, "Error serializing tree\n");
        restore_failed_serialize();
        co_return;
      }
      auto discards = tree->storage_map.take_discarded();
      if (!discards.empty())
        taken_discards.emplace_back(tree.get(), std::move(discards));
      // Clear dirty HERE, in the same synchronous block as the serialization -- not after the
      // write lands. The chain suspends at every co_await below, and tree-loop work (an LOD
      // batch's adjust) can dirty the tree meanwhile; a post-write clear would wipe that flag and
      // the mutation would never reach another checkpoint (observed as top-level LOD nodes stuck
      // as count=0 placeholders with their generated blobs orphaned). On write failure the flag
      // is restored, so nothing is dropped there either.
      tree->is_dirty = false;
      serialized_tree_ptrs.push_back(tree.get());
    }
  }

  // Refresh the input-registry snapshot embedded in the registry blob (resume support).
  if (_input_registry_snapshot_provider)
    _tree_registry.input_registry_snapshot = _input_registry_snapshot_provider();

  // Step 2: Write trees to storage (cross-loop call via callback)
  callback_awaitable_t<write_trees_result_t> write_trees_awaitable(_event_loop);
  {
    auto state = write_trees_awaitable._state;
    _file_cache.write_trees(std::move(tree_ids), std::move(serialized_trees),
      [state](std::vector<tree_id_t> &&ids, std::vector<storage_location_t> &&locs, dew_error_t &&err)
      {
        state->result.tree_ids = std::move(ids);
        state->result.locations = std::move(locs);
        state->result.error = std::move(err);
        state->caller_loop.run_in_loop([state] { state->continuation.resume(); });
      });
  }
  if (chain_debug)
    fmt::print(stderr, "[chain] awaiting write_trees ({} trees)\n", tree_ids.size());
  auto trees_result = co_await write_trees_awaitable;
  if (chain_debug)
    fmt::print(stderr, "[chain] write_trees done err={}\n", trees_result.error.code);
  if (trees_result.error.code != 0)
  {
    // Trees turn dirty again; the next checkpoint retries them (with their discards restored).
    fmt::print(stderr, "Error writing trees: {}\n", trees_result.error.msg);
    restore_failed_serialize();
    co_return;
  }

  // Step 3: Update tree registry with new locations; writes durable -> now clear the dirty flags.
  std::vector<storage_location_t> old_locations;
  for (auto &[tree, discards] : taken_discards)
  {
    (void)tree;
    old_locations.insert(old_locations.end(), discards.begin(), discards.end());
  }
  taken_discards.clear();
  for (int i = 0; i < int(trees_result.tree_ids.size()); i++)
  {
    auto &tree_id = trees_result.tree_ids[i];
    // (is_dirty was cleared at serialization time -- clearing it here would lose a re-dirty that
    // happened while the writes above were in flight.)
    auto &location = _tree_registry.locations[tree_id.data];
    if (location.offset > 0)
    {
      old_locations.emplace_back(location);
    }
    location = trees_result.locations[i];
  }

  // Step 4: Serialize and write tree registry
  auto serialized_registry = tree_registry_serialize(_tree_registry);
  callback_awaitable_t<write_tree_registry_result_t> write_registry_awaitable(_event_loop);
  {
    auto state = write_registry_awaitable._state;
    _file_cache.write_tree_registry(std::move(serialized_registry),
      [state](storage_location_t loc, dew_error_t &&err)
      {
        state->result.location = loc;
        state->result.error = std::move(err);
        state->caller_loop.run_in_loop([state] { state->continuation.resume(); });
      });
  }
  if (chain_debug)
    fmt::print(stderr, "[chain] awaiting write_registry\n");
  auto registry_result = co_await write_registry_awaitable;
  if (chain_debug)
    fmt::print(stderr, "[chain] write_registry done err={}\n", registry_result.error.code);

  // Step 5: Write blob locations and update header. The completion callback runs on the STORAGE
  // loop strictly BEFORE the handler posts the index-written event (see
  // do_write_blob_locations_and_update_header), so publishing the committed watermark here is
  // guaranteed visible to the processor's index-written handler -- which uses it to tell a
  // pass-concluding commit apart from a mid-pass cache-pressure checkpoint.
  callback_awaitable_t<write_blob_result_t> write_blob_awaitable(_event_loop);
  {
    auto state = write_blob_awaitable._state;
    auto committed_watermark = _tree_registry.lod_watermark;
    _file_cache.write_blob_locations_and_update_header(registry_result.location, std::move(old_locations),
      [state, this, committed_watermark](dew_error_t &&err)
      {
        if (err.code == 0)
        {
          {
            std::unique_lock<std::mutex> lock(_committed_watermark_mutex);
            _last_committed_watermark = committed_watermark;
          }
          std::unique_lock<std::mutex> lock(_band_emission_mutex);
          _commits_seen++;
        }
        state->result.error = std::move(err);
        state->caller_loop.run_in_loop([state] { state->continuation.resume(); });
      });
  }
  if (chain_debug)
    fmt::print(stderr, "[chain] awaiting blob_locations/index\n");
  auto blob_result = co_await write_blob_awaitable;
  if (chain_debug)
    fmt::print(stderr, "[chain] index committed err={}\n", blob_result.error.code);
  if (blob_result.error.code != 0)
  {
    fmt::print(stderr, "Error committing checkpoint index: {}\n", blob_result.error.msg);
    co_return;
  }
  // (The committed watermark was already published from the storage-side completion callback,
  // happens-before the index-written event.)
  // Everything this checkpoint finalized is now derivable from COMMITTED state -- band it, then
  // complete the commit->emission handshake wait_idle relies on.
  emit_band_job();
  {
    std::unique_lock<std::mutex> lock(_band_emission_mutex);
    _emissions_done++;
  }
  _band_emission_cv.notify_all();
}

void tree_handler_t::handle_deserialize_tree(tree_id_t &&tree_id, serialized_tree_t &&data)
{
  assert(_tree_registry.get(tree_id) == nullptr);
  _tree_registry.data[tree_id.data] = std::make_unique<tree_t>();
  auto tree = _tree_registry.get(tree_id);
  assert(tree);
  dew_error_t error;
  auto ret = tree_deserialize(data, *tree, error);
  if (ret)
    tree_compute_leaves_collapsed(*tree, _tree_registry);
    #ifndef NDEBUG
    tree_compute_mins(*tree); // debug-only shadow, never serialized -- see tree.hpp
    #endif
  if (!ret)
  {
    fmt::print("Error deserializing tree registry {}\n", error.msg);
    return;
  }
  _tree_registry.tree_id_initialized.resize(_tree_registry.data.size());
  std::atomic_ref<uint8_t>(_tree_registry.tree_id_initialized[tree_id.data]).store(1, std::memory_order_release);
  if (tree_id.data == _tree_registry.root.data)
  {
    std::unique_lock<std::mutex> lock(_root_mutex);
    _first_root_initialized = true;
    _root_cv.notify_all();
  }
}

void tree_handler_t::handle_request_aabb(std::function<void(double *, double *)> &&function)
{
  const auto &offset = _tree_registry.tree_config.offset;
  const auto &scale = _tree_registry.tree_config.scale;
  double min[3];
  double max[3];

  // Guard against a request before any tree/data exists (would dereference a null/OOB root).
  if (_tree_registry.data.empty() || _tree_registry.root.data >= _tree_registry.data.size() || !_tree_registry.data[_tree_registry.root.data])
  {
    min[0] = min[1] = min[2] = 0.0;
    max[0] = max[1] = max[2] = 0.0;
    function(min, max);
    return;
  }

  auto tree = _tree_registry.get(_tree_registry.root);

  morton::morton192_t morton_max = {};
  morton::morton192_t morton_min = morton::morton_negate(morton_max);
  bool found = false;
  // Points are held at the coarsest node they fit under node_limit — for a small cloud that
  // is data[0][0], not data[4]. Scan every level, skipping empty collections (whose min/max
  // are uninitialized). Only ever inspect populated collections.
  for (int level = 0; level < 5; level++)
  {
    for (auto &data : tree->data[level])
    {
      if (data.point_count == 0)
        continue;
      found = true;
      if (data.min < morton_min)
        morton_min = data.min;
      if (morton_max < data.max)
        morton_max = data.max;
    }
  }
  if (!found)
  {
    // Root holds no point collections directly (all data pushed into sub-trees, or none yet).
    // Fall back to the root cell's bounds — a valid, if loose, box — never an inverted min>max.
    morton_min = tree->morton_min;
    morton_max = tree->morton_max;
  }
  convert_morton_to_pos(scale, offset, morton_min, min);
  convert_morton_to_pos(scale, offset, morton_max, max);
  function(min, max);
}

void tree_handler_t::handle_request_root()
{
  handle_request_trees_batch(std::vector<tree_id_t>{_tree_registry.root});
}
} // namespace dew::converter
