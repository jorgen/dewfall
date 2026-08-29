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
#pragma once

#include <vio/awaitable_event_pipe.h>
#include <vio/event_loop.h>
#include <vio/event_pipe.h>
#include <vio/task.h>
#include <vio/thread_pool.h>

#include <atomic>

#include "perf_stats.hpp"
#include "tree.hpp"
#include "tree_collapse.hpp"
#include "tree_lod_generator.hpp"
#include "upload_handler.hpp" // band_job_t

namespace dew::converter
{
using namespace dew::core;
class storage_handler_t;

struct waiting_for_root_t
{
  std::condition_variable *cv;
};

class tree_handler_t : public vio::about_to_block_t
{
public:
  tree_handler_t(vio::thread_pool_t &thread_pool, storage_handler_t &file_cache, attributes_configs_t &attributes_configs, perf_stats_t &perf_stats, vio::event_pipe_t<input_data_id_t> &done_input);
  ~tree_handler_t();
  // Ordered-teardown hooks (see processor_t::~processor_t). begin_shutdown() stops the tree loop from
  // enqueuing new tree-load tasks onto the shared thread pool -- it must run, and the pool must then drain,
  // while this loop is still alive (parked pool tasks post their results back here). stop_loop() then joins
  // the loop thread before _tree_registry / the event pipes it touches are destroyed.
  void begin_shutdown();
  void stop_loop();
  [[nodiscard]] dew_error_t deserialize_tree_registry(std::unique_ptr<uint8_t[]> &tree_registry_buffer, uint32_t tree_registry_blobs_size);
  void request_root();
  void set_tree_initialization_config(const tree_config_t &config);
  void set_tree_initialization_scale(double scale);
  // Read the pre-init config WITHOUT sealing it (for consumers that only need provisional values
  // while the first batch's source scale may still be adopted).
  tree_config_t tree_config_peek()
  {
    std::unique_lock<std::mutex> lock(_configuration_mutex);
    return _configuration_initialized ? _tree_registry.tree_config : _pre_init_tree_config;
  }
  void set_tree_initialization_node_point_limit(uint32_t limit);
  void set_tree_initialization_read_chunk_bytes(uint64_t bytes);
  // Mutable mode: never flip a tree to `final`, and emit no bands. Unlike the tree_initialization
  // setters above, this is legal AFTER the configuration is sealed -- it changes no stored geometry,
  // and the whole point is to set it on a dataset that has already been converted and reopened.
  // Posted to the tree loop, which owns _tree_registry; the next checkpoint persists it.
  void set_mutable(bool value);
  [[nodiscard]] bool is_mutable();
  void about_to_block() override;
  // Thread-safe: posts to the tree loop. The pass target/generator state belong to the tree loop,
  // which may be mid-checkpoint (serialize chain, band emission) when the processor advances the
  // watermark -- a direct cross-thread call raced that state and could quiesce the pass.
  void generate_lod(const morton::morton192_t &max);
  // Cache-pressure checkpoint: serialize + commit WITHOUT promoting the pending pass watermark (no
  // LOD pass completed; marking against an in-flight target would finalize trees whose LOD writes
  // haven't landed). Safe to call from any thread; debounced by the serialize-in-flight guard.
  void request_checkpoint();
  // Watermark of the last checkpoint whose index write COMMITTED. The processor compares this
  // against its in-flight LOD target to tell a pass-concluding commit apart from a cache-pressure
  // checkpoint that landed mid-pass (which must NOT conclude the pass). Any-thread safe.
  morton::morton192_t last_committed_watermark() const
  {
    std::unique_lock<std::mutex> lock(_committed_watermark_mutex);
    return _last_committed_watermark;
  }
  // Installed by the processor: returns the serialized input_data_source_registry snapshot to embed
  // in the registry blob (v2) at each checkpoint. Called on the tree loop; must be thread-safe.
  void set_input_registry_snapshot_provider(std::function<std::vector<uint8_t>()> provider);
  // Upload tier: after every committed checkpoint, newly-final unbanded trees are assembled into a
  // band job and handed to this sink (typically upload_handler_t::enqueue_band; thread-safe).
  // next_band_id seeds numbering on resume (= the bucket's committed band count).
  void set_band_sink(std::function<void(band_job_t &&)> sink, uint32_t next_band_id, bool terminal_band_committed);
  // Resume: mark these trees as already uploaded (from the bucket's band manifests).
  void restore_uploaded_trees(const std::vector<std::pair<uint32_t, uint32_t>> &tree_and_band);
  // Mark a committed band's trees uploaded (thread-safe; hops to the tree loop).
  void mark_band_uploaded(uint32_t band_id, std::vector<uint32_t> tree_ids);
  tree_config_t tree_config();
  void request_aabb(std::function<void(double *, double *)> function);
  void request_trees_async(std::vector<tree_id_t> tree_ids);

  const tree_registry_t &tree_registry() const { return _tree_registry; }
  const attributes_configs_t &attributes_configs() const { return _attributes_configs; }
  // True once the tree configuration is fixed (sealed by first use, or restored from a reopened
  // dataset's registry): set_tree_initialization_config must not be called any more.
  bool configuration_initialized()
  {
    std::unique_lock<std::mutex> lock(_configuration_mutex);
    return _configuration_initialized;
  }

private:
  void handle_add_points(storage_header_t &&header, attributes_id_t &&attributes_id, std::vector<storage_location_t> &&storage);
  void handle_generate_lod(morton::morton192_t &&max);
  void handle_serialize_trees();
  void handle_checkpoint_request();
  // Launch do_serialize_trees with the in-flight guard (tree loop only). Overlapping triggers (LOD
  // completion vs cache-pressure requests) coalesce into a rerun instead of racing the chain.
  void launch_serialize_chain();
  vio::task_t<void> do_serialize_trees();
  void handle_deserialize_tree(tree_id_t &&tree_id, serialized_tree_t &&data);
  void handle_request_aabb(std::function<void(double *, double *)> &&function);
  void handle_request_root();
  void handle_request_trees_batch(std::vector<tree_id_t> &&tree_ids);

  void seal_configuration()
  {
    std::unique_lock<std::mutex> lock(_configuration_mutex);
    if (_configuration_initialized)
      return;
    _configuration_initialized = true;
    _tree_registry.tree_config = _pre_init_tree_config;
    // node_point_limit is the single source of truth for both the read/sort chunk size and the octree
    // node capacity; seed the registry's node_limit from it.
    _tree_registry.node_limit = _pre_init_tree_config.node_point_limit;
  }

  vio::thread_pool_t &_thread_pool;
  vio::thread_with_event_loop_t _event_loop_thread;
  vio::event_loop_t &_event_loop;
  std::atomic<bool> _shutting_down{false}; // set by begin_shutdown(): stop enqueuing tree-load pool tasks
  std::mutex _configuration_mutex;
  bool _initialized;
  bool _configuration_initialized;
  tree_config_t _pre_init_tree_config;

  std::mutex _root_mutex;
  std::condition_variable _root_cv;
  bool _first_root_initialized;

  storage_handler_t &_file_cache;
  attributes_configs_t &_attributes_configs;
  perf_stats_t &_perf_stats;

  tree_registry_t _tree_registry;
  std::vector<uint8_t> _tree_id_requested;

  // Done-morton watermark plumbing for finality marking (monotone max across passes). generate_lod
  // stores the TARGET into _pending_pass_watermark; handle_serialize_trees -- posted by the LOD
  // generator strictly at pass COMPLETION -- promotes it into _pass_watermark, which
  // do_serialize_trees then uses to mark trees with morton_max strictly below it as final, inside
  // the same atomic checkpoint that serializes them. The split matters: a checkpoint taken while a
  // pass is still in flight (e.g. a cache-pressure checkpoint request) must mark against the last
  // COMPLETED pass, or it would finalize trees whose LOD writes haven't landed. Terminal = all-0xFF.
  morton::morton192_t _pending_pass_watermark = {};
  morton::morton192_t _pass_watermark = {};
  bool _has_pending_pass_watermark = false;
  bool _has_pass_watermark = false;
  // Serialize-chain guard (tree loop only): checkpoint triggers arriving mid-chain coalesce.
  bool _serialize_in_flight = false;
  bool _serialize_rerun = false;
  mutable std::mutex _committed_watermark_mutex;
  morton::morton192_t _last_committed_watermark = {};
  // Upload tier (tree loop only, except the sink itself which must be thread-safe).
  std::function<void(band_job_t &&)> _band_sink;
  uint32_t _next_band_id = 0;
  // At the terminal watermark every commit would otherwise emit a (tree-less) terminal band; the
  // registry/complete flag only needs to land once. Restored from the bucket on resume.
  bool _terminal_band_emitted = false;
  void emit_band_job(); // called on the tree loop after each committed checkpoint
  // Commit -> band-emission handshake: every committed checkpoint (counted in the storage-side
  // completion callback, happens-before the index-written event) is followed by exactly one
  // band-emission evaluation on the tree loop. wait_band_emissions() lets wait_idle close the gap
  // where the processor sees the commit before the tree loop has handed the band to the uploader.
  std::mutex _band_emission_mutex;
  std::condition_variable _band_emission_cv;
  uint64_t _commits_seen = 0;
  uint64_t _emissions_done = 0;
  uint64_t _serialize_rounds = 0; // chains FINISHED (committed or errored); lets waiters not hang on errors

public:
  void wait_band_emissions()
  {
    std::unique_lock<std::mutex> lock(_band_emission_mutex);
    _band_emission_cv.wait(lock, [this] { return _emissions_done >= _commits_seen; });
  }
  uint64_t commits_seen()
  {
    std::unique_lock<std::mutex> lock(_band_emission_mutex);
    return _commits_seen;
  }
  // Request one checkpoint and block until its serialize chain finishes (commit OR error). Used by
  // wait_idle in destination mode as the final quiesce checkpoint: it persists the residency facts
  // the terminal band produced (uploaded blobs, spill derefs), so the post-commit sweep can delete
  // dead spill segments, punch evicted bytes and settle the cache at/under its cap -- and a reopen
  // resumes from committed-uploaded state instead of reconciling everything from the bucket.
  void checkpoint_and_wait();

private:
  std::function<std::vector<uint8_t>()> _input_registry_snapshot_provider;

  tree_lod_generator_t _tree_lod_generator;
  tree_collapse_runner_t _tree_collapse;

  std::vector<std::function<void()>> _tree_deserialized_callbacks;

public:
  vio::event_pipe_t<storage_header_t, attributes_id_t, std::vector<storage_location_t>> add_points;
  vio::event_pipe_t<morton::morton192_t> _generate_lod_pipe;
  vio::event_pipe_t<void> _serialize_trees;
  vio::event_pipe_t<void> _checkpoint_request;
  vio::event_pipe_t<tree_id_t, serialized_tree_t> _deserialize_tree;
  vio::event_pipe_t<input_data_id_t> &_done_with_input;
  vio::event_pipe_t<std::function<void(double *, double *)>> _request_aabb;
  vio::event_pipe_t<void> _request_root;
  vio::event_pipe_t<std::vector<tree_id_t>> _request_trees_batch;

private:
};
} // namespace dew::converter
