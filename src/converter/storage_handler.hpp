/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2021  Jørgen Lind
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

#include <vio/event_loop.h>
#include <vio/event_pipe.h>
#ifndef __EMSCRIPTEN__
#include <vio/operation/file.h> // pulls libuv; the storage handler drives IO through the backend, not vio file ops
#endif
#include <vio/operation/work.h>
#include <vio/task.h>
#include <vio/thread_pool.h>

#include "attributes_configs.hpp"
#include "compressor.hpp"
#include "dataset_types.hpp"
#include "error.hpp"
#include "blob_reader.hpp"
#include "perf_stats.hpp"
#include "storage_backend.hpp"
#include "tree.hpp"
#include <ankerl/unordered_dense.h>

#include <atomic>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <set>

#include <cstdint>

namespace dew::converter
{
using namespace dew::core;

struct compressed_write_data_t
{
  int buffer_index;
  std::shared_ptr<uint8_t[]> data;
  uint32_t size;
  std::string attribute_name;
  point_format_t format;
  uint32_t uncompressed_size;
  double min_value = std::numeric_limits<double>::max();
  double max_value = std::numeric_limits<double>::lowest();
  bool is_lod = false;
};

class storage_handler_t;

// One attribute buffer destined for a unit that ALREADY has storage. Parallel to one entry of an
// attribute_buffers_t, minus everything a whole-unit write needs and an amend does not.
struct attribute_write_t
{
  std::shared_ptr<uint8_t[]> data;
  uint32_t size = 0;
  uint32_t point_count = 0;
  point_format_t format{};
  std::string attribute_name;
  bool is_lod = false;
};

class storage_handler_t
{
public:
  storage_handler_t(const std::string &url, vio::thread_pool_t &thread_pool, attributes_configs_t &attributes_configs, perf_stats_t &perf_stats, vio::event_pipe_t<void> &index_written, vio::event_pipe_t<dew_error_t> &storage_error_pipe, dew_error_t &error);
  ~storage_handler_t();
  // Join the storage loop thread. Idempotent. Called by the processor's ordered teardown (and the destructor)
  // before _backend / _read_cache / the event pipes an in-flight read touches are destroyed.
  void stop_loop();
  [[nodiscard]] bool file_exists() const { return _reader.file_exists(); }
  [[nodiscard]] std::string file_exists_error() const { return _reader.file_exists_error(); }
  [[nodiscard]] dew_error_t read_index(std::unique_ptr<uint8_t[]> &free_blobs_buffer, uint32_t &free_blobs_size, std::unique_ptr<uint8_t[]> &attribute_configs_buffer, uint32_t &attribute_configs_size,
                                   std::unique_ptr<uint8_t[]> &tree_registry_buffer, uint32_t &tree_registry_size);
  [[nodiscard]] dew_error_t deserialize_free_blobs(const std::unique_ptr<uint8_t[]> &data, uint32_t size);
  [[nodiscard]] dew_error_t upgrade_to_write(bool truncate);

  void write(const storage_header_t &header, attributes_id_t attributes_id, attribute_buffers_t &&buffers,
             std::function<void(const storage_header_t &, attributes_id_t, std::vector<storage_location_t>, const dew_error_t &error)> done);
  // Write attribute buffers for units that already exist, and hand back just their locations, one per
  // input in order.
  //
  // write() above is the whole-unit path: it re-serializes slot 0 (the storage_header_t and the
  // morton codes travel with the position buffer) and compresses every buffer it is given. An amend
  // adds one attribute to units whose other blobs are already on disk and unchanged, so routing it
  // through write() would recompress all of them and mint new locations for blobs nobody asked to
  // move. This compresses and writes exactly the buffers handed to it, with the same preprocessing,
  // the same constant-band detection and the same compression-stats accounting.
  //
  // Never slot 0 -- only the whole-unit path knows how to build a position buffer's header.
  //
  // Batched deliberately: an amend touches many units at once, and one call per unit would serialize
  // the compression of thousands of buffers behind one another. The whole batch goes to the thread
  // pool in one shot, exactly as a single unit's buffers do.
  void write_attributes(std::vector<attribute_write_t> &&writes, std::function<void(std::vector<storage_location_t> &&, const dew_error_t &error)> done);
  void write_trees(std::vector<tree_id_t> &&tree_ids, std::vector<serialized_tree_t> &&serialized_trees, std::function<void(std::vector<tree_id_t> &&, std::vector<storage_location_t> &&, dew_error_t &&error)> done);
  void write_tree_registry(serialized_tree_registry_t &&serialized_tree_registry, std::function<void(storage_location_t, dew_error_t &&error)> done);
  void write_blob_locations_and_update_header(storage_location_t location, std::vector<storage_location_t> &&old_locations, std::function<void(dew_error_t &&error)> done);

  // decompress_inline: on a RAM-cache hit of a compressed blob the decompress normally hops to the
  // shared thread pool (a render-thread latency fix). Callers that ALREADY run on a pool worker and
  // synchronously wait_for_read (LOD + collapse readers) MUST pass true: with every pool thread
  // parked in such a wait, the hopped decompress queues behind the waiters and the pool deadlocks.
  std::shared_ptr<read_request_t> read(storage_location_t location, bool raw = false, bool decompress_inline = false)
  {
    return _reader.read(location, read_options_t{raw, decompress_inline, {}});
  }
  // Pass-through to the reader for callers that want the completion hook (see read_options_t).
  std::shared_ptr<read_request_t> read(storage_location_t location, read_options_t options)
  {
    return _reader.read(location, std::move(options));
  }
  [[nodiscard]] blob_reader_t &reader() { return _reader; }

  void register_input_file_size(uint32_t file_id, uint64_t size_bytes);
  void set_compressor(compression_method_t method);
  void set_compression_level(int level);
  void set_read_cache_size(uint64_t max_bytes) { _reader.set_read_cache_size(max_bytes); }
  void set_decompressed_cache_size(uint64_t max_bytes) { _reader.set_decompressed_cache_size(max_bytes); }
  uint64_t read_cache_current_bytes() { return _reader.read_cache_current_bytes(); }
  void set_on_write_progress(std::function<void()> cb) { _on_write_progress = std::move(cb); }
  // Cache-tier pressure: invoked (from the storage loop) at most once per arm when the backend
  // reports that only a checkpoint can relieve pressure. rearm after the checkpoint completes.
  void set_checkpoint_request_callback(std::function<void()> cb) { _on_checkpoint_request = std::move(cb); }
  void rearm_checkpoint_request() { _checkpoint_requested.store(false, std::memory_order_release); }
  // Upload tier: a band committed; mark its blobs uploaded in the cache tier (thread-safe; hops to
  // the storage loop). Each pair is (cache blob offset, bucket pack location).
  void note_blobs_uploaded(std::vector<std::pair<uint64_t, storage_location_t>> &&blobs);
  // Leaf collapse freed a source chunk's blobs: remove their contribution from the compression stats
  // (thread-safe; hops to the storage loop where the stats live).
  void note_source_blobs_freed(attributes_id_t attributes_id, std::vector<storage_location_t> locations, uint32_t point_count);
  // Block until every already-posted storage-loop task has run (FIFO barrier). NOT wasm-safe
  // (single-threaded); destination-mode teardown only.
  void drain_posted_events();

  // ---- cache-tier configuration (destination mode; requires a LOCAL packed cache file) ----
  // Enable residency tracking (cap 0 = track only) and, when destination_url is non-empty, the
  // spill area + destination reads on that bucket. Call at bootstrap, before read_index/open.
  [[nodiscard]] dew_error_t configure_cache_tier(uint64_t cap_bytes, const std::string &destination_url, const std::string &connection);
  // Runtime cap adjustment (thread-safe; lowering triggers a reclaim pass on the storage loop).
  void set_cache_max_bytes(uint64_t cap_bytes);
  // Returns the dataset uuid, generating + installing a fresh one if the cache has none yet.
  // Only valid in destination mode (packed backend).
  void ensure_dataset_uuid(uint8_t (&out)[16]);
  // After read_index/restore_allocator at bootstrap: replay spill liveness + GC orphan segments.
  [[nodiscard]] dew_error_t run_spill_bootstrap();
  // Mark the next checkpoint as the orderly-teardown one (keeps local_* residency on reopen).
  void set_clean_shutdown_next_checkpoint();
  struct cache_tier_stats_t
  {
    uint64_t resident_bytes = 0;
    uint64_t cap_bytes = 0;
    uint32_t tracked_blobs = 0;
    uint64_t spilled_bytes = 0;
  };
  // Approximate (unsynchronized counters); false when no cache tier is configured.
  bool get_cache_tier_stats(cache_tier_stats_t &out) const;

  const compression_stats_t &get_compression_stats() const { return _compression_stats; }
  const perf_stats_t::deserialized_perf_stats_t &get_deserialized_perf_stats() const { return _deserialized_perf_stats; }

private:
  void handle_write_events(
    std::tuple<storage_header_t, attributes_id_t, attribute_buffers_t, std::function<void(const storage_header_t &, attributes_id_t, std::vector<storage_location_t> &&, const dew_error_t &error)>> &&event);
  void handle_write_trees(std::tuple<std::vector<tree_id_t>, std::vector<serialized_tree_t>, std::function<void(std::vector<tree_id_t> &&, std::vector<storage_location_t> &&, dew_error_t &&)>> &&event);
  void handle_write_tree_registry(serialized_tree_registry_t &&serialized_trr, std::function<void(storage_location_t, dew_error_t &&error)> &&done);
  void handle_write_blob_locations_and_update_header(storage_location_t &&new_tree_registry_location, std::vector<storage_location_t> &&old_locations, std::function<void(dew_error_t &&error)> &&done);

  vio::task_t<void> do_write(const std::shared_ptr<uint8_t[]> &data, const storage_location_t &location);
  vio::task_t<void> do_write_events(storage_header_t header, attributes_id_t attributes_id, attribute_buffers_t attribute_buffers,
                                    std::function<void(const storage_header_t &, attributes_id_t, std::vector<storage_location_t> &&, const dew_error_t &error)> done);
  void handle_write_attributes(std::tuple<std::vector<attribute_write_t>, std::function<void(std::vector<storage_location_t> &&, const dew_error_t &)>> &&event);
  vio::task_t<void> do_write_attributes(std::vector<attribute_write_t> writes, std::function<void(std::vector<storage_location_t> &&, const dew_error_t &)> done);
  vio::task_t<void> do_write_trees(std::vector<tree_id_t> tree_ids, std::vector<serialized_tree_t> serialized_trees,
                                   std::function<void(std::vector<tree_id_t> &&, std::vector<storage_location_t> &&, dew_error_t &&)> done);
  vio::task_t<void> do_write_tree_registry(serialized_tree_registry_t serialized_tree_registry, std::function<void(storage_location_t, dew_error_t &&error)> done);
  vio::task_t<void> do_write_blob_locations_and_update_header(storage_location_t new_tree_registry_location, std::vector<storage_location_t> old_locations, std::function<void(dew_error_t &&error)> done);

  vio::thread_pool_t &_thread_pool;
  // Declared before every write pipe below so it outlives them: the pipes are bound to ITS event loop.
  blob_reader_t _reader;
  vio::event_loop_t &_event_loop;
  std::unique_ptr<compressor_t> _compressor;

  attributes_configs_t &_attributes_configs;

  perf_stats_t &_perf_stats;
  std::function<void()> _on_write_progress;
  std::function<void()> _on_checkpoint_request;
  std::atomic<bool> _checkpoint_requested{false};
  compression_stats_t _compression_stats;
  perf_stats_t::deserialized_perf_stats_t _deserialized_perf_stats{};
  std::set<uint32_t> _seen_input_files;
  ankerl::unordered_dense::map<uint32_t, uint64_t> _input_file_sizes;

  vio::event_pipe_t<void> &_index_written;
  vio::event_pipe_t<dew_error_t> &_storage_error;
  vio::event_pipe_t<std::tuple<storage_header_t, attributes_id_t, attribute_buffers_t, std::function<void(const storage_header_t &, attributes_id_t, std::vector<storage_location_t>, const dew_error_t &error)>>> _write_event_pipe;
  vio::event_pipe_t<std::tuple<std::vector<attribute_write_t>, std::function<void(std::vector<storage_location_t> &&, const dew_error_t &error)>>> _write_attributes_pipe;
  vio::event_pipe_t<std::tuple<std::vector<tree_id_t>, std::vector<serialized_tree_t>, std::function<void(std::vector<tree_id_t> &&, std::vector<storage_location_t> &&, dew_error_t &&error)>>> _write_trees_pipe;
  vio::event_pipe_t<serialized_tree_registry_t, std::function<void(storage_location_t, dew_error_t &&error)>> _write_tree_registry_pipe;
  vio::event_pipe_t<storage_location_t, std::vector<storage_location_t>, std::function<void(dew_error_t &&error)>> _write_blob_locations_and_update_header_pipe;


  std::mutex _mutex;
};

// deserialize_points moved to dataset_types.hpp (storage-free) so the decode path / a decode worker can
// use it without pulling this header.

struct read_only_points_t
{
  // retain_hot: false when this read consumes the unit WHOLE (the caller is its only reader), true
  // when it takes a subset (other nodes will read the other subsets of the same blob).
  read_only_points_t(storage_handler_t &storage_handler, storage_location_t a_location, bool retain_hot = true)
    : location(a_location)
    , read_request(storage_handler.read(location, read_options_t{/*raw=*/false, /*decompress_inline=*/true, {}, retain_hot}))
  {
    read_request->wait_for_read();
    // A failed read leaves data/header empty; callers MUST check `error` before dereferencing.
    error = read_request->error;
    if (error.code != 0)
      return;
    deserialize_points(read_request->buffer_info, header, data, error);
  }
  ~read_only_points_t()
  {
  }

  storage_location_t location;
  std::shared_ptr<read_request_t> read_request;
  storage_header_t header;
  dew_blob_t data;
  dew_error_t error;
};

struct read_attribute_t
{
  read_attribute_t(storage_handler_t &a_storage_handler, storage_location_t a_location)
    : storage_handler(a_storage_handler)
    , location(a_location)
    , read_request(a_storage_handler.read(a_location, /*raw=*/false, /*decompress_inline=*/true))
  {
    read_request->wait_for_read();
    // A failed read leaves data empty; callers MUST check `error` before dereferencing.
    error = read_request->error;
    if (error.code != 0)
      return;
    data = read_request->buffer_info;
  }

  storage_handler_t &storage_handler;
  storage_location_t location;
  std::shared_ptr<read_request_t> read_request;
  dew_blob_t data;
  dew_error_t error;
};

} // namespace dew::converter
