/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2024  Jørgen Lind
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
#include "processor.hpp"

#include "dataset_types.hpp"
#include "converter.hpp"
#include "loop_quiesce.hpp"
#include "frustum_tree_walker.hpp"

#include "morton_tree_coordinate_transform.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <functional>

namespace dew::converter
{
using namespace dew::core;
processor_t::processor_t(std::string url, file_existence_requirement_t existence_requirement, dew_error_t &error, const destination_config_t &destination)
  : _url(std::move(url))
  , _thread_pool(int(std::thread::hardware_concurrency()))
  , _runtime_callbacks({})
  , _runtime_callback_user_ptr(nullptr)
  , _convert_callbacks({})
  , _thread_with_event_loop()
  , _event_loop(_thread_with_event_loop.event_loop())
  , _generating_lod(false)
  , _has_errors(false)
  , _idle(true)
  , _new_file_events_sent(0)
  , _storage_handler(_url, _thread_pool, _attributes_configs, _perf_stats, _storage_index_write_done, _storage_handler_error, error)
  , _tree_handler(_thread_pool, _storage_handler, _attributes_configs, _perf_stats, _tree_done_with_input)
  , _files_added(_event_loop, bind(&processor_t::handle_new_files))
  , _input_init(_event_loop, bind(&processor_t::handle_input_init_done))
  , _sub_added(_event_loop, bind(&processor_t::handle_sub_added))
  , _sorted_points(_event_loop, bind(&processor_t::handle_sorted_points))
  , _point_reader_file_errors(_event_loop, bind(&processor_t::handle_file_errors))
  , _point_reader_done_with_file(_event_loop, bind(&processor_t::handle_file_reading_done))
  , _storage_index_write_done(_event_loop, bind(&processor_t::handle_index_write_done))
  , _storage_handler_error(_event_loop, bind(&processor_t::handle_storage_error))
  , _tree_done_with_input(_event_loop, bind(&processor_t::handle_tree_done_with_input))
  , _input_event_loop_thread()
  , _input_event_loop(_input_event_loop_thread.event_loop())
  , _point_reader(_input_event_loop, _thread_pool, _attributes_configs, _perf_stats, _input_init, _sub_added, _sorted_points, _point_reader_done_with_file, _point_reader_file_errors)
  , _read_sort_budget(uint64_t(1) << 30)
  , _read_sort_active_approximate_size(0)
{
  _destination = destination;
  _event_loop.add_about_to_block_listener(this);

  // Each checkpoint embeds an input-registry snapshot in the registry blob (v2) so a resumed
  // conversion can skip re-added done inputs and restore the done-morton watermark. serialize()
  // locks internally; the provider runs on the tree loop. Installed before any file can be added,
  // so no checkpoint can precede it.
  _tree_handler.set_input_registry_snapshot_provider([this] { return _input_data_source_registry.serialize(); });

  // Cache-tier pressure: when the storage backend can only be relieved by a checkpoint (pending
  // remote facts need their durable flip before eviction may punch), route the request to the tree
  // loop's non-promoting checkpoint entry. Fired from the storage loop; request_checkpoint is a
  // pipe post (thread-safe). Rearmed in handle_index_write_done.
  _storage_handler.set_checkpoint_request_callback([this] { _tree_handler.request_checkpoint(); });

  if (error.code != 0)
    return;

#ifndef __EMSCRIPTEN__
  if (!_destination.url.empty())
  {
    // Destination mode: residency tracking + spill/destination reads on the cache backend. Must be
    // configured before the index is read below (a reopened cache restores its residency table).
    error = _storage_handler.configure_cache_tier(_destination.cache_max_bytes, _destination.url, _destination.connection);
    if (error.code != 0)
      return;
  }
#endif

  if (existence_requirement == file_existence_requirement_t::exist)
  {
    if (!_storage_handler.file_exists())
    {
      auto probe_error = _storage_handler.file_exists_error();
      if (!probe_error.empty())
        error = {1, fmt::format("Failed to reach the dataset: {} (pass a connection string with region/credentials, e.g. region=eu-north-1;anonymous=true for a public bucket)", probe_error)};
      else
        error = {1, "File does not exist"};
      return;
    }
  }
  else if (existence_requirement == file_existence_requirement_t::not_exist)
  {
    if (_storage_handler.file_exists())
    {
      error = {1, "File exists"};
      return;
    }
  }
  if (_storage_handler.file_exists())
  {
    std::unique_ptr<uint8_t[]> free_blobs_buffer;
    uint32_t free_blobs_buffer_size = 0;
    std::unique_ptr<uint8_t[]> attribute_blobs_buffer;
    uint32_t attribute_blobs_buffer_size = 0;
    std::unique_ptr<uint8_t[]> tree_registry_buffer;
    uint32_t tree_registry_blobs_size = 0;
    error = _storage_handler.read_index(free_blobs_buffer, free_blobs_buffer_size, attribute_blobs_buffer, attribute_blobs_buffer_size, tree_registry_buffer, tree_registry_blobs_size);
    if (error.code != 0)
      return;
    error = _storage_handler.deserialize_free_blobs(free_blobs_buffer, free_blobs_buffer_size);
    if (error.code != 0)
      return;
    error = _attributes_configs.deserialize(attribute_blobs_buffer, attribute_blobs_buffer_size);
    if (error.code != 0)
      return;
    error = _tree_handler.deserialize_tree_registry(tree_registry_buffer, tree_registry_blobs_size);
    if (error.code != 0)
      return;
    // Resume (registry v2): restore the input registry (skip re-added done inputs, keep morton
    // bounds) and the done-morton watermark, so incremental LOD/finality continues where the last
    // checkpoint left off instead of restarting from zero. v1 files have no snapshot (empty).
    auto &snapshot = _tree_handler.tree_registry().input_registry_snapshot;
    if (!snapshot.empty())
    {
      error = _input_data_source_registry.deserialize(snapshot.data(), uint32_t(snapshot.size()));
      if (error.code != 0)
        return;
      _lod_done_morton = _tree_handler.tree_registry().lod_watermark;
      _current_lod_target_morton = _lod_done_morton;
    }
    // A MUTABLE dataset can take new points, and the insert path assumes every tree it descends
    // into is resident -- true during a fresh conversion, false after a reopen, where data[] is a
    // vector of null pointers filled in lazily. Pull them all in before any input arrives.
    if (_tree_handler.is_mutable())
      _tree_handler.request_all_trees();
    else
      _tree_handler.request_root();
  }

#ifndef __EMSCRIPTEN__
  if (!_destination.url.empty())
  {
    // Spill liveness/orphan GC (no-op on a fresh cache), then stand up the uploader against the
    // bucket. The dataset uuid ties the two: an existing bucket claimed by a different cache
    // generation is refused at bootstrap.
    error = _storage_handler.run_spill_bootstrap();
    if (error.code != 0)
      return;
    uint8_t uuid[16];
    _storage_handler.ensure_dataset_uuid(uuid);
    _upload_handler = std::make_unique<upload_handler_t>(_destination.url, _destination.connection, _storage_handler, _thread_pool, uuid, error);
    if (error.code != 0)
      return;
    error = _upload_handler->bootstrap();
    if (error.code != 0)
      return;
    // The bucket is authoritative for what is uploaded: seed tree states from its manifests (also
    // clears provisional band assignments the cache recorded for bands that never committed).
    _tree_handler.restore_uploaded_trees(_upload_handler->uploaded_tree_bands());
    _upload_handler->set_on_band_committed([this](uint32_t band_id, std::vector<uint32_t> tree_ids, const morton::morton192_t &watermark) {
      _tree_handler.mark_band_uploaded(band_id, std::move(tree_ids));
      if (_upload_callbacks.band_committed)
      {
        uint64_t done_morton[3] = {watermark.data[0], watermark.data[1], watermark.data[2]};
        _upload_callbacks.band_committed(_upload_callback_user_ptr, band_id, done_morton);
      }
      auto stats = _upload_handler->stats();
      if (_upload_callbacks.progress)
        _upload_callbacks.progress(_upload_callback_user_ptr, stats.bytes_uploaded, stats.bands_committed);
      if (stats.complete && _upload_callbacks.done)
        _upload_callbacks.done(_upload_callback_user_ptr);
      _event_loop.run_in_loop([] {}); // wake the processor loop so idle/wait states re-evaluate
    });
    _upload_handler->set_on_error([this](const dew_error_t &upload_error, bool parked) {
      if (_upload_callbacks.error)
        _upload_callbacks.error(_upload_callback_user_ptr, &upload_error, parked ? 1 : 0);
      _event_loop.run_in_loop([] {});
    });
    _tree_handler.set_band_sink([this](band_job_t &&job) { _upload_handler->enqueue_band(std::move(job)); }, _upload_handler->committed_band_count(), _upload_handler->stats().complete);
  }
#endif // __EMSCRIPTEN__
}

processor_t::~processor_t()
{
  // Ordered teardown. The default member-destruction order joins each event-loop thread LAST (each is
  // declared before the state its handlers touch) and drains the shared _thread_pool LAST -- but the pool's
  // parked tasks read via the storage loop and post results to the tree loop, so draining it after those
  // loops (and their pipes / registry / cache) are gone is a use-after-free / mutex-EINVAL race. Do it
  // explicitly here, while every member is still alive, honoring these invariants:
  //   (1) stop generating new pool tasks, (2) drain the pool while the storage + tree loops are still alive
  //   (parked reads complete on the storage loop and post to the tree loop), (3) only then stop those loops.
  // stop_and_join() and thread_pool::join() are idempotent, so the subsequent member destructors are no-ops.

#ifndef __EMSCRIPTEN__
  // (0) Uploader first: its band coroutines read the cache via the storage loop and park waits on
  //     the shared pool, so it must fully drain (or be parked on errors -- drained() covers both)
  //     and stop BEFORE the pool is joined or the storage loop goes away.
  if (_upload_handler)
  {
    _upload_handler->wait_drained();
    _upload_handler->stop();
  }
#endif

  // (1) No more scheduling. Detach the main-loop about-to-block hook (input file scheduling) and stop
  //     every loop that feeds the shared pool from enqueuing anything else. All three run while their
  //     loops are still alive, and each barrier returns only once its flag has been observed ON that
  //     loop -- otherwise a callback already past the check could still reach enqueue.
  //
  //     All three are required: the pool is joined in (2) but the main and input loops keep running
  //     until (4), and vio::thread_pool_t::enqueue answers an enqueue-after-stop with a bare abort()
  //     -- no message, which is what made this show up in CI as an unexplained SIGABRT.
  //     Each barrier is BOUNDED (loop_quiesce.hpp): a destructor that can wait forever on a loop is
  //     worse than the abort it prevents, so on timeout the flag is set here and we accept the
  //     narrow race instead of deadlocking.
  _event_loop.remove_about_to_block_listener(this);
  if (!core::run_on_loop_and_wait(_event_loop, [this]() { _shutting_down.store(true, std::memory_order_release); }))
    _shutting_down.store(true, std::memory_order_release);
  _point_reader.begin_shutdown();
  _tree_handler.begin_shutdown();

  // (2) Drain + join the shared pool. Its parked tree-load tasks wait_for_read() on the storage loop and
  //     post the decoded tree to the tree loop -- both are still running here.
  _thread_pool.join();

  // (3) No worker references storage/tree state now. Stop those loops before their backend / read cache /
  //     registry / event pipes are destroyed.
  _tree_handler.stop_loop();
  _storage_handler.stop_loop();

  // (4) Stop the remaining loops (input reader loop, then the main processor loop).
  _input_event_loop_thread.stop_and_join();
  _thread_with_event_loop.stop_and_join();
}

void processor_t::add_files(std::vector<std::pair<std::unique_ptr<char[]>, uint32_t>> &&input_files)
{
  {
    std::unique_lock<std::mutex> lock(_idle_mutex);
    _idle = false;
    _new_file_events_sent++;
  }
  _perf_stats.conversion_start = perf_stats_t::clock_t::now();
  _files_added.post_event(std::move(input_files));
}

void processor_t::walk_tree(frustum_tree_walker_t &walker)
{
  if (!_attribute_index_map || _cached_attribute_names != walker.m_attribute_names)
  {
    _attribute_index_map = std::make_unique<attribute_index_map_t>(_tree_handler.attributes_configs(), walker.m_attribute_names);
    _cached_attribute_names = walker.m_attribute_names;
  }
  walk_tree_direct(_tree_handler.tree_registry(), *_attribute_index_map, walker);
  _tree_handler.request_trees_async(std::move(walker.m_trees_to_load));
}

tree_config_t processor_t::tree_config()
{
  return _tree_handler.tree_config();
}

void processor_t::request_aabb(std::function<void(double[3], double[3])> callback)
{
  _tree_handler.request_aabb(callback);
}

void processor_t::wait_local_complete()
{
  std::unique_lock<std::mutex> lock(_idle_mutex);
  _idle_condition.wait(lock, [this] { return _idle; });
}

void processor_t::finalize()
{
  // 1. Land whatever is in flight BEFORE arming the seal. Flipping a tree to final while its LOD
  //    writes are still queued trips "LOD write into finalized tree" in the generator.
  wait_local_complete();

  // 2. Arm sealing, and wait for the tree loop to actually have the flag (see clear_mutable_and_wait).
  _tree_handler.clear_mutable_and_wait();

  // 3. Force the checkpoint that does the sealing, and wait for the CHAIN -- not for idle.
  //    wait_local_complete cannot serve here: _idle is already latched by step 1, so it would return
  //    immediately and steps 4-5 would run against an unsealed dataset. checkpoint_and_wait blocks on
  //    the serialize chain finishing, which is the event that actually flips the trees.
  _tree_handler.checkpoint_and_wait();

  // 4. Bands, uploads, and the destination-mode quiesce checkpoints. Now that trees are final,
  //    emit_band_job has something to ship and is no longer suppressed.
  wait_idle();
}

bool processor_t::declare_attribute(const std::string &name, const std::string &key_attribute, dew_type_t type, dew_components_t components)
{
  // Set BEFORE declare, because declare is what opens the spill file. Doing it in
  // commit_attributes -- where the pass that reads the spill lives -- is too late by the whole
  // add_data phase, and the amend silently falls back to the system temp directory. That put a
  // 3.3 GB spill on a system drive with 24 GB free while the dataset sat on a 3 TB volume.
  //
  // Beside the dataset, because that is where the caller already provisioned space for data of this
  // size. A remote url has no local directory to speak of, so those keep the temp fallback.
  {
    const auto slash = _url.find_last_of("/\\");
    const bool looks_remote = _url.find("://") != std::string::npos;
    if (!looks_remote && slash != std::string::npos)
      _attribute_amend.set_spill_directory(_url.substr(0, slash));
  }
  std::string message;
  if (!_tree_handler.is_mutable())
  {
    message = "attributes can only be added to a mutable dataset";
  }
  else if (_attribute_amend.declare(name, key_attribute, type, components, message))
  {
    return true;
  }
  dew_error_t error;
  error.code = -1;
  error.msg = message;
  handle_storage_error(std::move(error));
  return false;
}

bool processor_t::add_attribute_data(const std::string &name, const uint64_t *keys, const void *values, uint64_t count)
{
  if (_attribute_amend.add_data(name, keys, values, count))
    return true;
  dew_error_t error;
  error.code = -1;
  error.msg = fmt::format("no attribute '{}' has been declared on this converter", name);
  handle_storage_error(std::move(error));
  return false;
}

void processor_t::commit_attributes()
{
  if (_attribute_amend.empty())
    return;
  if (!_tree_handler.is_mutable())
  {
    dew_error_t error;
    error.code = -1;
    error.msg = "attributes can only be committed to a mutable dataset";
    handle_storage_error(std::move(error));
    return;
  }

  // 1. Land everything in flight. The amend snapshots each unit's storage locations and then writes
  //    against that snapshot; a conversion still placing blobs would be amending a moving dataset.
  wait_local_complete();

  // 2. Every tree has to be resident to be walked. Mutable mode loads them all at open, but an
  //    amend can follow input added in the same session, which mints trees the open never saw.
  _tree_handler.request_all_trees();

  // 3. The pass itself: read each unit's key blob, write one new blob, record it.
  auto error = _attribute_amend.commit(_tree_handler, _storage_handler, _attributes_configs);
  if (error.code != 0)
  {
    handle_storage_error(std::move(error));
    return;
  }

  // 4. Persist the storage maps that now point at those blobs. Without this the amend is in memory
  //    only: the blobs are on disk, nothing references them, and the next open reads the old maps.
  _tree_handler.checkpoint_and_wait();
  wait_idle();
}

void processor_t::wait_idle()
{
  // Full quiesce: conversion done (cache is a complete valid DEW), then every committed
  // checkpoint's band emission has been evaluated on the tree loop (the processor can observe the
  // commit before the tree loop hands the band to the uploader -- the handshake closes that gap),
  // then the upload backlog drained (or parked on persistent errors -- the cache keeps everything
  // for a later resume).
  wait_local_complete();
#ifndef __EMSCRIPTEN__
  if (_upload_handler)
  {
    _tree_handler.wait_band_emissions();
    _upload_handler->wait_drained();
    // Final quiesce sequence (skipped when nothing ever committed). First checkpoint: the terminal
    // band's facts (blobs uploaded -> pack ids, spill derefs) live only in memory until a
    // checkpoint persists the residency table; its post-commit sweep deletes dead spill segments
    // and punches/evicts uploaded bytes, settling the cache at/under its cap and the bucket free
    // of spill trash. Second checkpoint: a CLEAN snapshot of that settled state (no destructive
    // action follows it, so reopen skips the crash-safety demotion and local bytes stay locally
    // readable -- the cache remains a self-sufficient render source).
    if (_tree_handler.commits_seen() > 0)
    {
      // Barrier first: the last band's upload facts are posted to the storage loop asynchronously;
      // the checkpoint must serialize a residency table that already contains them, or their spill
      // segments stay referenced (and undeletable) in the snapshot.
      _storage_handler.drain_posted_events();
      _tree_handler.checkpoint_and_wait();
      _storage_handler.set_clean_shutdown_next_checkpoint();
      _tree_handler.checkpoint_and_wait();
    }
  }
#endif
}

bool processor_t::upload_active() const
{
#ifdef __EMSCRIPTEN__
  return false;
#else
  return _upload_handler && !_upload_handler->drained();
#endif
}

upload_stats_t processor_t::upload_stats() const
{
#ifdef __EMSCRIPTEN__
  return {};
#else
  return _upload_handler ? _upload_handler->stats() : upload_stats_t{};
#endif
}

void processor_t::about_to_block()
{
  while (_read_sort_budget - _read_sort_active_approximate_size > 0)
  {
    auto next_input = _input_data_source_registry.next_input_to_process();
    if (!next_input)
      break;
    _read_sort_active_approximate_size += next_input->approximate_point_count * next_input->approximate_point_size_bytes;
    get_points_file_t file;
    file.callbacks = _convert_callbacks;
    file.id = next_input->id;
    file.filename = next_input->name;
    _point_reader.add_file(_tree_handler.tree_config(), std::move(file));
  }
  std::unique_lock<std::mutex> lock(_idle_mutex);
  if (_input_data_source_registry.all_inserted_into_tree() && _new_file_events_sent == 0 && !_generating_lod)
  {
    _idle = true;
    _idle_condition.notify_all();
  }
}

const dew_attributes_t &processor_t::get_attributes(attributes_id_t id)
{
  return _attributes_configs.get(id);
}

void processor_t::handle_new_files(std::vector<std::pair<std::unique_ptr<char[]>, uint32_t>> &&new_files)
{
  // Teardown: the coroutine launched at the bottom of this function schedules pre-init work onto the
  // shared pool, which by now may already be joined. Bail before registering anything.
  if (_shutting_down.load(std::memory_order_acquire))
  {
    std::unique_lock<std::mutex> lock(_idle_mutex);
    _new_file_events_sent--;
    return;
  }
  // Peek, do not seal: the first batch's pre-init results may still adopt the source scale.
  auto tree_config_val = _tree_handler.tree_config_peek();
  std::vector<std::pair<input_data_id_t, input_name_ref_t>> file_refs;
  file_refs.reserve(new_files.size());
  for (auto &new_file : new_files)
  {
    bool already_done = false;
    auto input_ref = _input_data_source_registry.register_file(std::move(new_file.first), new_file.second, &already_done);
    if (already_done)
      continue; // resume: this input fully landed in a committed checkpoint of an earlier session
    file_refs.emplace_back(input_ref.input_id, input_ref.name);
  }
  {
    std::unique_lock<std::mutex> lock(_idle_mutex);
    _new_file_events_sent--;
  }

  // Launch pre-init processing as a detached coroutine using schedule_work
  [](processor_t *self, std::vector<std::pair<input_data_id_t, input_name_ref_t>> refs, tree_config_t tc) -> vio::detached_task_t
  {
    co_await self->do_handle_new_files(std::move(refs), std::move(tc));
  }(this, std::move(file_refs), std::move(tree_config_val));
}

struct pre_init_work_result_t
{
  input_data_id_t input_id;
  pre_init_info_file_result_t pre_init_result;
  file_error_t file_error;
  bool has_error = false;
};

vio::task_t<void> processor_t::do_handle_new_files(std::vector<std::pair<input_data_id_t, input_name_ref_t>> file_refs, tree_config_t tree_config_val)
{
  std::vector<std::function<std::expected<pre_init_work_result_t, vio::error_t>()>> work_items;
  work_items.reserve(file_refs.size());

  for (auto &[input_id, file_name] : file_refs)
  {
    auto *callbacks = &_convert_callbacks;
    work_items.push_back([input_id, file_name, callbacks]() -> std::expected<pre_init_work_result_t, vio::error_t>
    {
      pre_init_work_result_t result;
      result.input_id = input_id;

      dew_error_t *local_error = nullptr;
      auto pre_init_info = callbacks->pre_init(file_name.name, file_name.name_length, &local_error);
      if (local_error)
      {
        std::unique_ptr<dew_error_t> error(local_error);
        result.has_error = true;
        result.file_error.input_id = input_id;
        result.file_error.error = std::move(*error);
      }
      else
      {
        result.pre_init_result.id = input_id;
        result.pre_init_result.found_min = pre_init_info.found_aabb_min;
        memcpy(result.pre_init_result.min, pre_init_info.aabb_min, sizeof(result.pre_init_result.min));
        result.pre_init_result.found_scale = pre_init_info.found_scale != 0;
        memcpy(result.pre_init_result.scale, pre_init_info.scale, sizeof(result.pre_init_result.scale));
        result.pre_init_result.approximate_point_count = pre_init_info.approximate_point_count;
        result.pre_init_result.approximate_point_size_bytes = pre_init_info.approximate_point_size_bytes;
        result.pre_init_result.input_file_size_bytes = pre_init_info.input_file_size_bytes;
      }
      return result;
    });
  }

  // Belt and braces with the check in handle_new_files: schedule_work enqueues onto the shared pool,
  // and a pool that has been joined aborts rather than refusing.
  if (_shutting_down.load(std::memory_order_acquire))
    co_return;

  auto results = co_await vio::schedule_work(_event_loop, _thread_pool, std::move(work_items));

  // Process results on the event loop thread
  for (auto &result : results)
  {
    if (!result.has_value())
      continue;
    auto &r = result.value();
    if (r.has_error)
      continue;
    // Adopt the finest native scale reported by this batch as the octree scale, BEFORE anything below
    // consumes (and thereby seals) the tree config. Only while the configuration is still open (first
    // batch, no resume) and the user did not pin a scale explicitly. Coarser-than-source storage loses
    // precision; finer only inflates every morton code and densifies the LOD pyramid.
    if (r.pre_init_result.found_scale && !_tree_scale_explicit && !_tree_handler.configuration_initialized())
    {
      const double file_finest = std::min(r.pre_init_result.scale[0], std::min(r.pre_init_result.scale[1], r.pre_init_result.scale[2]));
      if (file_finest > 0.0 && (_source_scale_adopted <= 0.0 || file_finest < _source_scale_adopted))
      {
        _source_scale_adopted = file_finest;
        _tree_handler.set_tree_initialization_scale(file_finest);
      }
    }
  }
  std::vector<input_data_id_t> settled;
  settled.reserve(results.size());
  for (auto &result : results)
  {
    if (!result.has_value())
      continue;
    auto &r = result.value();
    if (r.has_error)
    {
      _input_data_source_registry.handle_file_failed(r.input_id);
      settled.push_back(r.input_id);
      _has_errors = true;
      if (_runtime_callbacks.error)
        _runtime_callbacks.error(_runtime_callback_user_ptr, &r.file_error.error);
    }
    else
    {
      // WARN ONCE if a file arrives with no minimum corner. It is legal -- the watermark just stays
      // conservative -- but the cost is invisible and enormous: without it every collapse and LOD
      // pass defers to a single terminal one, which on a 300M-point conversion is the difference
      // between checkpointing as it goes and running for over an hour with nothing readable on disk.
      // The field sits beside found_point_count and found_scale, which really are only hints, so
      // nothing about the struct suggests this one decides the shape of the whole conversion.
      if (!r.pre_init_result.found_min && !_warned_missing_aabb_min)
      {
        _warned_missing_aabb_min = true;
        if (_runtime_callbacks.warning)
          _runtime_callbacks.warning(_runtime_callback_user_ptr,
                                     "input reported no aabb_min from pre_init: the done-morton watermark cannot advance, so all LOD and collapse work will "
                                     "defer to a single terminal pass. See dew_converter_file_pre_init_info_t::aabb_min.");
      }
      _input_data_source_registry.register_pre_init_result(_tree_handler.tree_config(), r.pre_init_result.id, r.pre_init_result.found_min, r.pre_init_result.min,
                                                           r.pre_init_result.approximate_point_count, r.pre_init_result.approximate_point_size_bytes, r.pre_init_result.input_file_size_bytes);
      _storage_handler.register_input_file_size(r.pre_init_result.id.data, r.pre_init_result.input_file_size_bytes);
      settled.push_back(r.pre_init_result.id);
    }
  }
  // A MUTABLE dataset may already have been LOD-ed all the way to the terminal watermark, which
  // leaves maybe_start_lod's gate permanently shut. Re-open the pyramid from the lowest morton this
  // batch introduces, so the new input -- and every ancestor above it -- is regenerated, while the
  // rest of the tree is left alone. input_order is the morton of the pre-init aabb-min corner and is
  // zero when the source could not report one, which degrades to re-LOD-ing everything: correct, and
  // the only safe answer when the new points could be anywhere.
  if (_tree_handler.is_mutable())
  {
    bool have_floor = false;
    morton::morton192_t floor{};
    for (auto &result : results)
    {
      if (!result.has_value() || result.value().has_error)
        continue;
      const auto &pre = result.value().pre_init_result;
      morton::morton192_t candidate{};
      if (pre.found_min)
        convert_pos_to_morton(_tree_handler.tree_config().scale, _tree_handler.tree_config().offset, pre.min, candidate);
      if (!have_floor || candidate < floor)
      {
        floor = candidate;
        have_floor = true;
      }
    }
    if (have_floor)
    {
      _tree_handler.lower_lod_floor(floor);
      // The processor's own gate has to come down with it, or maybe_start_lod still refuses.
      if (floor < _lod_done_morton)
        _lod_done_morton = floor;
    }
  }

  // Every input registered in this batch has to SETTLE, one way or the other: an unsettled one is
  // neither dispatchable (next_input_to_process skips it) nor retired, so all_inserted_into_tree()
  // never turns true and wait_idle() blocks for good. schedule_work may answer with an unexpected,
  // which carries no input id, so sweep for whatever the results did not account for rather than
  // assuming they line up with the requests.
  for (const auto &file_ref : file_refs)
  {
    bool found = false;
    for (const auto &id : settled)
      found = found || id == file_ref.first;
    if (!found)
      _input_data_source_registry.handle_file_failed(file_ref.first);
  }
}

void processor_t::handle_input_init_done(std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t> &&event)
{
  _input_data_source_registry.handle_input_init(std::get<0>(event), std::get<1>(event), std::get<2>(event));
}

void processor_t::handle_sub_added(input_data_id_t &&event)
{
  if (std::getenv("DEW_DEBUG_CHAIN"))
    fmt::print(stderr, "[sched] sub_added file={}\n", event.data);
  _input_data_source_registry.handle_sub_added(event);
}

void processor_t::handle_sorted_points(std::pair<points_t, dew_error_t> &&event)
{
  _input_data_source_registry.handle_sorted_points(event.first.header.input_id, event.first.header.morton_min, event.first.header.morton_max);
  _storage_handler.write(
    event.first.header, event.first.attributes_id, std::move(event.first.buffers),
    [this](const storage_header_t &header, attributes_id_t attributes, std::vector<storage_location_t> locations, const dew_error_t &) { this->handle_points_written(header, attributes, std::move(locations)); });
}

void processor_t::handle_file_errors(file_error_t &&error)
{
  _has_errors = true;
  if (_runtime_callbacks.error)
    _runtime_callbacks.error(_runtime_callback_user_ptr, &error.error);
}

void processor_t::handle_file_reading_done(input_data_id_t &&file)
{
  if (std::getenv("DEW_DEBUG_CHAIN"))
    fmt::print(stderr, "[sched] reading_done file={}\n", file.data);
  _read_sort_active_approximate_size -= _input_data_source_registry.get_approximate_size(file);
  _input_data_source_registry.handle_reading_done(file);
}

void processor_t::handle_index_write_done()
{
  if (std::getenv("DEW_DEBUG_CHAIN"))
    fmt::print(stderr, "[sched] index_write_done generating={} committed={} target={}\n", _generating_lod, _tree_handler.last_committed_watermark().data[0], _current_lod_target_morton.data[0]);
  // Every committed checkpoint (pass-concluding or cache-pressure) re-arms the pressure request.
  _storage_handler.rearm_checkpoint_request();
  // A cache-pressure checkpoint can commit while a LOD pass is still generating; it carries the
  // PREVIOUS watermark, so it must not conclude the in-flight pass (that would advance
  // _lod_done_morton past unserialized state and stall/duplicate LOD scheduling).
  const bool was_generating = _generating_lod;
  if (was_generating && _tree_handler.last_committed_watermark() < _current_lod_target_morton)
    return;
  _lod_done_morton = _current_lod_target_morton;
  _generating_lod = false;
  _perf_stats.conversion_end = perf_stats_t::clock_t::now();
  maybe_start_lod();
  if (!_generating_lod && was_generating)
  {
    if (_runtime_callbacks.done)
      _runtime_callbacks.done(_runtime_callback_user_ptr);
  }
}

void processor_t::handle_storage_error(dew_error_t &&error)
{
  _has_errors = true;
  if (_runtime_callbacks.error)
    _runtime_callbacks.error(_runtime_callback_user_ptr, &error);
}

void processor_t::handle_points_written(const storage_header_t &header, attributes_id_t attributes_id, std::vector<storage_location_t> &&locations)
{
  if (input_data_id_is_leaf(header.input_id))
  {
    auto locations_copy = locations;
    _input_data_source_registry.handle_points_written(header.input_id, std::move(locations));
    storage_header_t header_copy(header);
    _tree_handler.add_points(std::move(header_copy), std::move(attributes_id), std::move(locations_copy));
  }
}

void processor_t::handle_tree_done_with_input(input_data_id_t &&event)
{
  if (std::getenv("DEW_DEBUG_CHAIN"))
    fmt::print(stderr, "[sched] tree_done file={}\n", event.data);
  _input_data_source_registry.handle_tree_done_with_input(event);
  maybe_start_lod();
}

void processor_t::maybe_start_lod()
{
  if (std::getenv("DEW_DEBUG_CHAIN"))
  {
    auto dm = _input_data_source_registry.get_done_morton();
    fmt::print(stderr, "[sched] maybe_start_lod generating={} done_morton={} lod_done={}\n", _generating_lod, dm ? dm->data[0] : 0, _lod_done_morton.data[0]);
  }
  if (_generating_lod)
    return;
  auto done_morton = _input_data_source_registry.get_done_morton();
  if (done_morton && *done_morton > _lod_done_morton)
  {
    _generating_lod = true;
    _current_lod_target_morton = *done_morton;
    _tree_handler.generate_lod(*done_morton);
  }
}

dew_error_t processor_t::upgrade_to_write(bool truncate)
{
  auto ret = _storage_handler.upgrade_to_write(truncate);
  if (truncate)
  {
    //_input_data_source_registry.~input_data_source_registry_t();
    // new (&_input_data_source_registry) input_data_source_registry_t();

    //_attributes_configs.~attributes_configs_t();
    // new (&_attributes_configs) attributes_configs_t();

    //_tree_handler.~tree_handler_t();
    // new (&_tree_handler) tree_handler_t(_thread_pool, _storage_handler, _attributes_configs, _tree_done_with_input);
  }
  return ret;
}

void processor_t::set_pre_init_tree_config(const tree_config_t &tree_config)
{
  _tree_handler.set_tree_initialization_config(tree_config);
}

void processor_t::set_tree_scale_override(double scale)
{
  _tree_scale_explicit = true;
  _tree_handler.set_tree_initialization_scale(scale);
}

void processor_t::set_pre_init_node_point_limit(uint32_t node_point_limit)
{
  _tree_handler.set_tree_initialization_node_point_limit(node_point_limit);
}

void processor_t::set_pre_init_read_chunk_bytes(uint64_t bytes)
{
  _tree_handler.set_tree_initialization_read_chunk_bytes(bytes);
}

void processor_t::set_runtime_callbacks(const dew_converter_runtime_callbacks_t &runtime_callbacks, void *user_ptr)
{
  _runtime_callbacks = runtime_callbacks;
  _runtime_callback_user_ptr = user_ptr;
  _storage_handler.set_on_write_progress([this]() {
    if (_runtime_callbacks.progress)
      _runtime_callbacks.progress(_runtime_callback_user_ptr, 0.0f);
  });
}

void processor_t::set_converter_callbacks(const dew_converter_file_convert_callbacks_t &convert_callbacks)
{
  _convert_callbacks = convert_callbacks;
}

uint32_t processor_t::attrib_name_registry_count()
{
  return _attributes_configs.attrib_name_registry_count();
}

uint32_t processor_t::attrib_name_registry_get(uint32_t index, char *name, uint32_t buffer_size)
{
  return _attributes_configs.attrib_name_registry_get(index, name, buffer_size);
}
} // namespace dew::converter
