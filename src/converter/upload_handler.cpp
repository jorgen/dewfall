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
#include "upload_handler.hpp"

#include "loop_quiesce.hpp"

#include "memory_writer.hpp"

#include <vio/objstore/create_object_store.h>
#include <vio/operation/sleep.h>

#include <fmt/format.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <future>

namespace dew::converter
{
using namespace dew::core;

static dew_error_t from_vio_error(const vio::error_t &e)
{
  dew_error_t r;
  r.code = e.code;
  r.msg = e.msg;
  return r;
}

namespace
{
// Await a blocking cache read without stalling the uploader loop: the wait parks on the shared
// thread pool; completion hops back to the uploader loop to resume the coroutine.
struct pool_read_wait_t
{
  std::shared_ptr<read_request_t> request;
  vio::thread_pool_t &pool;
  vio::event_loop_t &loop;
  bool await_ready() const noexcept
  {
    return false;
  }
  void await_suspend(std::coroutine_handle<> handle) noexcept
  {
    pool.enqueue_detached([request = request, &loop = loop, handle]() {
      request->wait_for_read();
      loop.run_in_loop([handle]() { handle.resume(); });
    });
  }
  void await_resume() const noexcept
  {
  }
};
} // namespace

upload_handler_t::upload_handler_t(std::unique_ptr<vio::objstore::io_manager_t> io, storage_handler_t &storage, vio::thread_pool_t &pool, const uint8_t (&dataset_uuid)[16])
  : _loop_thread()
  , _loop(_loop_thread.event_loop())
  , _io(std::move(io))
  , _storage(storage)
  , _pool(pool)
  , _band_pipe(_loop, [this](band_job_t &&job) { handle_band(std::move(job)); })
{
  memcpy(_uuid, dataset_uuid, sizeof(_uuid));
}

upload_handler_t::upload_handler_t(const std::string &destination_url, const std::string &connection, storage_handler_t &storage, vio::thread_pool_t &pool, const uint8_t (&dataset_uuid)[16], dew_error_t &error)
  : _loop_thread()
  , _loop(_loop_thread.event_loop())
  , _storage(storage)
  , _pool(pool)
  , _band_pipe(_loop, [this](band_job_t &&job) { handle_band(std::move(job)); })
{
  memcpy(_uuid, dataset_uuid, sizeof(_uuid));
  auto io = vio::objstore::create_io_manager(destination_url, std::string_view(connection), _loop);
  if (!io.has_value())
  {
    error = {io.error().code != 0 ? io.error().code : -1, io.error().msg};
    return;
  }
  _io = std::move(io.value());
}

upload_handler_t::~upload_handler_t()
{
  stop();
}

void upload_handler_t::stop()
{
  // The io manager owns this loop's TCP handles, and its destructor uv_close()es them. Left to
  // member order it is destroyed AFTER the loop thread has joined, so those closes sit queued on a
  // loop nobody will ever run again: uv_loop_close then answers EBUSY and vio's event_loop_t
  // destructor asserts -- which is what made every destination-mode run abort on teardown in a debug
  // build, long after the upload itself had succeeded.
  //
  // So it goes first, ON the loop, while the loop is still alive to reap them. stop_and_join's
  // exit_event_loop_cb runs a nested uv_run that then finishes the close callbacks. Callers drain
  // (wait_drained) before stopping and a parked band has already returned, so no coroutine is
  // suspended inside an _io operation here.
  if (_io)
  {
    if (!core::run_on_loop_and_wait(_loop, [this]() { _io.reset(); }))
      _io.reset(); // bounded, like every teardown barrier: a narrow race beats a hang in a destructor
  }
  _loop_thread.stop_and_join();
}

dew_error_t upload_handler_t::bootstrap()
{
  // Synchronous bootstrap on the uploader loop (called before any band exists).
  std::promise<dew_error_t> done;
  auto fut = done.get_future();
  _loop.run_in_loop([this, &done]() -> vio::task_t<void> {
    return [](upload_handler_t *self, std::promise<dew_error_t> &promise) -> vio::task_t<void> {
      dew_error_t error = {};
      root_manifest_t root;
      std::vector<uint8_t> buffer(k_root_manifest_size);
      auto info = co_await self->_io->object_info(bucket_root_manifest_name());
      if (!info.has_value())
      {
        promise.set_value(from_vio_error(info.error()));
        co_return;
      }
      if (!info.value().exists)
      {
        // Fresh bucket: claim it with an empty root manifest carrying our uuid.
        memcpy(root.dataset_uuid, self->_uuid, sizeof(root.dataset_uuid));
        auto data = serialize_root_manifest(root);
        auto put = co_await self->_io->write_object(bucket_root_manifest_name(), std::move(data), k_root_manifest_size);
        promise.set_value(put.has_value() ? dew_error_t{} : from_vio_error(put.error()));
        co_return;
      }
      auto read = co_await self->_io->read_object(bucket_root_manifest_name(), buffer.data(), {});
      if (!read.has_value())
      {
        promise.set_value(from_vio_error(read.error()));
        co_return;
      }
      error = deserialize_root_manifest(buffer.data(), uint32_t(read.value()), root);
      if (error.code != 0)
      {
        promise.set_value(error);
        co_return;
      }
      if (memcmp(root.dataset_uuid, self->_uuid, sizeof(root.dataset_uuid)) != 0)
      {
        promise.set_value(dew_error_t{1, "Destination bucket belongs to a different dataset generation (uuid mismatch)"});
        co_return;
      }
      self->_band_count = root.band_count;
      self->_bootstrap_band_count = root.band_count;
      self->_next_object_id = root.next_object_id;
      // Committed bands: rebuild the dedup map + uploaded tree table (names are deterministic; no
      // list op needed).
      for (uint32_t band = 0; band < root.band_count; band++)
      {
        auto band_info = co_await self->_io->object_info(bucket_band_name(band));
        if (!band_info.has_value() || !band_info.value().exists)
        {
          promise.set_value(dew_error_t{1, "Committed band manifest missing from destination"});
          co_return;
        }
        std::vector<uint8_t> band_buffer(band_info.value().size);
        auto band_read = co_await self->_io->read_object(bucket_band_name(band), band_buffer.data(), {});
        if (!band_read.has_value())
        {
          promise.set_value(from_vio_error(band_read.error()));
          co_return;
        }
        band_manifest_t manifest;
        error = deserialize_band_manifest(band_buffer.data(), uint32_t(band_read.value()), manifest);
        if (error.code != 0)
        {
          promise.set_value(error);
          co_return;
        }
        for (auto &blob : manifest.blobs)
          self->_dedup[blob.cache_offset] = blob.location;
        for (auto &tree : manifest.trees)
        {
          self->_tree_locations[tree.tree_id] = tree.location;
          self->_bootstrap_tree_bands.emplace_back(tree.tree_id, manifest.band_id);
        }
      }
      {
        std::unique_lock<std::mutex> lock(self->_stats_mutex);
        self->_stats.bands_committed = self->_band_count;
        self->_stats.complete = root.complete != 0;
      }
      promise.set_value(dew_error_t{});
      co_return;
    }(this, done);
  });
  return fut.get();
}

void upload_handler_t::enqueue_band(band_job_t &&job)
{
  {
    std::unique_lock<std::mutex> lock(_stats_mutex);
    _enqueued_bands++;
  }
  _band_pipe.post_event(std::move(job));
}

void upload_handler_t::handle_band(band_job_t &&job)
{
  _queued.push_back(std::move(job));
  if (_band_in_flight || _parked)
    return;
  _band_in_flight = true;
  auto next = std::move(_queued.front());
  _queued.erase(_queued.begin());
  [](upload_handler_t *self, band_job_t band) -> vio::detached_task_t {
    co_await self->process_band(std::move(band));
    self->_band_in_flight = false;
    if (!self->_parked && !self->_queued.empty())
    {
      auto following = std::move(self->_queued.front());
      self->_queued.erase(self->_queued.begin());
      self->handle_band(std::move(following)); // re-enters the kick path with in_flight false
    }
  }(this, std::move(next));
}

vio::task_t<dew_error_t> upload_handler_t::read_cache_blob(storage_location_t location, std::vector<uint8_t> &out)
{
  auto request = _storage.read(location, /*raw=*/true);
  co_await pool_read_wait_t{request, _pool, _loop};
  if (request->error.code != 0)
    co_return dew_error_t{request->error};
  out.assign(static_cast<const uint8_t *>(request->buffer_info.data), static_cast<const uint8_t *>(request->buffer_info.data) + request->buffer_info.size);
  co_return dew_error_t{};
}

vio::task_t<dew_error_t> upload_handler_t::put_with_retry(std::string name, std::shared_ptr<uint8_t[]> data, uint64_t size)
{
  dew_error_t last = {};
  for (int attempt = 0; attempt < 5; attempt++)
  {
    if (attempt)
      (void)co_await vio::sleep(_loop, std::chrono::milliseconds(250u << attempt)); // 500ms..4s
    auto r = co_await _io->write_object(name, data, size);
    if (r.has_value())
    {
      std::unique_lock<std::mutex> lock(_stats_mutex);
      _stats.bytes_uploaded += size;
      co_return dew_error_t{};
    }
    last = from_vio_error(r.error());
  }
  co_return last;
}

vio::detached_task_t upload_handler_t::put_data_object_windowed(put_window_t *window, uint32_t object_id, std::shared_ptr<uint8_t[]> data, uint64_t size)
{
  auto err = co_await put_with_retry(bucket_data_object_name(object_id), std::move(data), size);
  if (err.code == 0)
  {
    std::unique_lock<std::mutex> lock(_stats_mutex);
    _stats.objects_written++;
  }
  else if (window->first_error.code == 0)
  {
    window->first_error = err;
  }
  window->in_flight--;
  if (window->waiter)
  {
    auto handle = window->waiter;
    window->waiter = {};
    handle.resume(); // the waiter re-checks room in its wait loop
  }
}

vio::task_t<void> upload_handler_t::process_band(band_job_t job)
{
  auto park = [this](const dew_error_t &error) {
    _parked = true;
    {
      std::unique_lock<std::mutex> lock(_stats_mutex);
      _stats.parked = true;
    }
    _drained_cv.notify_all();
    if (_on_error)
      _on_error(error, true);
  };

  // Data-object PUTs run through a bounded window (small ~0.2-2MB objects; sequential PUTs would
  // pay a full round trip each). The window lives in this frame: EVERY exit below first drains it.
  constexpr int k_max_in_flight_puts = 8;
  put_window_t window;
  auto launch_put = [&](uint32_t object_id, const uint8_t *bytes, uint32_t size) {
    auto data = std::make_shared<uint8_t[]>(size);
    memcpy(data.get(), bytes, size);
    window.in_flight++;
    put_data_object_windowed(&window, object_id, std::move(data), size);
  };

  assert(job.band_id == _band_count && "bands must commit strictly in order");
  band_manifest_t manifest;
  manifest.band_id = job.band_id;
  memcpy(manifest.dataset_uuid, _uuid, sizeof(manifest.dataset_uuid));
  manifest.watermark = job.watermark;
  manifest.first_object_id = _next_object_id;
  manifest.attributes_configs_snapshot = std::move(job.attributes_snapshot);

  // Deterministic object-id assignment: trees ascending by id (the job is emitted that way, but
  // enforce), per tree the storage-map units sorted by input id, blobs in location order -- so a
  // crashed band retries into the SAME object names/contents and overwrites its own orphans.
  std::sort(job.trees.begin(), job.trees.end(), [](const auto &a, const auto &b) { return a.first < b.first; });

  struct unit_t
  {
    input_data_id_t id;
    attributes_id_t attributes_id;
    std::vector<storage_location_t> locations;
  };
  std::vector<uint8_t> blob_bytes;
  std::vector<std::pair<uint32_t, serialized_tree_t>> remapped_trees;

  for (auto &[tree_id, tree_location] : job.trees)
  {
    // 1. Load + deserialize the (immutable) tree snapshot from the cache.
    std::vector<uint8_t> tree_bytes;
    auto err = co_await read_cache_blob(tree_location, tree_bytes);
    if (err.code != 0)
    {
      while (window.in_flight >= 1)
        co_await window.wait_for_room(1);
      park(err);
      co_return;
    }
    serialized_tree_t serialized;
    serialized.size = int(tree_bytes.size());
    serialized.data = std::make_shared<uint8_t[]>(tree_bytes.size());
    memcpy(serialized.data.get(), tree_bytes.data(), tree_bytes.size());
    auto tree = std::make_unique<tree_t>();
    dew_error_t tree_error = {};
    if (!tree_deserialize(serialized, *tree, tree_error))
    {
      while (window.in_flight >= 1)
        co_await window.wait_for_room(1);
      park(tree_error);
      co_return;
    }

    // 2. Walk the storage map deterministically; upload every unit not already in the bucket.
    std::vector<unit_t> units;
    tree->storage_map.for_each([&](input_data_id_t id, attributes_id_t attributes_id, const std::vector<storage_location_t> &locations) { units.push_back({id, attributes_id, locations}); });
    std::sort(units.begin(), units.end(), [](const unit_t &a, const unit_t &b) { return a.id.data != b.id.data ? a.id.data < b.id.data : a.id.sub < b.id.sub; });

    for (auto &unit : units)
    {
      for (auto &location : unit.locations)
      {
        if (location.size == 0)
          continue; // empty attribute slot
        if (_dedup.contains(location.offset))
          continue; // shared chunk straddling a subtree boundary, uploaded by an earlier band/tree
        auto read_error = co_await read_cache_blob(location, blob_bytes);
        if (read_error.code != 0)
        {
          while (window.in_flight >= 1)
            co_await window.wait_for_room(1);
          park(read_error);
          co_return;
        }
        while (window.in_flight >= k_max_in_flight_puts)
          co_await window.wait_for_room(k_max_in_flight_puts);
        if (window.first_error.code != 0)
          break; // drain + park below
        storage_location_t bucket_location;
        bucket_location.file_id = _next_object_id++;
        bucket_location.offset = 0;
        bucket_location.size = uint32_t(blob_bytes.size());
        launch_put(bucket_location.file_id, blob_bytes.data(), bucket_location.size);
        // The dedup/manifest entries are recorded at LAUNCH: ids are deterministic, and nothing is
        // committed unless every put succeeded (the drain below gates the band manifest).
        _dedup[location.offset] = bucket_location;
        manifest.blobs.push_back({location.offset, bucket_location});
      }
      if (window.first_error.code != 0)
        break;
    }
    if (window.first_error.code != 0)
      break;

    // 3. Remap the tree's storage map to bucket object locations and re-serialize the tree.
    tree->storage_map.remap_storage([&](std::vector<storage_location_t> &locations) {
      for (auto &location : locations)
        if (location.size != 0)
          location = _dedup.at(location.offset);
    });
    auto reserialized = tree_serialize(*tree);
    if (!reserialized.data)
    {
      park(dew_error_t{1, "Failed to serialize tree for upload"});
      co_return;
    }
    remapped_trees.emplace_back(tree_id, std::move(reserialized));
  }

  for (auto &[tree_id, serialized] : remapped_trees)
  {
    if (window.first_error.code != 0)
      break;
    while (window.in_flight >= k_max_in_flight_puts)
      co_await window.wait_for_room(k_max_in_flight_puts);
    storage_location_t location;
    location.file_id = _next_object_id++;
    location.offset = 0;
    location.size = uint32_t(serialized.size);
    launch_put(location.file_id, serialized.data.get(), location.size);
    _tree_locations[tree_id] = location;
    manifest.trees.push_back({tree_id, location});
  }

  // Terminal band: upload the registry (locations remapped to bucket objects, states marked uploaded)
  // + the attributes snapshot, and point the root manifest at them.
  storage_location_t registry_location = {};
  storage_location_t attributes_location = {};
  if (job.terminal && window.first_error.code == 0)
  {
    tree_registry_t registry;
    auto buffer = std::make_unique<uint8_t[]>(job.registry_snapshot.size);
    memcpy(buffer.get(), job.registry_snapshot.data.get(), job.registry_snapshot.size);
    auto err = tree_registry_deserialize(buffer, uint32_t(job.registry_snapshot.size), registry);
    if (err.code != 0)
    {
      while (window.in_flight >= 1)
        co_await window.wait_for_room(1);
      park(err);
      co_return;
    }
    for (uint32_t i = 0; i < uint32_t(registry.locations.size()); i++)
    {
      auto it = _tree_locations.find(i);
      if (it == _tree_locations.end())
      {
        while (window.in_flight >= 1)
          co_await window.wait_for_room(1);
        park(dew_error_t{1, fmt::format("Terminal band missing bucket location for tree {}", i)});
        co_return;
      }
      registry.locations[i] = it->second;
      registry.tree_state[i] = uint8_t(tree_state_t::uploaded);
      registry.tree_band[i] = registry.tree_band[i] == tree_band_none ? job.band_id : registry.tree_band[i];
    }
    auto reserialized_registry = tree_registry_serialize(registry);
    if (!reserialized_registry.data)
    {
      while (window.in_flight >= 1)
        co_await window.wait_for_room(1);
      park(dew_error_t{1, "Failed to serialize registry for upload"});
      co_return;
    }
    registry_location.file_id = _next_object_id++;
    registry_location.offset = 0;
    registry_location.size = uint32_t(reserialized_registry.size);
    launch_put(registry_location.file_id, reserialized_registry.data.get(), registry_location.size);
    attributes_location.file_id = _next_object_id++;
    attributes_location.offset = 0;
    attributes_location.size = uint32_t(manifest.attributes_configs_snapshot.size());
    launch_put(attributes_location.file_id, manifest.attributes_configs_snapshot.data(), attributes_location.size);
  }

  manifest.next_object_id = _next_object_id;

  // Drain the window: every data object must be durable before the band manifest names them.
  while (window.in_flight >= 1)
    co_await window.wait_for_room(1);
  if (window.first_error.code != 0)
  {
    park(window.first_error);
    co_return;
  }

  // Band manifest (immutable) AFTER all its data objects...
  auto band_bytes = serialize_band_manifest(manifest);
  if (band_bytes.empty())
  {
    park(dew_error_t{1, "Failed to serialize band manifest"});
    co_return;
  }
  auto band_data = std::make_shared<uint8_t[]>(band_bytes.size());
  memcpy(band_data.get(), band_bytes.data(), band_bytes.size());
  auto band_error = co_await put_with_retry(bucket_band_name(job.band_id), std::move(band_data), band_bytes.size());
  if (band_error.code != 0)
  {
    park(band_error);
    co_return;
  }

  // ...then the root manifest (the commit point).
  root_manifest_t root;
  memcpy(root.dataset_uuid, _uuid, sizeof(root.dataset_uuid));
  root.band_count = job.band_id + 1;
  root.next_object_id = _next_object_id;
  root.complete = job.terminal ? 1 : 0;
  root.tree_registry = registry_location;
  root.attribute_configs = attributes_location;
  auto root_data = serialize_root_manifest(root);
  auto root_error = co_await put_with_retry(bucket_root_manifest_name(), std::move(root_data), k_root_manifest_size);
  if (root_error.code != 0)
  {
    park(root_error);
    co_return;
  }

  _band_count = root.band_count;

  // Publish the band's side effects BEFORE signalling drained: both are posts to other loops
  // (storage loop / tree loop), so a wait_drained waiter that wakes and then runs a FIFO barrier
  // (drain_posted_events) or posts a checkpoint is guaranteed to observe them. Signalling first
  // let the final quiesce checkpoint serialize a residency table missing the LAST band's upload
  // facts -- its spill segments then stayed referenced (and undeletable) forever.
  //
  // The cache tier may now treat this band's blobs as uploaded (evictable once durable). The
  // remote id is the bucket object, so evicted blobs remain readable through whole-object GETs.
  std::vector<std::pair<uint64_t, storage_location_t>> uploaded;
  uploaded.reserve(manifest.blobs.size());
  for (auto &blob : manifest.blobs)
    uploaded.emplace_back(blob.cache_offset, blob.location);
  _storage.note_blobs_uploaded(std::move(uploaded));

  if (_on_band_committed)
  {
    std::vector<uint32_t> tree_ids;
    tree_ids.reserve(manifest.trees.size());
    for (auto &tree : manifest.trees)
      tree_ids.push_back(tree.tree_id);
    _on_band_committed(job.band_id, std::move(tree_ids), job.watermark);
  }

  {
    std::unique_lock<std::mutex> lock(_stats_mutex);
    _stats.bands_committed = _band_count;
    _stats.complete = root.complete != 0;
  }
  _drained_cv.notify_all();
  co_return;
}

bool upload_handler_t::drained() const
{
  std::unique_lock<std::mutex> lock(_stats_mutex);
  return _stats.parked || _stats.bands_committed >= _enqueued_bands + _bootstrap_band_count;
}

void upload_handler_t::wait_drained()
{

  std::unique_lock<std::mutex> lock(_stats_mutex);
  _drained_cv.wait(lock, [&] { return _stats.parked || _stats.bands_committed >= _enqueued_bands + _bootstrap_band_count; });
}

} // namespace dew::converter
