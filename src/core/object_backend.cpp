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
#include "object_backend.hpp"

#include "bucket_format.hpp"
#include "loop_blocking.hpp"

#include <algorithm>
#include <cassert>
#include <condition_variable>
#include <cstring>

#include <fmt/format.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h> // emscripten_sleep (Asyncify)
#endif

namespace dew::core
{

using vio::objstore::io_range_t;

// Translate a vio object-store error into the storage layer's dew_error_t (code 0 == success).
static dew_error_t to_points_error(const vio::error_t &e)
{
  dew_error_t p;
  p.code = e.code != 0 ? e.code : -1;
  p.msg = e.msg;
  return p;
}

// run_on_loop_blocking lives in loop_blocking.hpp -- packed_file_backend_t needs the identical wait
// for its own synchronous read_index, and one copy is how the two stay in step.

std::string object_backend_t::object_name(uint32_t file_id, uint64_t offset)
{
  // 96 bits of identity from the storage_location; a single object per (file_id, offset) pair.
  return fmt::format("blob_{:08x}_{:016x}", file_id, offset);
}

storage_location_t object_backend_t::next_location(uint32_t size)
{
  std::unique_lock<std::mutex> lock(_mutex);
  uint64_t id = _next_id++;
  storage_location_t loc;
  loc.file_id = uint32_t(id & 0xFFFFFFFFu); // low 32 bits (== the whole id until 4B blobs, offset stays 0)
  loc.offset = id >> 32;                    // high bits carry the overflow past 4B
  loc.size = size;
  return loc;
}

vio::task_t<dew_error_t> object_backend_t::probe_exists(bool &out) const
{
  auto r = co_await _io->object_info(k_manifest_name);
  out = r.has_value() && r->exists;
  if (!r.has_value())
    _exists_error = r.error().msg; // probe FAILED (auth/region/network) -- not the same as a clean 404
  _manifest_size = out ? r->size : 0; // 128 = legacy superblock, 256 = DEW2 root manifest
  co_return dew_error_t{};
}

object_backend_t::object_backend_t(std::unique_ptr<vio::objstore::io_manager_t> io, vio::event_loop_t &event_loop)
  : _io(std::move(io))
  , _event_loop(event_loop)
{
  // No probe here on purpose -- see the _probed comment in the header. Constructing a backend must not
  // cost a network round trip, because dew_dataset_create constructs one and must return immediately.
}

vio::task_t<dew_error_t> object_backend_t::probe_exists_async()
{
  if (_probed)
    co_return dew_error_t{};
  bool exists = false;
  dew_error_t error = co_await probe_exists(exists);
  _exists = exists;
  _probed = true;
  co_return error;
}

object_backend_t::~object_backend_t()
{
}

bool object_backend_t::exists() const
{
  // The blocking half of the lazy probe, for callers that never awaited probe_exists_async(). Under
  // wasm this needs ASYNCIFY, which is why the query engine awaits the async form first and only ever
  // reaches the cached branch below.
  if (!_probed)
  {
    bool exists = false;
    (void)run_on_loop_blocking(_event_loop, [this, &exists]() { return probe_exists(exists); });
    _exists = exists;
    _probed = true;
  }
  return _exists;
}

std::string object_backend_t::exists_error() const
{
  return _exists_error;
}

dew_error_t object_backend_t::open_for_write(bool truncate)
{
  // A DEW2 bucket is read-only through this backend: its writers (upload_handler_t, dew_copy) drive
  // the io_manager directly with pack/band/manifest ordering that checkpoint_t cannot express.
  // Truncate is allowed -- the fresh legacy manifest atomically supersedes the DEW2 one (stale packs
  // become orphans, same contract as legacy truncate).
  if (_dew2 && !truncate)
    return {1, "This destination holds a DEW2 dataset; it cannot be appended to through this backend. Convert with dew_converter_create_with_destination or copy with dew_copy instead."};
  if (truncate)
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _dew2 = false;
    _next_id = 0;
    _attributes_location = {};
    _stats_location = {};
    _perf_stats_location = {};
    _tree_registry_location = {};
    // Existing objects are not enumerated/removed here (no list op yet). The new manifest, written
    // last, references only new objects; old objects become orphans reclaimed by a future gc().
  }
  // The directory (file backend) is created lazily on the first write_object; nothing to open here.
  return {};
}

vio::task_t<dew_error_t> object_backend_t::read_location(storage_location_t loc, std::unique_ptr<uint8_t[]> &buf, uint32_t &size)
{
  if (loc.size == 0)
  {
    buf.reset();
    size = 0;
    co_return dew_error_t{};
  }
  buf = std::make_unique<uint8_t[]>(loc.size);
  // Whole-object GET in both layouts (read_object_all: no Range header, capacity-checked): DEW2
  // objects are exactly one blob ("data/{file_id:08x}", size == loc.size), legacy objects likewise.
  auto r = _dew2 ? co_await _io->read_object_all(bucket_data_object_name(loc.file_id), buf.get(), loc.size)
                 : co_await _io->read_object_all(object_name(loc.file_id, loc.offset), buf.get(), loc.size);
  size = loc.size;
  if (!r.has_value())
    co_return to_points_error(r.error());
  if (r.value() != loc.size)
    co_return dew_error_t{1, "Metadata object size does not match its recorded location"};
  co_return dew_error_t{};
}

vio::task_t<dew_error_t> object_backend_t::do_read_index(index_load_t &out)
{
  // The "manifest" object name is shared by both layouts; sniff by content. Sizes differ (128-byte
  // legacy superblock vs 256-byte DEW2 root manifest), so size the read from a HEAD instead of
  // relying on stores clamping an over-long range.
  static_assert(k_root_manifest_size > k_serialized_index_size);
  auto info = co_await _io->object_info(k_manifest_name);
  if (!info.has_value())
    co_return to_points_error(info.error());
  if (!info->exists)
    co_return dew_error_t{1, "No manifest object at the dataset url"};
  _manifest_size = info->size;
  auto manifest_bytes = uint32_t(std::min<uint64_t>(info->size, k_root_manifest_size));
  auto manifest = std::make_shared<uint8_t[]>(manifest_bytes);
  auto mr = co_await _io->read_object_all(k_manifest_name, manifest.get(), manifest_bytes);
  if (!mr.has_value())
    co_return to_points_error(mr.error());
  manifest_bytes = uint32_t(std::min<uint64_t>(mr.value(), manifest_bytes));

  uint32_t magic;
  if (manifest_bytes < sizeof(magic))
    co_return dew_error_t{1, "Manifest object is too small"};
  memcpy(&magic, manifest.get(), sizeof(magic));
  if (is_root_manifest_magic(magic))
  {
    // DEW2 bucket. Read-only here: the root manifest's metadata slots are only populated once the
    // terminal band committed; before that there is no registry to read, so refuse cleanly.
    root_manifest_t root;
    auto err = deserialize_root_manifest(manifest.get(), manifest_bytes, root);
    if (err.code != 0)
      co_return err;
    if (!root.complete)
      co_return dew_error_t{1, "DEW2 dataset is incomplete: the conversion that writes it has not finished uploading. Retry when it completes (or resume the parked upload)."};
    _dew2 = true;
    out.free_blobs.reset();
    out.free_blobs_size = 0;
    err = co_await read_location(root.attribute_configs, out.attribute_configs, out.attribute_configs_size);
    if (err.code != 0)
      co_return err;
    err = co_await read_location(root.tree_registry, out.tree_registry, out.tree_registry_size);
    if (err.code != 0)
      co_return err;
    err = co_await read_location(root.compression_stats, out.stats, out.stats_size);
    if (err.code != 0)
      co_return err;
    err = co_await read_location(root.perf_stats, out.perf, out.perf_size);
    if (err.code != 0)
      co_return err;
    co_return dew_error_t{};
  }

  storage_location_t free_blobs;
  storage_location_t attribute_configs;
  storage_location_t tree_registry;
  storage_location_t compression_stats;
  storage_location_t perf_stats;
  if (manifest_bytes < k_serialized_index_size)
    co_return dew_error_t{1, "Manifest object is too small"};
  auto err = deserialize_index(manifest.get(), k_serialized_index_size, free_blobs, attribute_configs, tree_registry, compression_stats, perf_stats);
  if (err.code != 0)
    co_return err;

  _next_id = free_blobs.offset; // full 64-bit next id (see next_location / object_name)
  _attributes_location = attribute_configs;
  _stats_location = compression_stats;
  _perf_stats_location = perf_stats;
  _tree_registry_location = tree_registry;

  // free_blobs carries only next_id in object mode; there is no free-blobs object.
  out.free_blobs.reset();
  out.free_blobs_size = 0;

  err = co_await read_location(attribute_configs, out.attribute_configs, out.attribute_configs_size);
  if (err.code != 0)
    co_return err;
  err = co_await read_location(tree_registry, out.tree_registry, out.tree_registry_size);
  if (err.code != 0)
    co_return err;
  err = co_await read_location(compression_stats, out.stats, out.stats_size);
  if (err.code != 0)
    co_return err;
  err = co_await read_location(perf_stats, out.perf, out.perf_size);
  if (err.code != 0)
    co_return err;

  co_return dew_error_t{};
}

dew_error_t object_backend_t::read_index(index_load_t &out)
{
  return run_on_loop_blocking(_event_loop, [this, &out]() { return do_read_index(out); });
}

vio::task_t<dew_error_t> object_backend_t::read_index_async(index_load_t &out)
{
  // The same coroutine the blocking form drives; here it is simply awaited instead.
  co_return co_await do_read_index(out);
}

dew_error_t object_backend_t::restore_allocator(const std::unique_ptr<uint8_t[]> &data, uint32_t size)
{
  (void)data;
  (void)size;
  return {}; // object mode has no packed free-list; the id counter comes from the manifest.
}

void object_backend_t::allocate_blob(uint32_t size, blob_kind_t kind, storage_location_t &out)
{
  (void)kind; // object mode is remote already; kind only matters for the local cached backend
  assert(!_dew2 && "DEW2 buckets are read-only through object_backend_t (open_for_write refuses)");
  out = next_location(size);
}

vio::task_t<dew_error_t> object_backend_t::write_allocated(storage_location_t location, std::shared_ptr<uint8_t[]> data)
{
  auto r = co_await _io->write_object(object_name(location.file_id, location.offset), std::move(data), location.size);
  if (!r.has_value())
    co_return to_points_error(r.error());
  co_return dew_error_t{};
}

vio::task_t<dew_error_t> object_backend_t::read_blob(storage_location_t location, uint8_t *dst, uint32_t &bytes_read)
{
  // Whole-object GET in both layouts (read_object_all: no Range header, capacity-checked so a
  // mismatched object can never overrun the buffer). DEW2: the blob IS object "data/{file_id:08x}"
  // (offset always 0). Legacy: the blob IS the object (there `offset` is the high half of the
  // blob-id counter, never a byte offset -- the interpretations must never mix).
  assert(!_dew2 || location.offset == 0);
  auto r = _dew2 ? co_await _io->read_object_all(bucket_data_object_name(location.file_id), dst, location.size)
                 : co_await _io->read_object_all(object_name(location.file_id, location.offset), dst, location.size);
  if (!r.has_value())
    co_return to_points_error(r.error());
  // A SHORT read is a failure, not a partial success. The object is supposed to BE the blob, so
  // fewer bytes means a truncated object -- and a caller that trusts location.size then hands the
  // unwritten tail of its buffer onward: `dew copy` wrote it straight into the destination dataset
  // (uninitialized heap published into an object store) and fed it to tree_deserialize. Every
  // sibling read already rejects this (read_location just above, packed_file_backend's remote path,
  // co_read_into_buffer); read_blob was the one that did not.
  if (r.value() != location.size)
    co_return dew_error_t{1, "Data object is shorter than its recorded location"};
  bytes_read = uint32_t(r.value());
  co_return dew_error_t{};
}

vio::task_t<dew_error_t> object_backend_t::write_index(checkpoint_t checkpoint)
{
  storage_location_t attributes_location = next_location(checkpoint.attribute_configs_size);
  {
    auto w = co_await _io->write_object(object_name(attributes_location.file_id, attributes_location.offset), checkpoint.attribute_configs, attributes_location.size);
    if (!w.has_value())
      co_return to_points_error(w.error());
  }

  storage_location_t stats_location = next_location(checkpoint.stats_size);
  {
    auto w = co_await _io->write_object(object_name(stats_location.file_id, stats_location.offset), checkpoint.stats, stats_location.size);
    if (!w.has_value())
      co_return to_points_error(w.error());
  }

  storage_location_t perf_location = next_location(checkpoint.perf_size);
  {
    auto w = co_await _io->write_object(object_name(perf_location.file_id, perf_location.offset), checkpoint.perf, perf_location.size);
    if (!w.has_value())
      co_return to_points_error(w.error());
  }

  // The manifest's free-blobs slot carries the full 64-bit next id so allocation resumes on reopen.
  uint64_t next_id_snapshot;
  {
    std::unique_lock<std::mutex> lock(_mutex);
    next_id_snapshot = _next_id;
  }
  storage_location_t free_blobs_slot{0, 0, next_id_snapshot};
  auto index = serialize_index(k_serialized_index_size, free_blobs_slot, attributes_location, checkpoint.tree_registry, stats_location, perf_location);

  // Write the manifest LAST (atomic replace) — the crash-safety commit point.
  {
    auto w = co_await _io->write_object(k_manifest_name, index, k_serialized_index_size);
    if (!w.has_value())
      co_return to_points_error(w.error());
  }

  // Commit succeeded: record new metadata locations and only NOW reclaim superseded objects.
  storage_location_t old_attributes = _attributes_location;
  storage_location_t old_stats = _stats_location;
  storage_location_t old_perf = _perf_stats_location;
  storage_location_t old_tree_registry = _tree_registry_location;
  _attributes_location = attributes_location;
  _stats_location = stats_location;
  _perf_stats_location = perf_location;
  _tree_registry_location = checkpoint.tree_registry;

  // Best-effort reclamation (a failed remove only leaks an orphan object; the dataset stays correct).
  for (auto &loc : checkpoint.freed)
    (void)co_await _io->remove_object(object_name(loc.file_id, loc.offset));
  if (old_attributes.size > 0)
    (void)co_await _io->remove_object(object_name(old_attributes.file_id, old_attributes.offset));
  if (old_stats.size > 0)
    (void)co_await _io->remove_object(object_name(old_stats.file_id, old_stats.offset));
  if (old_perf.size > 0)
    (void)co_await _io->remove_object(object_name(old_perf.file_id, old_perf.offset));
  // Reclaim the previous tree-registry object (previously leaked one orphan object per checkpoint).
  if (old_tree_registry.size > 0)
    (void)co_await _io->remove_object(object_name(old_tree_registry.file_id, old_tree_registry.offset));

  co_return dew_error_t{};
}

} // namespace dew::core
