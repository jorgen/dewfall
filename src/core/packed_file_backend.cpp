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
#include "packed_file_backend.hpp"

#include "loop_blocking.hpp"

#include "bucket_format.hpp"
#include "file_hole_punch.hpp"

#include <uv.h>

#include <cassert>
#include <cstring>
#include <fcntl.h>

namespace dew::core
{

// Read one blob into a fresh buffer, off the loop.
//
// The blocking uv_fs_read this replaces was fine when the only caller was the converter opening from
// its own thread, but these blobs are not small -- a tree registry runs to megabytes -- and the query
// engine reads them from a loop that must stay responsive. co_await hands the wait back to the loop
// instead of owning the thread for the duration.
static vio::task_t<dew_error_t> co_read_into_buffer(vio::event_loop_t &event_loop, vio::file_t &file, storage_location_t location, std::unique_ptr<uint8_t[]> &out)
{
  out.reset();
  if (location.size == 0)
    co_return dew_error_t{1, "cannot read a zero-sized blob"};
  auto buffer = std::make_unique<uint8_t[]>(location.size);
  auto result = co_await vio::read_file(event_loop, file, buffer.get(), location.size, int64_t(location.offset));
  if (!result.has_value())
    co_return dew_error_t{result.error().code != 0 ? result.error().code : 1, result.error().msg};
  if (result.value() != location.size)
    co_return dew_error_t{1, "Could not read the entire buffer"};
  out = std::move(buffer);
  co_return dew_error_t{};
}

packed_file_backend_t::packed_file_backend_t(std::string file_name, vio::event_loop_t &event_loop, dew_error_t &error)
  : _file_name(std::move(file_name))
  , _event_loop(event_loop)
{
  auto stat_result = vio::stat_file(_event_loop, _file_name);
  if (!stat_result.has_value())
  {
    auto index = _blob_manager.register_blob({_serialized_index_size});
    assert(index.data == 0);
    (void)index;
    return;
  }
  _file_exists = true;
  auto open_result = vio::open_file(_event_loop, _file_name, vio::file_open_flags_t(vio::file_open_flag_t::rdonly), 0);
  if (!open_result.has_value())
  {
    error.code = open_result.error().code;
    error.msg = open_result.error().msg;
    return;
  }
  _file = std::move(open_result.value());
}

packed_file_backend_t::~packed_file_backend_t()
{
  _file.reset();
}

void packed_file_backend_t::enable_cache_tier(uint64_t cap_bytes)
{
  if (!_residency)
    _residency = std::make_unique<blob_residency_t>();
  _residency->set_cap(cap_bytes);
}

void packed_file_backend_t::enable_spill(std::unique_ptr<vio::objstore::io_manager_t> io, std::string prefix, uint32_t segment_target_bytes)
{
  assert(_residency && "spill requires the cache tier");
  _spill_io = std::move(io);
  _spill = std::make_unique<spill_store_t>(*_spill_io, std::move(prefix), segment_target_bytes);
}

uint64_t packed_file_backend_t::run_eviction_pass()
{
  if (!_residency || !_residency->cap() || _punch_supported == 0 || !_file)
    return 0;
  // Reclaim down to the low watermark (cap - 2*cap/8) so a single pass buys real headroom instead
  // of oscillating at the soft threshold.
  const uint64_t low = _residency->cap() - 2 * (_residency->cap() / 8);
  uint64_t reclaimed = 0;
  while (_residency->resident_bytes() > low)
  {
    auto *victim = _residency->pick_evict_victim();
    if (!victim)
      break;
    // Punch FIRST, then flip the state: both run on the storage loop, so no read can dispatch
    // in between; a failed/unsupported punch leaves the victim fully intact and local.
    auto punch = file_hole_punch((**_file).handle, victim->offset, victim->size);
    if (punch.status == hole_punch_status_t::unsupported)
    {
      _punch_supported = 0; // degrade: eviction reclaims nothing here; the cap falls back to spill
      break;
    }
    if (punch.status == hole_punch_status_t::error)
      break;
    _punch_supported = 1;
    const uint32_t size = victim->size;
    const uint64_t offset = victim->offset;
    if (!_residency->evict(offset))
      break; // races are impossible on the loop; treat as a stop condition regardless
    reclaimed += size;
  }
  return reclaimed;
}

vio::task_t<dew_error_t> packed_file_backend_t::run_spill_pass()
{
  if (!_residency || !_spill || !_file)
    co_return dew_error_t{};
  const uint64_t cap = _residency->cap();
  if (!cap)
    co_return dew_error_t{};
  const uint64_t low = cap - 2 * (cap / 8);
  // Descending offset ≈ highest morton ≈ produced most recently ≈ consumed latest by the LOD
  // generator (inputs stream in rising-morton order) -- spilling these avoids a refetch storm from
  // the very next LOD pass. The bytes remain local (and readable) until the spill fact turns
  // durable; the reclaim happens in the post-checkpoint punch of local_spilled entries.
  uint64_t pending_reclaim = 0;
  std::vector<uint8_t> buffer;
  for (size_t i = _data_blobs.size(); i-- > 0 && _residency->resident_bytes() - pending_reclaim > low;)
  {
    const auto [offset, size] = _data_blobs[i];
    if (_residency->find(offset))
      continue; // already uploaded/spilled/diverted
    {
      std::unique_lock<std::mutex> lock(_mutex);
      if (_write_pending.contains(offset))
        continue; // allocated, bytes not on disk yet -- spilling now would pack garbage
    }
    buffer.resize(size);
    auto read_result = co_await vio::read_file(_event_loop, **_file, buffer.data(), size, int64_t(offset));
    if (!read_result.has_value() || uint32_t(read_result.value()) != size)
      co_return dew_error_t{1, "Failed to read blob for spill"};
    uint64_t remote_id = 0;
    auto err = co_await _spill->spill_blob(buffer.data(), size, remote_id);
    if (err.code != 0)
      co_return err;
    if (_residency->find(offset))
    {
      // An upload note landed while the read/PUT above was suspended: the blob is uploaded now.
      // Recording the spill would clobber the pack locator and strand the blob (and its segment)
      // as spilled forever. Drop the reservation; the spilled bytes become dead segment tail.
      _spill->deref(remote_id);
      continue;
    }
    _residency->mark_spilled(offset, size, remote_id);
    pending_reclaim += size;
  }
  co_return dew_error_t{};
}

vio::task_t<dew_error_t> packed_file_backend_t::spill_bootstrap()
{
  if (!_residency || !_spill)
    co_return dew_error_t{};
  _residency->for_each([&](const blob_residency_entry_t &e) {
    if (e.state == blob_residency_state_t::local_spilled || e.state == blob_residency_state_t::remote_spilled)
      _spill->add_live(e.remote_id);
  });
  co_return co_await _spill->gc_orphans();
}

bool packed_file_backend_t::wants_checkpoint() const
{
  if (!_residency || !_residency->cap())
    return false;
  if (!_residency->over_soft())
    return false;
  // Over pressure with nothing durable to evict: only a checkpoint can convert pending remote
  // facts into evictable ones. (const_cast-free: pick is logically const, our copy isn't -- keep a
  // local scan instead.)
  auto *self = const_cast<packed_file_backend_t *>(this);
  return self->_residency->pick_evict_victim() == nullptr;
}

bool packed_file_backend_t::exists() const
{
  return _file_exists;
}

dew_error_t packed_file_backend_t::open_for_write(bool truncate)
{
  vio::file_open_flags_t open_flags(vio::file_open_flag_t::rdwr);
  if (!_file_exists)
  {
    open_flags |= vio::file_open_flag_t::creat;
  }

  _file.reset();

#ifdef WIN32
  int open_mode = _S_IREAD | _S_IWRITE;
#else
  int open_mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH;
#endif

  auto open_result = vio::open_file(_event_loop, _file_name, open_flags, open_mode);
  if (!open_result.has_value())
  {
    dew_error_t error;
    error.code = open_result.error().code;
    error.msg = open_result.error().msg;
    return error;
  }
  _file = std::move(open_result.value());

  if (truncate)
  {
    uv_fs_t request = {};
    uv_fs_ftruncate(_event_loop.loop(), &request, (**_file).handle, 0, NULL);

    // When the file already existed the constructor skipped reserving the index/superblock region
    // at offset 0 (it only reserves that for a brand-new file). Truncation discards the existing
    // dataset, so re-establish the reservation here to match the fresh-file initial state --
    // otherwise the first allocate_blob would hand out offset 0 and write_index would later clobber
    // that blob with the 128-byte index.
    if (_file_exists)
    {
      auto index = _blob_manager.register_blob({_serialized_index_size});
      assert(index.data == 0);
      (void)index;
    }
  }

  return {};
}

dew_error_t packed_file_backend_t::read_index(index_load_t &out)
{
  // The synchronous entry point is now just a wait around the coroutine below -- one implementation,
  // two ways in. Callers are the converter opening from its constructing thread and the CLI tools;
  // loop-side code (the query engine) calls read_index_async and never blocks here.
  return run_on_loop_blocking(_event_loop, [this, &out]() { return do_read_index(out); });
}

vio::task_t<dew_error_t> packed_file_backend_t::read_index_async(index_load_t &out)
{
  co_return co_await do_read_index(out);
}

vio::task_t<dew_error_t> packed_file_backend_t::do_read_index(index_load_t &out)
{
  dew_error_t error;
  // Closing the file on failure is what tells everything downstream the backend is unusable; it has to
  // happen on every early exit, and there are a dozen. A scope guard is the only way to keep that true
  // as the function grows -- but note it now runs at CO_RETURN, i.e. when the coroutine frame unwinds,
  // not at the end of a stack frame. Same guarantee, different mechanism.
  struct close_on_error_t
  {
    std::optional<vio::auto_close_file_t> &file;
    dew_error_t &error;
    ~close_on_error_t()
    {
      if (error.code != 0)
      {
        file.reset();
      }
    }
  } closer{_file, error};

  if (!_file)
  {
    error = {1, "packed file is not open"};
    co_return error;
  }
  auto &file = **_file;

  auto index_buffer = std::make_unique<uint8_t[]>(_serialized_index_size);
  {
    auto read = co_await vio::read_file(_event_loop, file, index_buffer.get(), _serialized_index_size, 0);
    if (!read.has_value())
    {
      error = {read.error().code != 0 ? read.error().code : 1, read.error().msg};
      co_return error;
    }
    if (uint32_t(read.value()) != _serialized_index_size)
    {
      error = {1, "could not read index"};
      co_return error;
    }
  }

  storage_location_t free_blobs;
  storage_location_t attribute_configs;
  storage_location_t tree_registry;
  storage_location_t compression_stats;
  storage_location_t perf_stats;
  index_extras_t extras;
  error = deserialize_index(index_buffer.get(), _serialized_index_size, free_blobs, attribute_configs, tree_registry, compression_stats, perf_stats, &extras);
  if (error.code != 0)
  {
    co_return error;
  }
  memcpy(_dataset_uuid, extras.dataset_uuid, sizeof(_dataset_uuid));
  _residency_location = extras.residency_table;
  if (extras.residency_table.size > 0)
  {
    // A residency table on disk means this cache was written with the tier enabled: restore it so
    // remote-only blobs stay reachable and resident accounting continues. (deserialize applies the
    // unclean-shutdown demotion recorded in the table itself.)
    std::unique_ptr<uint8_t[]> residency_buffer;
    error = co_await co_read_into_buffer(_event_loop, file, extras.residency_table, residency_buffer);
    if (error.code != 0)
    {
      error = {1, "Failed to read blob residency table: " + error.msg};
      co_return error;
    }
    if (!_residency)
      _residency = std::make_unique<blob_residency_t>();
    uint64_t cap_before = _residency->cap();
    error = _residency->deserialize(residency_buffer.get(), extras.residency_table.size);
    if (error.code != 0)
      co_return error;
    if (cap_before) // an explicitly configured cap overrides the persisted one
      _residency->set_cap(cap_before);
  }

  error = co_await co_read_into_buffer(_event_loop, file, free_blobs, out.free_blobs);
  out.free_blobs_size = free_blobs.size;
  if (error.code != 0)
  {
    error = {1, "Failed to read free blobs: " + error.msg};
    co_return error;
  }

  error = co_await co_read_into_buffer(_event_loop, file, attribute_configs, out.attribute_configs);
  out.attribute_configs_size = attribute_configs.size;
  if (error.code != 0)
  {
    error = {1, "Failed to read attribute_configs: " + error.msg};
    co_return error;
  }

  error = co_await co_read_into_buffer(_event_loop, file, tree_registry, out.tree_registry);
  out.tree_registry_size = tree_registry.size;
  if (error.code != 0)
  {
    error = {1, "Failed to read tree_registry: " + error.msg};
    co_return error;
  }

  _stats_location = compression_stats;
  _perf_stats_location = perf_stats;
  _tree_registry_location = tree_registry;

  // Stats and perf are optional: a dataset written without them is not broken, so a failure here is
  // dropped rather than propagated -- matching the pre-coroutine behavior, which ignored the error out
  // parameter for exactly these two.
  if (compression_stats.size > 0)
  {
    dew_error_t ignored = co_await co_read_into_buffer(_event_loop, file, compression_stats, out.stats);
    out.stats_size = ignored.code == 0 ? compression_stats.size : 0;
  }

  if (perf_stats.size > 0)
  {
    dew_error_t ignored = co_await co_read_into_buffer(_event_loop, file, perf_stats, out.perf);
    out.perf_size = ignored.code == 0 ? perf_stats.size : 0;
  }

  co_return error;
}

dew_error_t packed_file_backend_t::restore_allocator(const std::unique_ptr<uint8_t[]> &data, uint32_t size)
{
  return _blob_manager.deserialize(data, size);
}

void packed_file_backend_t::allocate_blob(uint32_t size, blob_kind_t kind, storage_location_t &out)
{
  std::unique_lock<std::mutex> lock(_mutex);
  out.file_id = 0;
  out.size = size;
  free_blob_manager_t::blob_size_t blob_size = {size};
  out.offset = _blob_manager.register_blob(blob_size).data;
  if (_residency)
  {
    if (kind == blob_kind_t::data)
    {
      _data_blobs.emplace_back(out.offset, size); // spill-existing candidates, appended in offset-ish order
      _write_pending.insert(out.offset);          // shields the offset from the spill pass until written
    }
    // Hard-capped DATA blobs divert: the offset stays reserved (permanent identity) but the bytes
    // go to the spill area at write time and resident bytes are never charged. Metadata is always
    // local (pinned): it is rewritten each checkpoint and reclaimed through the freed path.
    if (_spill && kind == blob_kind_t::data && _residency->over_hard(size))
    {
      _divert_on_write.insert(out.offset);
      return;
    }
    _residency->account_alloc(size); // resident bytes cover ALL local bytes, data and metadata alike
  }
}

vio::task_t<dew_error_t> packed_file_backend_t::write_allocated(storage_location_t location, std::shared_ptr<uint8_t[]> data)
{
  assert(location.size > 0);
  assert(data != nullptr);

  bool divert = false;
  if (_residency && _spill)
  {
    std::unique_lock<std::mutex> lock(_mutex);
    divert = _divert_on_write.erase(location.offset) > 0;
  }
  // Whatever path this write takes, the offset stops being spill-pass-shielded when we return.
  struct write_pending_guard_t
  {
    packed_file_backend_t *self;
    uint64_t offset;
    ~write_pending_guard_t()
    {
      if (!self->_residency)
        return;
      std::unique_lock<std::mutex> lock(self->_mutex);
      self->_write_pending.erase(offset);
    }
  } write_pending_guard{this, location.offset};
  if (divert)
  {
    // Over the hard cap at allocation time: the blob is born remote_spilled -- its offset identity
    // is reserved, its bytes live in a spill segment. The PUT completing is what releases the
    // writer's completion callback, so ingestion throttles to network speed under pressure.
    uint64_t remote_id = 0;
    auto err = co_await _spill->spill_blob(data.get(), location.size, remote_id);
    if (err.code != 0)
      co_return err;
    _residency->mark_spilled_remote(location.offset, location.size, remote_id);
    co_return dew_error_t{};
  }

  auto &file = **_file;
  auto result = co_await vio::write_file(_event_loop, file, data.get(), location.size, int64_t(location.offset));
  dew_error_t error;
  if (!result.has_value())
  {
    if (_residency && _spill && result.error().code == UV_ENOSPC)
    {
      // Disk full below the cap (shared disk): divert reactively instead of poisoning the
      // conversion. The alloc charge is reversed -- the bytes never landed locally.
      uint64_t remote_id = 0;
      auto err = co_await _spill->spill_blob(data.get(), location.size, remote_id);
      if (err.code != 0)
        co_return err;
      _residency->mark_spilled_remote(location.offset, location.size, remote_id);
      _residency->account_freed(location.size);
      co_return dew_error_t{};
    }
    error.code = result.error().code;
    error.msg = result.error().msg;
  }
  co_return error;
}

vio::task_t<dew_error_t> packed_file_backend_t::read_blob(storage_location_t location, uint8_t *dst, uint32_t &bytes_read)
{

  // Cache-tier dispatch: one hash lookup in a map that stays EMPTY until upload/spill pressure
  // exists. Tracked blobs read locally (with LRU touch + an in-flight guard that blocks a punch
  // racing the suspended pread) or from their remote copy.
  if (_residency)
  {
    if (auto *entry = _residency->find(location.offset))
    {
      if (_residency->has_local_bytes(*entry))
      {
        _residency->touch(*entry);
        _residency->begin_local_read(*entry);
        auto &file = **_file;
        auto result = co_await vio::read_file(_event_loop, file, dst, location.size, int64_t(location.offset));
        // `entry` may DANGLE now: any blob tracked while the pread was in flight can grow the
        // residency slab (vector reallocation). Re-find by offset -- writing through the stale
        // pointer would corrupt freed heap.
        if (auto *entry_after = _residency->find(location.offset))
          _residency->end_local_read(*entry_after);
        dew_error_t error;
        if (!result.has_value())
        {
          error.code = result.error().code;
          error.msg = result.error().msg;
        }
        else if (result.value() != location.size)
        {
          // Short read: the blob is not all there. Reporting success would hand the caller a buffer
          // whose tail it never wrote -- see the note on object_backend_t::read_blob.
          error.code = 1;
          error.msg = "Could not read the entire blob";
        }
        else
        {
          bytes_read = uint32_t(result.value());
        }
        co_return error;
      }
      if (entry->state == blob_residency_state_t::remote_spilled)
      {
        if (!_spill)
          co_return dew_error_t{1, "Blob is spilled but no spill store is attached"};
        auto err = co_await _spill->read(entry->remote_id, dst, location.size);
        if (err.code == 0)
          bytes_read = location.size;
        co_return err;
      }
      // remote_uploaded: read from the destination's final layout -- the blob IS the whole object
      // data/{object_id:08x} (remote_id = object_id << 32; no Range header, capacity-checked).
      // The spill io_manager IS the destination bucket.
      if (!_spill_io)
        co_return dew_error_t{1, "Blob was evicted to the destination; destination reads not attached"};
      assert(uint32_t(entry->remote_id & 0xFFFFFFFFu) == 0 && "dataset objects hold exactly one blob");
      auto remote = co_await _spill_io->read_object_all(bucket_data_object_name(uint32_t(entry->remote_id >> 32)), dst, location.size);
      if (!remote.has_value())
        co_return dew_error_t{int(remote.error().code), remote.error().msg};
      if (remote.value() != location.size)
        co_return dew_error_t{1, "Destination object size does not match the blob"};
      bytes_read = location.size;
      co_return dew_error_t{};
    }
  }
  auto &file = **_file;
  auto result = co_await vio::read_file(_event_loop, file, dst, location.size, int64_t(location.offset));
  dew_error_t error;
  if (!result.has_value())
  {
    error.code = result.error().code;
    error.msg = result.error().msg;
  }
  else
  {
    bytes_read = uint32_t(result.value());
  }
  co_return error;
}

vio::task_t<dew_error_t> packed_file_backend_t::write_index(checkpoint_t checkpoint)
{
  auto make_error = [](std::string msg) {
    dew_error_t e;
    e.code = -1;
    e.msg = std::move(msg);
    return e;
  };

  // Spill flush first (a suspension point): every spill locator the residency snapshot below will
  // persist must be backed by a durable segment. Done before ANY allocator work so the allocator
  // block stays synchronous.
  if (_residency && _spill)
  {
    auto flush_error = co_await _spill->flush();
    if (flush_error.code != 0)
      co_return flush_error;
  }

  // ---- Synchronous allocator block (no co_await): concurrent writes interleave with this
  // coroutine only at suspension points, so everything between here and the first file write is
  // atomic with respect to allocate_blob. Rules that make the checkpoint safe against them:
  //   1. New checkpoint metadata is allocated on the LIVE manager -- a concurrent data blob can
  //      never be handed the same range.
  //   2. Superseded ranges (freed + old metadata) return to the LIVE free list only AFTER the new
  //      superblock commits -- pre-commit, nothing can overwrite bytes the OLD index references,
  //      and a failed checkpoint needs no free-list rollback.
  //   3. The on-disk free-list is serialized from a COPY that has those ranges freed, so the new
  //      index recovers with the space reclaimable. Allocations made while the file writes below
  //      are in flight live only in the LIVE manager (they are committed by the NEXT checkpoint,
  //      exactly as allocations between checkpoints always were).
  uint64_t freed_bytes = 0;
  std::vector<std::pair<uint64_t, uint32_t>> punch_freed;         // tracked blobs freed this checkpoint
  std::vector<std::pair<uint64_t, uint32_t>> deferred_unregister; // returned to the LIVE free list post-commit

  storage_location_t attributes_location;
  storage_location_t stats_location;
  storage_location_t perf_location;
  storage_location_t residency_location = {};
  std::vector<uint8_t> serialized_residency;
  uint32_t residency_generation = 0;
  bool clean_snapshot = false;
  serialized_free_blob_manager_t serialized_blob;
  storage_location_t serialized_blob_location;
  {
    std::unique_lock<std::mutex> lock(_mutex);

    for (auto &location : checkpoint.freed)
    {
      if (_residency)
      {
        auto forgotten = _residency->forget(location.offset);
        if (forgotten.existed)
        {
          // Freed while uploaded/spilled: the entry leaves the snapshot this checkpoint writes, its
          // spill reference dies here (the segment is deleted by the post-commit sweep once nothing
          // references it), and any local bytes are punched post-commit. The offset must NEVER
          // return to the allocator: through a recycled offset a stale remote copy would shadow the
          // new blob. (forget() settles the resident-bytes accounting itself.)
          if (forgotten.spilled && _spill)
            _spill->deref(forgotten.remote_id);
          if (forgotten.punch_local)
            punch_freed.emplace_back(location.offset, location.size);
          continue;
        }
      }
      deferred_unregister.emplace_back(location.offset, location.size);
      freed_bytes += location.size;
    }
    for (auto *old_location : {&_attributes_location, &_blobs_location, &_stats_location, &_perf_stats_location, &_tree_registry_location, &_residency_location})
    {
      if (old_location->offset > 0)
      {
        deferred_unregister.emplace_back(old_location->offset, old_location->size);
        freed_bytes += old_location->size;
      }
    }

    // New checkpoint metadata: LIVE allocations (rule 1). Never lands in still-referenced freed
    // space (rule 2 keeps that space allocated until commit).
    attributes_location.offset = _blob_manager.register_blob({checkpoint.attribute_configs_size}).data;
    attributes_location.size = checkpoint.attribute_configs_size;
    stats_location.offset = _blob_manager.register_blob({checkpoint.stats_size}).data;
    stats_location.size = checkpoint.stats_size;
    perf_location.offset = _blob_manager.register_blob({checkpoint.perf_size}).data;
    perf_location.size = checkpoint.perf_size;

    // Cache tier: snapshot the residency table into the same allocator transaction. The snapshot
    // generation is committed only after the superblock lands (the durable-bit barrier).
    if (_residency)
    {
      residency_generation = _residency->serialize_generation();
      clean_snapshot = _clean_shutdown_next_checkpoint;
      _clean_shutdown_next_checkpoint = false; // one-shot
      serialized_residency = _residency->serialize(clean_snapshot);
      if (serialized_residency.empty())
        co_return make_error("Failed to serialize residency table");
      residency_location.offset = _blob_manager.register_blob({uint32_t(serialized_residency.size())}).data;
      residency_location.size = uint32_t(serialized_residency.size());
    }

    // Serialization copy (rule 3): the live state (including the metadata just allocated), with the
    // superseded ranges freed.
    auto snapshot_manager = _blob_manager;
    for (auto &[unregister_offset, unregister_size] : deferred_unregister)
    {
      if (!snapshot_manager.unregister_blob({unregister_offset}, {unregister_size}))
        co_return make_error("Failed to remove blob"); // live untouched: new metadata blobs leak until reuse, nothing corrupts
    }
    // The free-list blob itself: allocate LIVE, claim the same range in the copy, size by fixpoint
    // (the claim can change the copy's section count and thereby its serialized size).
    for (int attempt = 0;; attempt++)
    {
      if (attempt == 5)
        co_return make_error("Failed to place free-list blob");
      uint32_t list_size = snapshot_manager.calculate_serialized_size() + 32 * attempt;
      auto list_offset = _blob_manager.register_blob({list_size});
      if (!snapshot_manager.claim_blob(list_offset, {list_size}))
        co_return make_error("Free-list blob range not free in snapshot");
      if (snapshot_manager.calculate_serialized_size() <= list_size)
      {
        serialized_blob = snapshot_manager.serialize_preallocated(list_offset, {list_size});
        break;
      }
      if (!snapshot_manager.unregister_blob(list_offset, {list_size}) || !_blob_manager.unregister_blob(list_offset, {list_size}))
        co_return make_error("Failed to retry free-list blob placement");
    }
    serialized_blob_location = {0, serialized_blob.size, serialized_blob.offset};
  }
  // ---- End of the synchronous allocator block. ----

  if (_residency)
  {
    // These metadata blobs were allocated directly on the manager (not via allocate_blob), so
    // charge them here; their predecessors are settled via account_freed after the commit below.
    _residency->account_alloc(uint64_t(attributes_location.size) + stats_location.size + perf_location.size + residency_location.size + serialized_blob_location.size);
  }

  auto &file = **_file;
  // On any pre-commit failure: unregister this checkpoint's own metadata blobs so the space is not
  // leaked. The superseded ranges were never freed (rule 2), so there is nothing else to undo.
  auto rollback_metadata = [&]() {
    std::unique_lock<std::mutex> lock(_mutex);
    (void)_blob_manager.unregister_blob({attributes_location.offset}, {attributes_location.size});
    (void)_blob_manager.unregister_blob({stats_location.offset}, {stats_location.size});
    (void)_blob_manager.unregister_blob({perf_location.offset}, {perf_location.size});
    if (residency_location.size > 0)
      (void)_blob_manager.unregister_blob({residency_location.offset}, {residency_location.size});
    (void)_blob_manager.unregister_blob({serialized_blob_location.offset}, {serialized_blob_location.size});
    if (_residency)
      _residency->account_freed(uint64_t(attributes_location.size) + stats_location.size + perf_location.size + residency_location.size + serialized_blob_location.size);
  };
  auto make_io_error = [](const vio::error_t &e) {
    dew_error_t error;
    error.code = e.code;
    error.msg = e.msg;
    return error;
  };

  auto result1 = co_await vio::write_file(_event_loop, file, reinterpret_cast<const uint8_t *>(serialized_blob.data.get()), serialized_blob.size, int64_t(serialized_blob.offset));
  if (!result1.has_value())
  {
    rollback_metadata();
    co_return make_io_error(result1.error());
  }

  auto result2 = co_await vio::write_file(_event_loop, file, checkpoint.attribute_configs.get(), attributes_location.size, int64_t(attributes_location.offset));
  if (!result2.has_value())
  {
    rollback_metadata();
    co_return make_io_error(result2.error());
  }

  auto result3 = co_await vio::write_file(_event_loop, file, checkpoint.stats.get(), stats_location.size, int64_t(stats_location.offset));
  if (!result3.has_value())
  {
    rollback_metadata();
    co_return make_io_error(result3.error());
  }

  auto result4 = co_await vio::write_file(_event_loop, file, checkpoint.perf.get(), perf_location.size, int64_t(perf_location.offset));
  if (!result4.has_value())
  {
    rollback_metadata();
    co_return make_io_error(result4.error());
  }

  if (_residency)
  {
    auto result5 = co_await vio::write_file(_event_loop, file, serialized_residency.data(), residency_location.size, int64_t(residency_location.offset));
    if (!result5.has_value())
    {
      rollback_metadata();
      co_return make_io_error(result5.error());
    }
  }

  // Write the index/superblock LAST (the crash-safety commit point). The extras region carries the
  // residency-table location + dataset uuid; with the tier dormant it is all zeros -- byte-identical
  // to the historical layout.
  index_extras_t extras;
  extras.residency_table = residency_location;
  memcpy(extras.dataset_uuid, _dataset_uuid, sizeof(extras.dataset_uuid));
  auto serialized_index = serialize_index(_serialized_index_size, serialized_blob_location, attributes_location, checkpoint.tree_registry, stats_location, perf_location, &extras);
  auto result = co_await vio::write_file(_event_loop, file, serialized_index.get(), _serialized_index_size, 0);
  if (!result.has_value())
  {
    rollback_metadata();
    co_return make_io_error(result.error());
  }

  {
    // Commit point passed: superseded ranges become reusable in the LIVE manager (rule 2). Live
    // allocations made while the writes above were in flight are untouched -- the old
    // copy-and-replace here silently erased them, double-allocating their offsets.
    std::unique_lock<std::mutex> lock(_mutex);
    for (auto &[unregister_offset, unregister_size] : deferred_unregister)
    {
      if (!_blob_manager.unregister_blob({unregister_offset}, {unregister_size}))
        co_return make_error("Failed to return superseded blob to the free list");
    }
  }
  _blobs_location = serialized_blob_location;
  _attributes_location = attributes_location;
  _stats_location = stats_location;
  _perf_stats_location = perf_location;
  _tree_registry_location = checkpoint.tree_registry;
  _residency_location = residency_location;

  uv_fs_t req = {};
  uv_fs_fsync(_event_loop.loop(), &req, file.handle, NULL);

  if (_residency)
  {
    // The remote facts snapshotted above are now durable: destructive actions (punch, spill-segment
    // delete) on them are permitted from here on. Also settle the accounting for every byte range
    // this checkpoint returned to the allocator.
    _residency->commit_durable(residency_generation);
    _residency->account_freed(freed_bytes);
    // Freed-while-tracked blobs: their entries left the snapshot committed above and their offsets
    // never return to the allocator, so this punch is the only physical reclaim they get.
    for (auto &[punch_offset, punch_size] : punch_freed)
    {
      if (_punch_supported == 0)
        break;
      auto punch = file_hole_punch((**_file).handle, punch_offset, punch_size);
      if (punch.status == hole_punch_status_t::unsupported)
      {
        _punch_supported = 0;
        break;
      }
      if (punch.status != hole_punch_status_t::ok)
        break;
      _punch_supported = 1;
    }
    if (_spill)
    {
      auto sweep_error = co_await _spill->sweep_after_checkpoint();
      (void)sweep_error; // best-effort reclamation; a failed delete just leaves a re-sweepable object
      // Spill-existing reclaim: local_spilled facts this checkpoint made durable can now drop their
      // local bytes (punch). Collect on the loop, then punch (same-loop atomicity vs read dispatch).
      // Suppressed after a CLEAN snapshot: flipping local_* state post-commit would make the clean
      // snapshot lie about local bytes on a crash (the whole point of the demotion it opted out of).
      if (!clean_snapshot && _punch_supported != 0 && _residency->over_soft())
      {
        std::vector<std::pair<uint64_t, uint32_t>> punchable;
        _residency->for_each([&](const blob_residency_entry_t &e) {
          if (e.state == blob_residency_state_t::local_spilled && e.durable && e.inflight_local_reads == 0)
            punchable.emplace_back(e.offset, e.size);
        });
        for (auto &[offset, size] : punchable)
        {
          auto punch = file_hole_punch((**_file).handle, offset, size);
          if (punch.status == hole_punch_status_t::unsupported)
          {
            _punch_supported = 0;
            break;
          }
          if (punch.status != hole_punch_status_t::ok)
            break;
          _punch_supported = 1;
          _residency->drop_spilled_local(offset);
        }
      }
    }
    // Fresh durable facts may have unlocked eviction; reclaim while we're on the loop anyway.
    // (Not after a clean snapshot -- same reason as above.)
    if (!clean_snapshot)
      run_eviction_pass();
  }
  co_return dew_error_t{};
}

} // namespace dew::core
