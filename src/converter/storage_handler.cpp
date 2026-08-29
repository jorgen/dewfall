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
#include "storage_handler.hpp"
#include "compressor_zstd.hpp"
#include "input_header.hpp"
#ifndef __EMSCRIPTEN__
#include "packed_file_backend.hpp" // cache-tier configuration (native only)
#include <vio/objstore/create_object_store.h>
#endif

#include <cassert>
#include <cstring>

#include <algorithm>
#include <future>
#include <limits>
#include <random>
#include <utility>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

namespace dew::converter
{
using namespace dew::core;

storage_handler_t::storage_handler_t(const std::string &url, vio::thread_pool_t &thread_pool, attributes_configs_t &attributes_configs, perf_stats_t &perf_stats, vio::event_pipe_t<void> &index_written,
                                     vio::event_pipe_t<dew_error_t> &storage_error_pipe, dew_error_t &error)
  : _thread_pool(thread_pool)
  // No connection string here: the converter installs its credentials as vio's process-global
  // override in dew_converter_set_destination, before any backend is built.
  , _reader(url, std::string_view{}, thread_pool, perf_stats, storage_error_pipe, error)
  , _event_loop(_reader.event_loop())
  , _attributes_configs(attributes_configs)
  , _perf_stats(perf_stats)
  , _index_written(index_written)
  , _storage_error(storage_error_pipe)
  , _write_event_pipe(_event_loop, vio::event_bind_t::bind(*this, &storage_handler_t::handle_write_events))
  , _write_attributes_pipe(_event_loop, vio::event_bind_t::bind(*this, &storage_handler_t::handle_write_attributes))
  , _write_trees_pipe(_event_loop, vio::event_bind_t::bind(*this, &storage_handler_t::handle_write_trees))
  , _write_tree_registry_pipe(_event_loop, vio::event_bind_t::bind(*this, &storage_handler_t::handle_write_tree_registry))
  , _write_blob_locations_and_update_header_pipe(_event_loop, vio::event_bind_t::bind(*this, &storage_handler_t::handle_write_blob_locations_and_update_header))
{
  set_compressor(compression_method_t::zstd);
}

storage_handler_t::~storage_handler_t()
{
  // Safety net: quiesce the shared loop before this object's write pipes destruct. The reader owns the
  // loop and the backend, so its stop_loop does the draining; ordering matters because an in-flight read
  // and a queued write both run on that one loop.
  stop_loop();
}

void storage_handler_t::stop_loop()
{
  _reader.stop_loop();
}

dew_error_t storage_handler_t::read_index(std::unique_ptr<uint8_t[]> &free_blobs_buffer, uint32_t &free_blobs_size, std::unique_ptr<uint8_t[]> &attribute_configs_buffer, uint32_t &attribute_configs_size,
                                      std::unique_ptr<uint8_t[]> &tree_registry_buffer, uint32_t &tree_registry_size)
{
  index_load_t load;
  auto error = _reader.read_index(load);
  if (error.code != 0)
    return error;

  free_blobs_buffer = std::move(load.free_blobs);
  free_blobs_size = load.free_blobs_size;
  attribute_configs_buffer = std::move(load.attribute_configs);
  attribute_configs_size = load.attribute_configs_size;
  tree_registry_buffer = std::move(load.tree_registry);
  tree_registry_size = load.tree_registry_size;

  if (load.stats && load.stats_size > 0)
    _compression_stats = compression_stats_t::deserialize(load.stats.get(), load.stats_size);
  if (load.perf && load.perf_size > 0)
    _deserialized_perf_stats = perf_stats_t::deserialize(load.perf.get(), load.perf_size);

  return error;
}

dew_error_t storage_handler_t::deserialize_free_blobs(const std::unique_ptr<uint8_t[]> &data, uint32_t size)
{
  return _reader.backend()->restore_allocator(data, size);
}

dew_error_t storage_handler_t::upgrade_to_write(bool truncate)
{
  auto error = _reader.backend()->open_for_write(truncate);
  if (error.code != 0)
  {
    auto error_copy = error;
    _storage_error.post_event(std::move(error));
    return error_copy;
  }
  return {};
}

void storage_handler_t::write(const storage_header_t &header, attributes_id_t attributes_id, attribute_buffers_t &&buffers,
                              std::function<void(const storage_header_t &storageheader, attributes_id_t attrib_id, std::vector<storage_location_t> locations, const dew_error_t &error)> done)
{
  _write_event_pipe.post_event(std::make_tuple(header, attributes_id, std::move(buffers), done));
}

void storage_handler_t::write_trees(std::vector<tree_id_t> &&tree_ids, std::vector<serialized_tree_t> &&serialized_trees,
                                    std::function<void(std::vector<tree_id_t> &&, std::vector<storage_location_t> &&, dew_error_t &&)> done)
{
  _write_trees_pipe.post_event(std::make_tuple(std::move(tree_ids), std::move(serialized_trees), done));
}

void storage_handler_t::write_tree_registry(serialized_tree_registry_t &&serialized_tree_registry, std::function<void(storage_location_t, dew_error_t &&error)> done)
{
  _write_tree_registry_pipe.post_event(std::move(serialized_tree_registry), std::move(done));
}

void storage_handler_t::write_blob_locations_and_update_header(storage_location_t location, std::vector<storage_location_t> &&old_locations, std::function<void(dew_error_t &&error)> done)
{
  _write_blob_locations_and_update_header_pipe.post_event(std::move(location), std::move(old_locations), std::move(done));
}

static void compute_attribute_min_max(const uint8_t *data, uint32_t size, const point_format_t &format, double &out_min, double &out_max)
{
  out_min = std::numeric_limits<double>::max();
  out_max = std::numeric_limits<double>::lowest();
  int elem_size = size_for_format(format.type) * static_cast<int>(format.components);
  if (elem_size <= 0 || size == 0)
    return;
  uint32_t count = size / static_cast<uint32_t>(elem_size);
  if (count == 0)
    return;
  for (uint32_t i = 0; i < count; i++)
  {
    const uint8_t *elem = data + i * elem_size;
    double val = 0.0;
    switch (format.type)
    {
    case dew_type_u8: { uint8_t v; memcpy(&v, elem, 1); val = double(v); break; }
    case dew_type_i8: { int8_t v; memcpy(&v, elem, 1); val = double(v); break; }
    case dew_type_u16: { uint16_t v; memcpy(&v, elem, 2); val = double(v); break; }
    case dew_type_i16: { int16_t v; memcpy(&v, elem, 2); val = double(v); break; }
    case dew_type_u32: { uint32_t v; memcpy(&v, elem, 4); val = double(v); break; }
    case dew_type_i32: { int32_t v; memcpy(&v, elem, 4); val = double(v); break; }
    case dew_type_r32: { float v; memcpy(&v, elem, 4); val = double(v); break; }
    case dew_type_u64: { uint64_t v; memcpy(&v, elem, 8); val = double(v); break; }
    case dew_type_i64: { int64_t v; memcpy(&v, elem, 8); val = double(v); break; }
    case dew_type_r64: { double v; memcpy(&v, elem, 8); val = v; break; }
    default: continue;
    }
    if (val < out_min) out_min = val;
    if (val > out_max) out_max = val;
  }
}

static bool serialize_points(const storage_header_t &header, const dew_blob_t &points, dew_blob_t &serialize_data, std::shared_ptr<uint8_t[]> &data_owner)
{
  serialize_data.size = sizeof(header) + points.size;
  data_owner = std::make_shared<uint8_t[]>(serialize_data.size);
  serialize_data.data = data_owner.get();
  auto output_bytes = static_cast<uint8_t *>(serialize_data.data);
  memcpy(output_bytes, &header, sizeof(header));
  memcpy(output_bytes + sizeof(header), points.data, points.size);
  return true;
}

vio::task_t<void> storage_handler_t::do_write(const std::shared_ptr<uint8_t[]> &data, const storage_location_t &location)
{
  assert(location.size > 0);
  assert(data != nullptr);
  // The blob manager reuses freed offsets, so an offset previously read (and cached) may now
  // hold different data. Invalidate the stale read-cache entry for this (file_id, offset) so a
  // later read does not return the old blob's bytes.
  _reader.invalidate(location);
  auto error = co_await _reader.backend()->write_allocated(location, data);
  if (error.code != 0)
  {
    _storage_error.post_event(std::move(error));
  }
}

vio::task_t<void> storage_handler_t::do_write_events(storage_header_t header, attributes_id_t attributes_id, attribute_buffers_t attribute_buffers,
                                                     std::function<void(const storage_header_t &, attributes_id_t, std::vector<storage_location_t> &&, const dew_error_t &error)> done)
{
  auto write_start = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(_mutex);

  bool is_leaf = input_data_id_is_leaf(header.input_id);
  if (is_leaf && !input_data_id_is_collapsed_leaf(header.input_id))
    _seen_input_files.insert(header.input_id.data); // keyed by real input-file ids only

  uint64_t uncompressed_total = 0;
  for (auto &buf : attribute_buffers.buffers)
    uncompressed_total += buf.size;

  auto formats = _attributes_configs.get_format_components(attributes_id);
  auto &attributes = _attributes_configs.get(attributes_id);
  int buffer_count = int(attribute_buffers.buffers.size());

  // Prepare per-buffer data for compression
  struct buffer_info_t
  {
    std::shared_ptr<uint8_t[]> data_owner;
    uint8_t *raw;
    uint32_t size;
    point_format_t format;
    std::string attr_name;
    bool is_lod;
  };
  std::vector<buffer_info_t> buffer_infos(buffer_count);

  bool is_lod = !is_leaf;

  for (int i = 0; i < buffer_count; i++)
  {
    auto &info = buffer_infos[i];
    if (i == 0)
    {
      dew_blob_t buffer_data;
      serialize_points(header, attribute_buffers.buffers[i], buffer_data, info.data_owner);
      info.raw = static_cast<uint8_t *>(buffer_data.data);
      info.size = buffer_data.size;
      info.format = header.point_format;
    }
    else
    {
      info.raw = static_cast<uint8_t *>(attribute_buffers.buffers[i].data);
      info.size = attribute_buffers.buffers[i].size;
      info.data_owner = std::move(attribute_buffers.data[i]);
      if (i < int(formats.size()))
        info.format = formats[i];
      else
        info.format = {dew_type_u8, dew_components_1};
    }

    if (i < int(attributes.attributes.size()))
      info.attr_name = std::string(attributes.attributes[i].name, attributes.attributes[i].name_size);
    else
      info.attr_name = "unknown";
    info.is_lod = is_lod;
  }

  lock.unlock();

  std::vector<storage_location_t> locations(buffer_count);
  dew_error_t error;

  if (_compressor)
  {
    auto *compressor = _compressor.get();
    uint32_t point_count = header.point_count;

    // Build work items for schedule_work
    std::vector<std::function<std::expected<compressed_write_data_t, vio::error_t>()>> work_items;
    work_items.reserve(buffer_count);

    for (int i = 0; i < buffer_count; i++)
    {
      auto &info = buffer_infos[i];
      work_items.push_back([compressor, raw = info.raw, size = info.size, data_owner = info.data_owner,
                            format = info.format, point_count, i, attr_name = info.attr_name, is_lod = info.is_lod]() -> std::expected<compressed_write_data_t, vio::error_t>
      {
        double attr_min = std::numeric_limits<double>::max();
        double attr_max = std::numeric_limits<double>::lowest();
        if (i > 0)
        {
          compute_attribute_min_max(raw, size, format, attr_min, attr_max);
        }
        auto compressed = try_compress_constant(raw, size, format);
        if (!compressed.data)
          compressed = compressor->compress(raw, size, format, point_count);

        compressed_write_data_t wd;
        wd.buffer_index = i;
        wd.attribute_name = attr_name;
        wd.format = format;
        wd.uncompressed_size = size;
        wd.min_value = attr_min;
        wd.max_value = attr_max;
        wd.is_lod = is_lod;

        if (compressed.error.code != 0)
        {
          wd.data = data_owner;
          wd.size = size;
        }
        else
        {
          wd.data = std::move(compressed.data);
          wd.size = compressed.size;
        }
        return wd;
      });
    }

    auto results = co_await vio::schedule_work(_event_loop, _thread_pool, std::move(work_items));

    // Process compression results on the event loop thread
    for (auto &result : results)
    {
      if (!result.has_value())
      {
        error.code = result.error().code;
        error.msg = result.error().msg;
        continue;
      }
      auto &wd = result.value();

      uint8_t compression_flags = 0;
      if (wd.data && wd.size >= sizeof(compression_header_t) && has_compression_magic(wd.data.get(), wd.size))
      {
        compression_header_t hdr;
        memcpy(&hdr, wd.data.get(), sizeof(hdr));
        compression_flags = hdr.flags;
      }
      _compression_stats.accumulate(wd.attribute_name, wd.format, wd.uncompressed_size, wd.size, wd.min_value, wd.max_value, compression_flags, wd.is_lod);

      auto &location = locations[wd.buffer_index];
      _reader.backend()->allocate_blob(wd.size, storage_backend_t::blob_kind_t::data, location);

      co_await do_write(wd.data, location);
    }
  }
  else
  {
    for (int i = 0; i < buffer_count; i++)
    {
      auto &info = buffer_infos[i];
      _compression_stats.accumulate(info.attr_name, info.format, info.size, info.size, std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(), 0, info.is_lod);

      auto &location = locations[i];
      _reader.backend()->allocate_blob(info.size, storage_backend_t::blob_kind_t::data, location);

      co_await do_write(info.data_owner, location);
    }
  }

  // Cache-tier pressure hooks (no-ops outside the cached local backend): reclaim opportunistically
  // after landing this node's blobs; when only a checkpoint can relieve pressure (pending remote
  // facts need their durable flip), request one -- debounced until the processor rearms after the
  // checkpoint completes.
  _reader.backend()->maybe_evict();
  if (_reader.backend()->wants_checkpoint() && _on_checkpoint_request && !_checkpoint_requested.exchange(true, std::memory_order_acq_rel))
    _on_checkpoint_request();

  auto write_end = std::chrono::steady_clock::now();
  auto write_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(write_end - write_start).count());
  if (is_leaf)
    _perf_stats.source_write.record(uncompressed_total, write_us);
  else
    _perf_stats.lod_write.record(uncompressed_total, write_us);
  if (_on_write_progress)
    _on_write_progress();

  if (done)
  {
    done(header, attributes_id, std::move(locations), error);
  }
}

void storage_handler_t::handle_write_events(
  std::tuple<storage_header_t, attributes_id_t, attribute_buffers_t, std::function<void(const storage_header_t &, attributes_id_t, std::vector<storage_location_t> &&, const dew_error_t &error)>> &&event)
{
  auto &&[storage_header, attributes_id, attribute_buffers, done] = std::move(event);
  [](storage_handler_t *self, storage_header_t header, attributes_id_t attrib_id, attribute_buffers_t buffers,
     std::function<void(const storage_header_t &, attributes_id_t, std::vector<storage_location_t> &&, const dew_error_t &error)> done_cb) -> vio::detached_task_t
  {
    co_await self->do_write_events(std::move(header), std::move(attrib_id), std::move(buffers), std::move(done_cb));
  }(this, std::move(storage_header), std::move(attributes_id), std::move(attribute_buffers), std::move(done));
}

void storage_handler_t::write_attributes(std::vector<attribute_write_t> &&writes, std::function<void(std::vector<storage_location_t> &&, const dew_error_t &error)> done)
{
  _write_attributes_pipe.post_event(std::make_tuple(std::move(writes), std::move(done)));
}

vio::task_t<void> storage_handler_t::do_write_attributes(std::vector<attribute_write_t> writes, std::function<void(std::vector<storage_location_t> &&, const dew_error_t &)> done)
{
  auto write_start = std::chrono::steady_clock::now();
  const int count = int(writes.size());
  std::vector<storage_location_t> locations(writes.size());
  dew_error_t error;

  uint64_t uncompressed_total = 0;
  uint64_t lod_bytes = 0;
  for (auto &w : writes)
  {
    uncompressed_total += w.size;
    if (w.is_lod)
      lod_bytes += w.size;
  }

  if (_compressor)
  {
    auto *compressor = _compressor.get();
    std::vector<std::function<std::expected<compressed_write_data_t, vio::error_t>()>> work_items;
    work_items.reserve(size_t(count));
    for (int i = 0; i < count; i++)
    {
      auto &w = writes[i];
      work_items.push_back([compressor, data = w.data, size = w.size, format = w.format, point_count = w.point_count, i,
                            attr_name = w.attribute_name, is_lod = w.is_lod]() -> std::expected<compressed_write_data_t, vio::error_t>
      {
        auto *raw = data.get();
        // Unconditionally, unlike the whole-unit path's `if (i > 0)`: index 0 there is the position
        // buffer, which carries the header and has no meaningful min/max. Every buffer here is an
        // ordinary attribute.
        double attr_min = std::numeric_limits<double>::max();
        double attr_max = std::numeric_limits<double>::lowest();
        compute_attribute_min_max(raw, size, format, attr_min, attr_max);

        auto compressed = try_compress_constant(raw, size, format);
        if (!compressed.data)
          compressed = compressor->compress(raw, size, format, point_count);

        compressed_write_data_t wd;
        wd.buffer_index = i;
        wd.attribute_name = attr_name;
        wd.format = format;
        wd.uncompressed_size = size;
        wd.min_value = attr_min;
        wd.max_value = attr_max;
        wd.is_lod = is_lod;
        if (compressed.error.code != 0)
        {
          // Compression failed: store the bytes verbatim. Same fallback the whole-unit path takes --
          // a blob without the compression magic reads back as raw.
          wd.data = data;
          wd.size = size;
        }
        else
        {
          wd.data = std::move(compressed.data);
          wd.size = compressed.size;
        }
        return wd;
      });
    }

    auto results = co_await vio::schedule_work(_event_loop, _thread_pool, std::move(work_items));

    for (auto &result : results)
    {
      if (!result.has_value())
      {
        error.code = result.error().code;
        error.msg = result.error().msg;
        continue;
      }
      auto &wd = result.value();

      uint8_t compression_flags = 0;
      if (wd.data && wd.size >= sizeof(compression_header_t) && has_compression_magic(wd.data.get(), wd.size))
      {
        compression_header_t hdr;
        memcpy(&hdr, wd.data.get(), sizeof(hdr));
        compression_flags = hdr.flags;
      }
      _compression_stats.accumulate(wd.attribute_name, wd.format, wd.uncompressed_size, wd.size, wd.min_value, wd.max_value, compression_flags, wd.is_lod);

      auto &location = locations[size_t(wd.buffer_index)];
      {
        std::lock_guard<std::mutex> lock(_mutex);
        _reader.backend()->allocate_blob(wd.size, storage_backend_t::blob_kind_t::data, location);
      }
      co_await do_write(wd.data, location);
    }
  }
  else
  {
    for (int i = 0; i < count; i++)
    {
      auto &w = writes[i];
      _compression_stats.accumulate(w.attribute_name, w.format, w.size, w.size, std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(), 0, w.is_lod);
      auto &location = locations[size_t(i)];
      {
        std::lock_guard<std::mutex> lock(_mutex);
        _reader.backend()->allocate_blob(w.size, storage_backend_t::blob_kind_t::data, location);
      }
      co_await do_write(w.data, location);
    }
  }

  // Same post-write housekeeping the whole-unit path does: an amend can add as many bytes as a
  // conversion pass, so it has to be able to relieve cache pressure and request a checkpoint too.
  _reader.backend()->maybe_evict();
  if (_reader.backend()->wants_checkpoint() && _on_checkpoint_request && !_checkpoint_requested.exchange(true, std::memory_order_acq_rel))
    _on_checkpoint_request();

  auto write_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - write_start).count());
  // Split by what the batch actually was rather than by a single flag: an amend covering a whole
  // subtree writes leaf and LOD buffers in one call, and charging the lot to one counter would make
  // both numbers wrong.
  if (uncompressed_total > lod_bytes)
    _perf_stats.source_write.record(uncompressed_total - lod_bytes, write_us);
  if (lod_bytes)
    _perf_stats.lod_write.record(lod_bytes, write_us);
  if (_on_write_progress)
    _on_write_progress();

  if (done)
    done(std::move(locations), error);
}

void storage_handler_t::handle_write_attributes(std::tuple<std::vector<attribute_write_t>, std::function<void(std::vector<storage_location_t> &&, const dew_error_t &)>> &&event)
{
  auto &&[writes, done] = std::move(event);
  [](storage_handler_t *self, std::vector<attribute_write_t> w, std::function<void(std::vector<storage_location_t> &&, const dew_error_t &)> done_cb) -> vio::detached_task_t
  {
    co_await self->do_write_attributes(std::move(w), std::move(done_cb));
  }(this, std::move(writes), std::move(done));
}

vio::task_t<void> storage_handler_t::do_write_trees(std::vector<tree_id_t> tree_ids, std::vector<serialized_tree_t> serialized_trees,
                                                    std::function<void(std::vector<tree_id_t> &&, std::vector<storage_location_t> &&, dew_error_t &&)> done)
{
  std::unique_lock<std::mutex> lock(_mutex);
  std::vector<storage_location_t> locations(tree_ids.size());
  dew_error_t error;

  for (int i = 0; i < int(tree_ids.size()); i++)
  {
    _reader.backend()->allocate_blob(serialized_trees[i].size, storage_backend_t::blob_kind_t::metadata, locations[i]);
  }
  lock.unlock();

  for (int i = 0; i < int(tree_ids.size()); i++)
  {
    auto result = co_await _reader.backend()->write_allocated(locations[i], serialized_trees[i].data);
    if (result.code != 0 && error.code == 0)
    {
      error = result;
    }
  }

  if (done)
  {
    done(std::move(tree_ids), std::move(locations), std::move(error));
  }
}

void storage_handler_t::handle_write_trees(std::tuple<std::vector<tree_id_t>, std::vector<serialized_tree_t>, std::function<void(std::vector<tree_id_t> &&, std::vector<storage_location_t> &&, dew_error_t &&)>> &&event)
{
  auto &&[tree_ids, serialized_trees, done] = std::move(event);
  [](storage_handler_t *self, std::vector<tree_id_t> ids, std::vector<serialized_tree_t> trees,
     std::function<void(std::vector<tree_id_t> &&, std::vector<storage_location_t> &&, dew_error_t &&)> done_cb) -> vio::detached_task_t
  {
    co_await self->do_write_trees(std::move(ids), std::move(trees), std::move(done_cb));
  }(this, std::move(tree_ids), std::move(serialized_trees), std::move(done));
}

vio::task_t<void> storage_handler_t::do_write_tree_registry(serialized_tree_registry_t serialized_tree_registry, std::function<void(storage_location_t, dew_error_t &&error)> done)
{
  std::unique_lock<std::mutex> lock(_mutex);
  storage_location_t location;
  _reader.backend()->allocate_blob(uint32_t(serialized_tree_registry.size), storage_backend_t::blob_kind_t::metadata, location);
  lock.unlock();

  dew_error_t error = co_await _reader.backend()->write_allocated(location, serialized_tree_registry.data);

  if (done)
  {
    done(location, std::move(error));
  }
}

void storage_handler_t::handle_write_tree_registry(serialized_tree_registry_t &&serialized_tree, std::function<void(storage_location_t, dew_error_t &&error)> &&done)
{
  [](storage_handler_t *self, serialized_tree_registry_t reg, std::function<void(storage_location_t, dew_error_t &&error)> done_cb) -> vio::detached_task_t
  {
    co_await self->do_write_tree_registry(std::move(reg), std::move(done_cb));
  }(this, std::move(serialized_tree), std::move(done));
}

vio::task_t<void> storage_handler_t::do_write_blob_locations_and_update_header(storage_location_t new_tree_registry_location, std::vector<storage_location_t> old_locations, std::function<void(dew_error_t &&error)> done)
{
  auto serialized_attributes_configs = _attributes_configs.serialize();

  _compression_stats.input_file_count = static_cast<uint32_t>(_seen_input_files.size());
  _compression_stats.input_file_size_bytes = 0;
  for (auto &[id, size] : _input_file_sizes)
    _compression_stats.input_file_size_bytes += size;
  if (_compressor)
    _compression_stats.method = _compressor->method();

  uint32_t stats_size = 0;
  auto serialized_stats_data = _compression_stats.serialize(stats_size);

  uint32_t perf_size = 0;
  auto serialized_perf_data = _perf_stats.serialize(perf_size);

  checkpoint_t checkpoint;
  checkpoint.tree_registry = new_tree_registry_location;
  checkpoint.freed = std::move(old_locations);
  checkpoint.attribute_configs = serialized_attributes_configs.data;
  checkpoint.attribute_configs_size = serialized_attributes_configs.size;
  checkpoint.stats = serialized_stats_data;
  checkpoint.stats_size = stats_size;
  checkpoint.perf = std::move(serialized_perf_data);
  checkpoint.perf_size = perf_size;

  // The backend writes the metadata blobs, then the index/manifest LAST, fsyncs, commits, and only
  // then reclaims the freed blobs. The index-written event fires only after that whole barrier succeeds.
  auto error = co_await _reader.backend()->write_index(std::move(checkpoint));
  if (error.code != 0)
  {
    done(std::move(error));
    co_return;
  }
  // Call `done` BEFORE posting the index-written event: the tree handler's completion callback
  // publishes the committed watermark synchronously (on this loop), and the processor's
  // index-written handler discriminates pass-concluding commits by reading that watermark --
  // posting first would let it observe the pre-commit value and drop the pass conclusion.
  done(dew_error_t{});
  _index_written.post_event();
}

void storage_handler_t::handle_write_blob_locations_and_update_header(storage_location_t &&new_tree_registry_location, std::vector<storage_location_t> &&old_locations, std::function<void(dew_error_t &&error)> &&done)
{
  [](storage_handler_t *self, storage_location_t loc, std::vector<storage_location_t> old_locs,
     std::function<void(dew_error_t &&error)> done_cb) -> vio::detached_task_t
  {
    co_await self->do_write_blob_locations_and_update_header(std::move(loc), std::move(old_locs), std::move(done_cb));
  }(this, std::move(new_tree_registry_location), std::move(old_locations), std::move(done));
}

void storage_handler_t::register_input_file_size(uint32_t file_id, uint64_t size_bytes)
{
  std::unique_lock<std::mutex> lock(_mutex);
  _input_file_sizes[file_id] = size_bytes;
}

void storage_handler_t::set_compressor(compression_method_t method)
{
  _compressor = create_compressor(method);
}

void storage_handler_t::set_compression_level(int level)
{
  if (_compressor && _compressor->method() == compression_method_t::zstd)
  {
    static_cast<compressor_zstd_t *>(_compressor.get())->set_compression_level(level);
  }
}

#ifdef __EMSCRIPTEN__
// The cache tier is native-only (packed local file + spill/upload); wasm streams remotely.
dew_error_t storage_handler_t::configure_cache_tier(uint64_t, const std::string &, const std::string &)
{
  return {1, "cache tier is not available in the wasm build"};
}
void storage_handler_t::set_cache_max_bytes(uint64_t)
{
}
void storage_handler_t::ensure_dataset_uuid(uint8_t (&out)[16])
{
  memset(out, 0, sizeof(out));
}
dew_error_t storage_handler_t::run_spill_bootstrap()
{
  return {};
}
void storage_handler_t::set_clean_shutdown_next_checkpoint()
{
}
bool storage_handler_t::get_cache_tier_stats(cache_tier_stats_t &) const
{
  return false;
}
#else
dew_error_t storage_handler_t::configure_cache_tier(uint64_t cap_bytes, const std::string &destination_url, const std::string &connection)
{
  if (!_reader.backend() || !_reader.backend()->is_packed_file())
    return {1, "The cache tier requires a local cache file (packed storage)"};
  auto *packed = static_cast<packed_file_backend_t *>(_reader.backend());
  packed->enable_cache_tier(cap_bytes);
  if (!destination_url.empty())
  {
    auto io = vio::objstore::create_io_manager(destination_url, std::string_view(connection), _event_loop);
    if (!io.has_value())
      return {io.error().code != 0 ? io.error().code : -1, io.error().msg};
    packed->enable_spill(std::move(io.value()), "spill/");
  }
  return {};
}

void storage_handler_t::set_cache_max_bytes(uint64_t cap_bytes)
{
  _event_loop.run_in_loop([this, cap_bytes]() {
    if (!_reader.backend()->is_packed_file())
      return;
    auto *packed = static_cast<packed_file_backend_t *>(_reader.backend());
    if (packed->residency())
    {
      packed->residency()->set_cap(cap_bytes);
      packed->maybe_evict();
    }
  });
}

void storage_handler_t::ensure_dataset_uuid(uint8_t (&out)[16])
{
  auto *packed = static_cast<packed_file_backend_t *>(_reader.backend());
  const auto &current = packed->dataset_uuid();
  bool all_zero = true;
  for (auto b : current)
    all_zero = all_zero && b == 0;
  if (all_zero)
  {
    // Fresh dataset: mint a uuid tying this cache to its bucket generation. Written to the
    // superblock extras at the first checkpoint.
    std::random_device rd;
    uint8_t fresh[16];
    for (auto &b : fresh)
      b = uint8_t(rd());
    packed->set_dataset_uuid(fresh);
  }
  memcpy(out, packed->dataset_uuid(), sizeof(out));
}

dew_error_t storage_handler_t::run_spill_bootstrap()
{
  if (!_reader.backend()->is_packed_file())
    return {};
  auto *packed = static_cast<packed_file_backend_t *>(_reader.backend());
  std::promise<dew_error_t> done_promise;
  auto fut = done_promise.get_future();
  _event_loop.run_in_loop([packed, &done_promise]() -> vio::task_t<void> {
    return [](packed_file_backend_t *backend, std::promise<dew_error_t> &promise) -> vio::task_t<void> {
      promise.set_value(co_await backend->spill_bootstrap());
      co_return;
    }(packed, done_promise);
  });
  return fut.get();
}

void storage_handler_t::set_clean_shutdown_next_checkpoint()
{
  if (_reader.backend()->is_packed_file())
    static_cast<packed_file_backend_t *>(_reader.backend())->set_clean_shutdown_next_checkpoint();
}

bool storage_handler_t::get_cache_tier_stats(cache_tier_stats_t &out) const
{
  if (!_reader.backend() || !_reader.backend()->is_packed_file())
    return false;
  auto *packed = static_cast<packed_file_backend_t *>(const_cast<storage_backend_t *>(_reader.backend()));
  auto *residency = packed->residency();
  if (!residency)
    return false;
  out.resident_bytes = residency->resident_bytes();
  out.cap_bytes = residency->cap();
  out.tracked_blobs = residency->entry_count();
  out.spilled_bytes = packed->spill() ? packed->spill()->bytes_spilled() : 0;
  return true;
}
#endif // __EMSCRIPTEN__

// Best-effort: the subtract hops to the storage loop and rides whatever checkpoint commits next, so
// it is NOT atomic with the registry snapshot that recorded the collapse. In the common (no-crash)
// case the FIFO storage loop applies it before the collapse's own checkpoint serializes the stats, so
// the persisted numbers are consistent. An unclean crash between a mid-pass cache-pressure checkpoint
// and this subtract can leave the reported source total off by the freed chunks' bytes. This is
// cosmetic (the compression numbers in `dew info` only) and never affects stored data, so it is not
// worth making transactional (that would need the freed-source descriptors threaded through the
// checkpoint payload + a format change).
void storage_handler_t::note_source_blobs_freed(attributes_id_t attributes_id, std::vector<storage_location_t> locations, uint32_t point_count)
{
  _event_loop.run_in_loop([this, attributes_id, locations = std::move(locations), point_count]() {
    const auto &attrs = _attributes_configs.get(attributes_id);
    const auto formats = _attributes_configs.get_format_components(attributes_id);
    const auto count = std::min(locations.size(), formats.size());
    for (size_t i = 0; i < count && i < size_t(attrs.attributes.size()); i++)
    {
      if (locations[i].size == 0)
        continue;
      const auto &a = attrs.attributes[i];
      // Buffer 0 (the points/morton buffer) was written with a storage_header_t prepended, so its
      // recorded uncompressed size includes the header; match that here (see serialize_points).
      const uint32_t header_bytes = i == 0 ? uint32_t(sizeof(storage_header_t)) : 0;
      _compression_stats.subtract_source(std::string(a.name, a.name_size), formats[i], point_count, header_bytes, locations[i].size);
    }
  });
}

void storage_handler_t::note_blobs_uploaded(std::vector<std::pair<uint64_t, storage_location_t>> &&blobs)
{
  // Hop to the storage loop: the residency table is single-threaded there. remote_id encodes the
  // bucket data object as object_id << 32 (one blob per object; the low half stays zero).
  _event_loop.run_in_loop([this, blobs = std::move(blobs)]() {
    for (auto &[cache_offset, bucket_location] : blobs)
    {
      assert(bucket_location.offset == 0 && "dataset objects hold exactly one blob");
      const uint64_t remote_id = uint64_t(bucket_location.file_id) << 32;
      _reader.backend()->note_blob_uploaded(cache_offset, bucket_location.size, remote_id);
    }
    _reader.backend()->maybe_evict();
  });
}

void storage_handler_t::drain_posted_events()
{
  // Barrier: run_in_loop tasks execute FIFO per loop, so once this no-op runs, every previously
  // posted task (e.g. note_blobs_uploaded facts) has been applied. Used by the final quiesce so its
  // checkpoint serializes a residency table that already knows about the last band's uploads.
  std::promise<void> done;
  auto fut = done.get_future();
  _event_loop.run_in_loop([&done]() { done.set_value(); });
  fut.get();
}
} // namespace dew::converter
