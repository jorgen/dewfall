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
#include "reader.hpp"

#include <algorithm>

#include "input_header.hpp"
#include "loop_quiesce.hpp"
#include "morton.hpp"
#include "sorter.hpp"

#include <fmt/printf.h>

#include <dew/core/default_attribute_names.h>

#include <assert.h>
#include <chrono>

namespace dew::converter
{
using namespace dew::core;
get_data_worker_t::get_data_worker_t(point_reader_file_t &a_point_reader_file, attributes_configs_t &a_attribute_configs, perf_stats_t &a_perf_stats, const get_points_file_t &a_file,
                                     vio::event_pipe_t<std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t>> &a_input_init_pipe, vio::event_pipe_t<input_data_id_t> &a_sub_added,
                                     vio::event_pipe_t<unsorted_points_event_t> &a_unsorted_points_queue)
  : point_reader_file(a_point_reader_file)
  , attribute_configs(a_attribute_configs)
  , perf_stats(a_perf_stats)
  , input_init_pipe(a_input_init_pipe)
  , sub_added(a_sub_added)
  , unsorted_points_queue(a_unsorted_points_queue)
  , file(a_file)
  , points_read(0)
  , split(0)
{
}

struct callback_closer
{
  callback_closer(dew_converter_file_convert_callbacks_t &a_callbacks, void *a_user_ptr)
    : callbacks(a_callbacks)
    , user_ptr(a_user_ptr)
  {
  }
  ~callback_closer()
  {
    if (callbacks.destroy_user_ptr && user_ptr)
    {
      callbacks.destroy_user_ptr(user_ptr);
    }
  }

  dew_converter_file_convert_callbacks_t &callbacks;
  void *user_ptr;
};

void get_data_worker_t::work()
{
  storage_header_initialize(storage_header);
  // Stamp the input id BEFORE anything can fail: point_reader_t posts
  // storage_header.input_id on _done_with_file for every finished file,
  // including the error returns below, and the processor looks that id up in
  // the input registry. Leaving it uninitialized until after the init callback
  // made any init-stage error (a failing user callback, or the xyz-first
  // check) abort the process on a garbage registry lookup.
  storage_header.input_id = file.id;
  dew_attributes_t tmp_attributes;
  dew_error_t *local_error = nullptr;
  // The init callback is expected to fill this, but it may fail before doing
  // so and callback_closer hands it to destroy_user_ptr regardless.
  void *user_ptr = nullptr;
  dew_converter_header_t public_header;
  file.callbacks.init(file.filename.name, file.filename.name_length, &public_header, &tmp_attributes, &user_ptr, &local_error);
  callback_closer closer(file.callbacks, user_ptr);
  if (local_error)
  {
    error.reset(local_error);
    return;
  }

  if (tmp_attributes.attributes[0].name_size != strlen(DEW_ATTRIBUTE_XYZ) || memcmp(tmp_attributes.attributes[0].name, DEW_ATTRIBUTE_XYZ, tmp_attributes.attributes[0].name_size) != 0)
  {
    error.reset(new dew_error_t());
    error->code = -1;
    error->msg = "First attribute has to be " DEW_ATTRIBUTE_XYZ;
    return;
  }

  attributes_id_t attributes_id = attribute_configs.get_attribute_config_index(std::move(tmp_attributes));
  auto &attributes = attribute_configs.get(attributes_id);
  auto attribute_info = attribute_configs.get_format_components(attributes_id);
  input_init_pipe.post_event(std::make_tuple(storage_header.input_id, attributes_id, public_header));

  // Read/sort chunk size targets read_chunk_byte_target bytes (default 64 MiB): big chunks
  // amortize source reads and the morton sort; the octree still subdivides them into
  // node_point_limit leaves, and leaf collapse rewrites final leaves into per-node units. Never
  // below node_point_limit (a chunk should fill at least one node), capped so per-chunk memory
  // spikes stay bounded.
  uint64_t bytes_per_point = 0;
  for (auto &format : attribute_info)
    bytes_per_point += uint64_t(size_for_format(format.type, format.components));
  uint64_t target_points = point_reader_file.tree_config.read_chunk_byte_target / (bytes_per_point ? bytes_per_point : 1);
  uint32_t convert_size = uint32_t(std::clamp<uint64_t>(target_points, point_reader_file.tree_config.node_point_limit, k_default_max_chunk_points));
  uint8_t done_read_file = false;
  uint32_t local_points_read;
  uint32_t sub_part = 0;
  while (!done_read_file)
  {
    auto batch_start = std::chrono::steady_clock::now();
    points_t points;
    points.header = storage_header;
    points.header.input_id.sub = sub_part++;
    assert(points.header.input_id.sub < (1u << 30) && "reader sub ids must stay clear of the collapsed-leaf id bits");
    points.header.point_count = convert_size;
    points.attributes_id = attributes_id;
    attribute_buffers_initialize(attribute_info, points.buffers, convert_size);
    file.callbacks.convert_data(user_ptr, &public_header, attributes.attributes.data(), uint32_t(attributes.attributes.size()), convert_size, points.buffers.buffers.data(), uint32_t(points.buffers.buffers.size()),
                                &local_points_read, &done_read_file, &local_error);
    if (local_error)
    {
      error.reset(local_error);
      return;
    }
    auto input_to_send = points.header.input_id;
    sub_added.post_event(std::move(input_to_send));
    attribute_buffers_adjust_buffers_to_size(attribute_info, points.buffers, local_points_read);
    points_read += local_points_read;
    points.header.point_count = local_points_read;

    uint64_t batch_bytes = 0;
    for (auto &buf : points.buffers.buffers)
      batch_bytes += buf.size;

    auto batch_end = std::chrono::steady_clock::now();
    auto batch_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(batch_end - batch_start).count());
    perf_stats.source_read.record(batch_bytes, batch_us);

    split++;
    unsorted_points_event_t event(attribute_info, public_header, std::move(points), point_reader_file);
    unsorted_points_queue.post_event(std::move(event));
  }
}

void get_data_worker_t::after_work()
{
  point_reader_file.input_split = split;
}

void get_data_worker_t::enqueue(vio::event_loop_t &event_loop, vio::thread_pool_t &thread_pool)
{
  thread_pool.enqueue([this, &event_loop] {
    this->work();
    event_loop.run_in_loop([this] {
      this->_done = true;
      this->after_work();
    });
  });
}

sort_worker_t::sort_worker_t(const tree_config_t &a_tree_config, point_reader_file_t &a_reader_file, attributes_configs_t &a_attributes_configs, perf_stats_t &a_perf_stats, dew_converter_header_t a_public_header, points_t &&a_points)
  : _tree_config(a_tree_config)
  , reader_file(a_reader_file)
  , attributes_configs(a_attributes_configs)
  , perf_stats(a_perf_stats)
  , public_header(a_public_header)
  , points(std::move(a_points))
{
}

void sort_worker_t::work()
{
  auto sort_start = std::chrono::steady_clock::now();
  sort_points(_tree_config, attributes_configs, public_header, points, error, _tree_config.store_original_order);
  auto sort_end = std::chrono::steady_clock::now();

  uint64_t sort_bytes = 0;
  for (auto &buf : points.buffers.buffers)
    sort_bytes += buf.size;

  auto sort_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(sort_end - sort_start).count());
  perf_stats.sort.record(sort_bytes, sort_us);
}

void sort_worker_t::after_work()
{
  reader_file.sort_done++;
  reader_file.sorted_points_pipe.post_event(std::make_pair(std::move(points), std::move(error)));
}

void sort_worker_t::enqueue(vio::event_loop_t &event_loop, vio::thread_pool_t &thread_pool)
{
  thread_pool.enqueue([this, &event_loop] {
    this->work();
    event_loop.run_in_loop([this] {
      this->_done = true;
      this->after_work();
    });
  });
}

point_reader_t::point_reader_t(vio::event_loop_t &event_loop, vio::thread_pool_t &thread_pool, attributes_configs_t &attributes_configs, perf_stats_t &perf_stats,
                               vio::event_pipe_t<std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t>> &input_init_pipe,
                               vio::event_pipe_t<input_data_id_t> &sub_added, vio::event_pipe_t<std::pair<points_t, dew_error_t>> &sorted_points_pipe, vio::event_pipe_t<input_data_id_t> &done_with_file,
                               vio::event_pipe_t<file_error_t> &file_errors)
  : _event_loop(event_loop)
  , _thread_pool(thread_pool)
  , _attributes_configs(attributes_configs)
  , _perf_stats(perf_stats)
  , _input_init_pipe(input_init_pipe)
  , _sub_added(sub_added)
  , _sorted_points_pipe(sorted_points_pipe)
  , _done_with_file(done_with_file)
  , _file_errors(file_errors)
  , _new_files_pipe(event_loop, bind(&point_reader_t::handle_new_files))
  , _unsorted_points(event_loop, bind(&point_reader_t::handle_unsorted_points))
{
  event_loop.add_about_to_block_listener(this);
}

void point_reader_t::add_file(tree_config_t tree_config, get_points_file_t &&new_file)
{
  _new_files_pipe.post_event(std::move(tree_config), std::move(new_file));
}

void point_reader_t::about_to_block()
{
  auto finished = std::partition(_point_reader_files.begin(), _point_reader_files.end(), [](const std::unique_ptr<point_reader_file_t> &a) { return !a->input_reader->done() || a->input_split != a->sort_done; });
  for (auto it = finished; it != _point_reader_files.end(); ++it)
  {
    auto &input_reader = it->get()->input_reader;
    assert(input_reader->done());
    if (input_reader->error)
    {
      file_error_t file_error;
      file_error.error = std::move(*input_reader->error);
      file_error.input_id = input_reader->file.id;
      _file_errors.post_event(std::move(file_error));
    }

    auto to_send = it->get()->input_reader->storage_header.input_id;
    _done_with_file.post_event(std::move(to_send));
  }
  _point_reader_files.erase(finished, _point_reader_files.end());
}

void point_reader_t::begin_shutdown()
{
  // Flip the flag ON the input loop and wait for it, so that after this returns no callback on that
  // loop can still reach thread_pool.enqueue. Bounded -- see loop_quiesce.hpp; on timeout set it
  // directly and accept the narrow race rather than deadlocking the destructor.
  if (!core::run_on_loop_and_wait(_event_loop, [this]() { _shutting_down.store(true, std::memory_order_release); }))
    _shutting_down.store(true, std::memory_order_release);
}

void point_reader_t::handle_new_files(tree_config_t &&tree_config, get_points_file_t &&new_file)
{
  // point_reader_file_t's constructor enqueues its get_data_worker onto the shared pool, so during
  // teardown it must not be built at all.
  if (_shutting_down.load(std::memory_order_acquire))
    return;
  _point_reader_files.emplace_back(new point_reader_file_t(tree_config, _event_loop, _thread_pool, _attributes_configs, _perf_stats, new_file, _input_init_pipe, _sub_added, _unsorted_points, _sorted_points_pipe));
}

void point_reader_t::handle_unsorted_points(unsorted_points_event_t &&unsorted_points)
{
  // Same reason: the sort worker below goes onto the shared pool.
  if (_shutting_down.load(std::memory_order_acquire))
    return;
  auto &reader_file = *unsorted_points.reader_file;
  auto &tree_config = reader_file.tree_config;
  reader_file.sort_workers.emplace_back(new sort_worker_t(tree_config, reader_file, _attributes_configs, _perf_stats, unsorted_points.public_header, std::move(unsorted_points.points)));
  reader_file.sort_workers.back()->enqueue(reader_file.event_loop, reader_file.thread_pool);
}

} // namespace dew::converter
