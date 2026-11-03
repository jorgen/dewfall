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

#include <fmt/printf.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <condition_variable>
#include <functional>
#include <dew/converter/converter.h>

#include <vio/event_loop.h>
#include <vio/event_pipe.h>
#include <vio/thread_pool.h>

#include "attributes_configs.hpp"
#include "dataset_types.hpp"
#include "error.hpp"
#include "perf_stats.hpp"

namespace dew::converter
{
using namespace dew::core;
class storage_handler_t;
struct get_points_file_t
{
  input_data_id_t id;
  input_name_ref_t filename;
  dew_converter_file_convert_callbacks_t callbacks;
};

struct point_reader_file_t;
struct unsorted_points_event_t
{
  unsorted_points_event_t(std::vector<point_format_t> a_attributes_def, const dew_converter_header_t &a_public_header, points_t &&a_points, point_reader_file_t &a_reader_file)
    : attributes_def(a_attributes_def)
    , public_header(a_public_header)
    , points(std::move(a_points))
    , reader_file(&a_reader_file)
  {
  }

  std::vector<point_format_t> attributes_def;
  dew_converter_header_t public_header;
  points_t points;
  // A pointer, not a reference: vio's event_pipe_t moves events between its buffers by assignment.
  point_reader_file_t *reader_file;
};

class get_data_worker_t
{
public:
  get_data_worker_t(point_reader_file_t &point_reader_file, attributes_configs_t &attribute_configs, perf_stats_t &perf_stats, const get_points_file_t &file,
                    vio::event_pipe_t<std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t>> &input_init_pipe,
                    vio::event_pipe_t<input_data_id_t> &sub_added, vio::event_pipe_t<unsorted_points_event_t> &unsorted_points_queue);
  void work();
  void after_work();
  void enqueue(vio::event_loop_t &event_loop, vio::thread_pool_t &thread_pool);
  [[nodiscard]] bool done() const { return _done; }

  point_reader_file_t &point_reader_file;
  attributes_configs_t &attribute_configs;
  perf_stats_t &perf_stats;
  vio::event_pipe_t<std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t>> &input_init_pipe;
  vio::event_pipe_t<input_data_id_t> &sub_added;
  vio::event_pipe_t<unsorted_points_event_t> &unsorted_points_queue;
  std::unique_ptr<dew_error_t> error;
  get_points_file_t file;
  storage_header_t storage_header;
  uint64_t points_read;
  uint32_t split;
  bool _done{false};
};

class sort_worker_t
{
public:
  sort_worker_t(const tree_config_t &tree_config, point_reader_file_t &reader_file, attributes_configs_t &attributes_configs, perf_stats_t &perf_stats, dew_converter_header_t public_header, points_t &&points);
  void work();
  void after_work();
  void enqueue(vio::event_loop_t &event_loop, vio::thread_pool_t &thread_pool);
  [[nodiscard]] bool done() const { return _done; }

  tree_config_t _tree_config;
  point_reader_file_t &reader_file;
  attributes_configs_t &attributes_configs;
  perf_stats_t &perf_stats;
  dew_converter_header_t public_header;
  points_t points;
  dew_error_t error;
  bool _done{false};
};

struct point_reader_file_t
{
  point_reader_file_t(const tree_config_t &a_tree_config, vio::event_loop_t &a_event_loop, vio::thread_pool_t &a_thread_pool, attributes_configs_t &a_attributes_configs, perf_stats_t &a_perf_stats,
                      const get_points_file_t &file,
                      vio::event_pipe_t<std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t>> &input_init_pipe, vio::event_pipe_t<input_data_id_t> &sub_added, vio::event_pipe_t<unsorted_points_event_t> &unsorted_points,
                      vio::event_pipe_t<std::pair<points_t, dew_error_t>> &a_sorted_points_pipe)
    : tree_config(a_tree_config)
    , event_loop(a_event_loop)
    , thread_pool(a_thread_pool)
    , perf_stats(a_perf_stats)
    , input_reader(new get_data_worker_t(*this, a_attributes_configs, a_perf_stats, file, input_init_pipe, sub_added, unsorted_points))
    , sorted_points_pipe(a_sorted_points_pipe)
  {
    input_reader->enqueue(a_event_loop, a_thread_pool);
  }
  ~point_reader_file_t()
  {
  }

  tree_config_t tree_config;
  vio::event_loop_t &event_loop;
  vio::thread_pool_t &thread_pool;
  perf_stats_t &perf_stats;
  std::unique_ptr<get_data_worker_t> input_reader;
  std::vector<std::unique_ptr<sort_worker_t>> sort_workers;
  vio::event_pipe_t<std::pair<points_t, dew_error_t>> &sorted_points_pipe;
  uint32_t input_split = 0;
  uint32_t sort_done = 0;
};

class memory_requester_t
{
  memory_requester_t(uint64_t available_memory)
    : _available_memory(available_memory)
  {
  }

  void request_memory(uint64_t memory)
  {
    std::unique_lock<std::mutex> lock(_mutex);
    uint64_t acquired_memory = 0;

    _wait_condition.wait(lock, [memory, &acquired_memory, this]() {
      acquired_memory += std::min(memory - acquired_memory, _available_memory);
      return acquired_memory == memory;
    });
    return;
  }

  void release_memory(uint64_t memory)
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _available_memory += memory;
    _wait_condition.notify_all();
  }

private:
  uint64_t _available_memory;
  std::mutex _mutex;
  std::condition_variable _wait_condition;
};

class point_reader_t : public vio::about_to_block_t
{
public:
  point_reader_t(vio::event_loop_t &event_loop, vio::thread_pool_t &thread_pool, attributes_configs_t &attributes_configs, perf_stats_t &perf_stats,
                 vio::event_pipe_t<std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t>> &input_init_pipe, vio::event_pipe_t<input_data_id_t> &sub_added,
                 vio::event_pipe_t<std::pair<points_t, dew_error_t>> &sorted_points_pipe, vio::event_pipe_t<input_data_id_t> &done_with_file, vio::event_pipe_t<file_error_t> &file_errors);
  void add_file(tree_config_t tree_config, get_points_file_t &&new_file);

  // Teardown barrier: after this returns, this reader will never enqueue onto the shared thread pool
  // again, so the pool may be joined. Mirrors tree_handler_t::begin_shutdown -- both loops outlive the
  // pool join in processor_t's ordered teardown, and enqueue-after-stop is a bare abort().
  void begin_shutdown();

  void about_to_block() override;

private:
  void handle_new_files(tree_config_t &&tree_config, get_points_file_t &&new_file);
  void handle_unsorted_points(unsorted_points_event_t &&unsorted_points);

  vio::event_loop_t &_event_loop;
  vio::thread_pool_t &_thread_pool;
  attributes_configs_t &_attributes_configs;
  perf_stats_t &_perf_stats;
  vio::event_pipe_t<std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t>> &_input_init_pipe;
  vio::event_pipe_t<input_data_id_t> &_sub_added;
  vio::event_pipe_t<std::pair<points_t, dew_error_t>> &_sorted_points_pipe;
  vio::event_pipe_t<input_data_id_t> &_done_with_file;
  vio::event_pipe_t<file_error_t> &_file_errors;
  vio::event_pipe_t<tree_config_t, get_points_file_t> _new_files_pipe;
  vio::event_pipe_t<unsorted_points_event_t> _unsorted_points;
  std::vector<std::unique_ptr<point_reader_file_t>> _point_reader_files;
  std::atomic_bool _shutting_down = false;
};
} // namespace dew::converter
