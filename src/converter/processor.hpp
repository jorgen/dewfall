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

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#ifndef __EMSCRIPTEN__
#include <uv.h> // unused here; pulls libuv, which the wasm build does not link
#endif
#include <vector>

#include <dew/converter/converter.h>

#include <vio/event_loop.h>
#include <vio/event_pipe.h>
#include <vio/operation/work.h>
#include <vio/task.h>
#include <vio/thread_pool.h>

#include "attributes_configs.hpp"
#include "attribute_amend.hpp"
#include "dataset_types.hpp"
#include "frustum_tree_walker.hpp"
#include "input_data_source_registry.hpp"
#include "perf_stats.hpp"
#include "pre_init_file_retriever.hpp"
#include "reader.hpp"
#include "storage_handler.hpp"
#include "tree_handler.hpp"

namespace dew::converter
{
using namespace dew::core;
enum class processor_open_file_semantics_t
{
  read,
  write,
  write_truncate,
};

enum class file_existence_requirement_t
{
  exist,
  not_exist,
  can_exist,
};

// Destination mode: convert into a local cache file (the primary `url`) while finalized subtrees
// upload incrementally to `url`'s bucket in the DEW2 layout. Empty url = classic local-only mode.
struct destination_config_t
{
  std::string url;               // s3:// az:// dir:// mem://; empty = no destination
  std::string connection;        // provider connection string (may be empty: env credentials)
  uint64_t cache_max_bytes = 0;  // resident-bytes cap for the cache file; 0 = unlimited
};

class frustum_tree_walker_t;
class processor_t : public vio::about_to_block_t
{
public:
  processor_t(std::string url, file_existence_requirement_t existence_requirement, dew_error_t &error, const destination_config_t &destination = {});
  ~processor_t();
  dew_error_t upgrade_to_write(bool truncate);
  void set_pre_init_tree_config(const tree_config_t &tree_config);
  void set_tree_scale_override(double scale);
  // Mutable mode. Legal at any time, including on a reopened dataset -- see tree_handler_t::set_mutable.
  void set_mutable(bool value)
  {
    _tree_handler.set_mutable(value);
  }
  [[nodiscard]] bool is_mutable()
  {
    return _tree_handler.is_mutable();
  }
  tree_config_t tree_config_peek()
  {
    return _tree_handler.tree_config_peek();
  }
  void set_pre_init_node_point_limit(uint32_t node_point_limit);
  void set_pre_init_read_chunk_bytes(uint64_t bytes);
  void set_runtime_callbacks(const dew_converter_runtime_callbacks_t &runtime_callbacks, void *user_ptr);
  void set_converter_callbacks(const dew_converter_file_convert_callbacks_t &convert_callbacks);
  void add_files(std::vector<std::pair<std::unique_ptr<char[]>, uint32_t>> &&input_files);
  void walk_tree(frustum_tree_walker_t &walker);
  tree_config_t tree_config();
  void request_aabb(std::function<void(double[3], double[3])> callback);
  uint32_t attrib_name_registry_count();
  uint32_t attrib_name_registry_get(uint32_t index, char *name, uint32_t buffer_size);

  // Full quiesce: conversion pipeline idle AND (destination mode) the uploader drained/parked --
  // after this the dataset is durable at the destination (unless parked on errors).
  void wait_idle();
  // Amending a mutable dataset with a new attribute (see attribute_amend.hpp and the public API
  // comment in converter.h). declare/add_data only buffer; commit does the work and blocks.
  bool declare_attribute(const std::string &name, const std::string &key_attribute, dew_type_t type, dew_components_t components);
  bool add_attribute_data(const std::string &name, const uint64_t *keys, const void *values, uint64_t count);
  void commit_attributes();

  // End mutable mode: seal the dataset and, in destination mode, ship it. Blocks until done.
  void finalize();
  // Conversion-only quiesce: the cache file is a complete valid DEW; uploads may still be running.
  void wait_local_complete();
  bool upload_active() const;
  upload_stats_t upload_stats() const;
  bool is_idle()
  {
    std::unique_lock<std::mutex> lock(_idle_mutex);
    return _idle;
  }
  bool configuration_initialized()
  {
    return _tree_handler.configuration_initialized();
  }

  // Source-scale adoption state (first pre-init batch may set the octree scale from the input files).
  bool _warned_missing_aabb_min = false;
  bool _tree_scale_explicit = false;
  double _source_scale_adopted = 0.0;

  void about_to_block() override;

  bool has_errors() const
  {
    return _has_errors;
  }

  storage_handler_t &storage_handler()
  {
    return _storage_handler;
  }

  perf_stats_t &perf_stats()
  {
    return _perf_stats;
  }

  const dew_attributes_t &get_attributes(attributes_id_t id);

private:
  std::string _url;
  vio::thread_pool_t _thread_pool;
  dew_converter_runtime_callbacks_t _runtime_callbacks;
  void *_runtime_callback_user_ptr;
  dew_converter_file_convert_callbacks_t _convert_callbacks;

  vio::thread_with_event_loop_t _thread_with_event_loop;
  vio::event_loop_t &_event_loop;

  bool _generating_lod;
  bool _has_errors;
  // Set on _event_loop during teardown, before the shared pool is joined: the pre-init coroutine
  // launched from handle_new_files schedules work onto that pool, and enqueue-after-stop aborts.
  std::atomic_bool _shutting_down = false;
  morton::morton192_t _lod_done_morton = {};
  morton::morton192_t _current_lod_target_morton = {};

  bool _idle;
  int _new_file_events_sent;
  std::mutex _idle_mutex;
  std::condition_variable _idle_condition;

  perf_stats_t _perf_stats;
  storage_handler_t _storage_handler;
  input_data_source_registry_t _input_data_source_registry;
  attributes_configs_t _attributes_configs;
  attribute_amend_t _attribute_amend;
  tree_handler_t _tree_handler;
  // Destination mode (native only; null in classic local-only conversions).
  destination_config_t _destination;
#ifndef __EMSCRIPTEN__
  std::unique_ptr<upload_handler_t> _upload_handler;
#endif
  dew_converter_upload_callbacks_t _upload_callbacks = {};
  void *_upload_callback_user_ptr = nullptr;

public:
  void set_upload_callbacks(const dew_converter_upload_callbacks_t &callbacks, void *user_ptr)
  {
    _upload_callbacks = callbacks;
    _upload_callback_user_ptr = user_ptr;
  }
  void set_cache_max_bytes(uint64_t cap_bytes)
  {
    _storage_handler.set_cache_max_bytes(cap_bytes);
  }

private:

  vio::event_pipe_t<std::vector<std::pair<std::unique_ptr<char[]>, uint32_t>>> _files_added;


  vio::event_pipe_t<std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t>> _input_init;
  vio::event_pipe_t<input_data_id_t> _sub_added;
  vio::event_pipe_t<std::pair<points_t, dew_error_t>> _sorted_points;
  vio::event_pipe_t<file_error_t> _point_reader_file_errors;
  vio::event_pipe_t<input_data_id_t> _point_reader_done_with_file;

  vio::event_pipe_t<void> _storage_index_write_done;
  vio::event_pipe_t<dew_error_t> _storage_handler_error;
  vio::event_pipe_t<input_data_id_t> _tree_done_with_input;

  vio::thread_with_event_loop_t _input_event_loop_thread;
  vio::event_loop_t &_input_event_loop;
  point_reader_t _point_reader;

  int64_t _read_sort_budget;
  int64_t _read_sort_active_approximate_size;

  std::unique_ptr<attribute_index_map_t> _attribute_index_map;
  std::vector<std::string> _cached_attribute_names;

  void handle_new_files(std::vector<std::pair<std::unique_ptr<char[]>, uint32_t>> &&new_files);
  vio::task_t<void> do_handle_new_files(std::vector<std::pair<input_data_id_t, input_name_ref_t>> file_refs, tree_config_t tree_config);
  void handle_input_init_done(std::tuple<input_data_id_t, attributes_id_t, dew_converter_header_t> &&event);
  void handle_sub_added(input_data_id_t &&event);
  void handle_sorted_points(std::pair<points_t, dew_error_t> &&event);
  void handle_file_errors(file_error_t &&error);
  void handle_file_reading_done(input_data_id_t &&file);
  void handle_index_write_done();
  void handle_storage_error(dew_error_t &&errors);
  void handle_points_written(const storage_header_t &header, attributes_id_t attributes, std::vector<storage_location_t> &&locations);
  void handle_tree_done_with_input(input_data_id_t &&events);
  void maybe_start_lod();
};
} // namespace dew::converter
