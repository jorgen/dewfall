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
#pragma once

#include <dew/access/query.h>

#include "attributes_configs.hpp"
#include "blob_reader.hpp"
#include "budget.hpp"
#include "decode.hpp"
#include "morton_tree_coordinate_transform.hpp"
#include "pump.hpp"
#include "region_walk.hpp"
#include "tree.hpp"
#include "tree_set.hpp"

#include <atomic>
#include <memory>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

namespace dew::access
{
using namespace dew::core;

struct request_impl_t;

// A region request's parameters, COPIED out of the caller's dew_region_request_t.
//
// Not a pointer to the caller's struct: the request now outlives dew_dataset_request_region, and the
// attribute_names array it points at belongs to the caller, who is free to destroy it the moment the
// call returns.
struct region_job_t
{
  double box_min[3] = {0, 0, 0};
  double box_max[3] = {0, 0, 0};
  dew_lod_mode_t lod_mode = dew_lod_full;
  int32_t lod = 0;
  uint64_t max_points = 0;
  std::vector<std::string> attribute_names;
  dew_position_format_t position_format = dew_position_r64_absolute;
  dew_clip_mode_t clip_mode = dew_clip_point;
};


struct dataset_impl_t
{
  dataset_impl_t(std::string url, std::string connection, const dew_dataset_options_t &options, dew_pump_t *shared_pump);
  ~dataset_impl_t();

  void on_storage_error(const dew_error_t &&e);
  // Nothing on the query path may block: under wasm a blocking read stalls the whole program, and
  // natively it would tie up the dataset loop that other requests run on. Tree loading itself lives
  // in dew::core::tree_set_t, shared with the renderer's frame-driven path.
  vio::task_t<bool> co_walk_to_convergence(const region_query_t &query, region_result_t &out);
  // The deferred open: index, attribute configs, tree registry, root tree. Ends in ready or error.
  vio::task_t<void> co_open();
  // Spawn a region request on the dataset's own loop. Returns immediately; the request reaches a
  // terminal status later and is published through the pump.
  void spawn_region_request(region_job_t job, std::shared_ptr<struct dew_request_t> request);
  void info(dew_dataset_info_t &out) const;

  // Queue a finished request for delivery and raise the pump. Called from whichever thread completed
  // the work; the callback itself runs later, on the host thread, from the pump's drain.
  void publish(const std::shared_ptr<struct dew_request_t> &request);
  // pump_source_t hooks. Both run on the host thread from dew_pump_poll.
  static uint32_t drain_fn(void *ctx);
  static uint32_t pending_fn(void *ctx);

  std::string url;
  std::string connection;
  vio::thread_pool_t pool;
  vio::thread_with_event_loop_t loop_thread;
  vio::event_pipe_t<dew_error_t> storage_error;
  perf_stats_t perf;
  derived_budgets_t budgets;
  uint32_t max_reads_in_flight = 16;
  std::unique_ptr<blob_reader_t> reader;
  attributes_configs_t attributes;
  std::unique_ptr<tree_set_t> trees;

  [[nodiscard]] const tree_registry_t &registry() const { return trees->registry(); }
  [[nodiscard]] tree_registry_t &registry() { return trees->registry(); }
  size_t registry_size_at_open = 0;

  dew_error_t error;
  std::atomic<dew_dataset_state_t> state{dew_dataset_opening};
  std::mutex state_mutex;
  std::condition_variable state_cond;
  // What dew_dataset_wait_ready waits on: `state`, but set only after set_state fired the pump for it.
  dew_dataset_state_t announced_state = dew_dataset_opening;
  void set_state(dew_dataset_state_t next);
  std::mutex mutex;
  // shared_ptr so an in-flight request survives dew_request_release: the caller is done with the
  // handle, but the work may still be running and must not be destroyed under itself.
  std::vector<std::shared_ptr<struct dew_request_t>> requests;

  dew_pump_t *pump = nullptr;
  bool owns_pump = false;
  std::mutex dispatch_mutex;
  std::vector<std::shared_ptr<struct dew_request_t>> awaiting_dispatch;
};

// One attribute's concatenated output buffer.
struct out_buffer_t
{
  std::string name;
  dew_type_t type = dew_type_u8;
  dew_components_t components = dew_components_1;
  uint32_t stride = 0;
  std::vector<uint8_t> data;
};

struct request_impl_t
{
  void cancel();
  // Mark terminal and wake anyone in dew_request_wait. Called once, from the thread that finished
  // the work.
  void finish(dew_request_status_t terminal);
  // Exactly-once guard shared by the pump drain and dew_request_wait -- either may get there first.
  bool claim_callback() { return !callback_fired.exchange(true, std::memory_order_acq_rel); }

  dataset_impl_t *dataset = nullptr;
  std::atomic<dew_request_status_t> status{dew_request_pending};
  dew_error_t error;
  dew_request_done_callback_t done = nullptr;
  void *done_user_ptr = nullptr;
  std::atomic<bool> callback_fired{false};
  std::mutex wait_mutex;
  std::condition_variable wait_cond;

  std::vector<out_buffer_t> buffers;
  std::vector<dew_result_node_t> nodes;
  uint64_t point_count = 0;

  // Views handed out by dew_request_get_result; kept alive by `buffers` until release.
  std::vector<dew_attribute_buffer_t> buffer_views;
};

} // namespace dew::access

// The public handles. Defined here rather than in query_api.cpp so that dataset.cpp can dispatch
// through them -- the drain has to see a complete dew_request_t.
struct dew_dataset_t : dew::access::dataset_impl_t
{
  using dew::access::dataset_impl_t::dataset_impl_t;
};

struct dew_request_t : dew::access::request_impl_t
{
};
