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

#include "dataset_impl.hpp"

#include "budget.hpp"
#include "loop_quiesce.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

namespace dew::access
{

dataset_impl_t::dataset_impl_t(std::string a_url, std::string a_connection, const dew_dataset_options_t &options, dew_pump_t *shared_pump)
  : url(std::move(a_url))
  , connection(std::move(a_connection))
  , pool(options.decode_threads ? options.decode_threads : std::max(2u, std::thread::hardware_concurrency() / 2))
  , storage_error(loop_thread.event_loop(), vio::event_bind_t::bind(*this, &dataset_impl_t::on_storage_error))
{
  // A shared pump lets one wake drive several subsystems; without one the dataset keeps a private
  // pump so dew_dataset_poll works with no ceremony.
  pump = shared_pump;
  owns_pump = pump == nullptr;
  if (owns_pump)
    pump = dew_pump_create();
  pump_register(pump, pump_source_t{this, &dataset_impl_t::drain_fn, &dataset_impl_t::pending_fn});

  const uint64_t budget = options.memory_budget_bytes ? options.memory_budget_bytes : (uint64_t(512) << 20);
  budgets = derive_budgets(budget);
  // How many blob reads may be in flight at once. This is what turns a query over a high-latency
  // store from N round trips into roughly N/max_reads_in_flight.
  max_reads_in_flight = options.max_reads_in_flight ? options.max_reads_in_flight : uint32_t(std::max(1, budgets.io_clamp));

  reader = std::make_unique<blob_reader_t>(url, connection, pool, perf, storage_error, error);
  if (error.code != 0)
  {
    // set_state, never a bare store: it notifies state_cond. A plain store leaves anyone in
    // dew_dataset_wait_ready blocked forever on a dataset that has already failed.
    set_state(dew_dataset_error);
    return;
  }
  reader->set_read_cache_size(budgets.read_cache_bytes);
  reader->set_decompressed_cache_size(budgets.decompressed_cache_bytes);
  // Loads run on the dataset's own loop, which is also where the walks happen -- that pairing is what
  // makes tree_set_t's residency checks lock-free.
  trees = std::make_unique<tree_set_t>(*reader, loop_thread.event_loop());

  // Everything past this point is deferred. dew_dataset_create returns with the dataset `opening`;
  // the existence probe, the index read, the registry and the root tree all happen on the dataset's
  // own loop. Constructing the backend does no IO -- object_backend_t's probe is lazy precisely so
  // that this constructor cannot block. src/wasm/access_noasyncify_probe.cpp is what holds that.
  loop_thread.event_loop().run_in_loop([this]() {
    [](dataset_impl_t *self) -> vio::detached_task_t { co_await self->co_open(); }(this);
  });
}

void dataset_impl_t::set_state(dew_dataset_state_t next)
{
  {
    std::unique_lock<std::mutex> lock(state_mutex);
    state.store(next, std::memory_order_release);
  }
  state_cond.notify_all();
  // A state change is something the host may be waiting to hear about, so it wakes the pump too.
  pump_fire(pump);
}

vio::task_t<void> dataset_impl_t::co_open()
{
  // Await the existence probe before asking for its answer. file_exists() is a plain getter, but on an
  // object store the answer costs a HEAD, and reaching it through the blocking path would stall the
  // dataset loop -- under wasm, fatally: the loop is cooperative and there is no ASYNCIFY to unwind.
  error = co_await reader->probe_exists_async(loop_thread.event_loop());
  if (error.code != 0)
  {
    set_state(dew_dataset_error);
    co_return;
  }
  if (!reader->file_exists())
  {
    error = {1, "dataset does not exist: " + reader->file_exists_error()};
    set_state(dew_dataset_error);
    co_return;
  }

  index_load_t load;
  error = co_await reader->read_index_async(load, loop_thread.event_loop());
  if (error.code != 0)
  {
    set_state(dew_dataset_error);
    co_return;
  }
  if (load.attribute_configs && load.attribute_configs_size > 0)
  {
    error = attributes.deserialize(load.attribute_configs, load.attribute_configs_size);
    if (error.code != 0)
    {
      set_state(dew_dataset_error);
      co_return;
    }
  }
  if (!load.tree_registry || load.tree_registry_size == 0)
  {
    error = {1, "dataset has no tree registry"};
    set_state(dew_dataset_error);
    co_return;
  }
  error = trees->initialize(load.tree_registry, load.tree_registry_size);
  if (error.code != 0)
  {
    set_state(dew_dataset_error);
    co_return;
  }
  // The registry is fully sized here and the read path never grows it. Every lock-free reader --
  // and the synchronous node accessors -- depend on that invariant holding.
  registry_size_at_open = registry().data.size();

  if (!co_await trees->load(trees->root(), error))
  {
    set_state(dew_dataset_error);
    co_return;
  }
  set_state(dew_dataset_ready);
}

dataset_impl_t::~dataset_impl_t()
{
  // Ordered teardown, same discipline as processor.cpp: stop being drained, cancel outstanding work,
  // then join the pool and the loop -- the reader's loop is still needed while in-flight reads
  // unwind, so it goes last.
  pump_unregister(pump, this);
  // Stop starting new tree loads before anything is torn down; the ones already in flight still
  // complete against a live reader, which stop_loop below waits out.
  if (trees)
    trees->begin_shutdown();
  for (auto &request : requests)
    request->cancel();
  requests.clear();
  {
    std::unique_lock<std::mutex> lock(dispatch_mutex);
    awaiting_dispatch.clear();
  }
  pool.join();
  if (reader)
    reader->stop_loop();
  // co_open and every tree load resume on THIS loop, and stop_loop only guarantees that those resumes
  // have been POSTED -- a dataset destroyed while still opening still has one queued here. Drain it
  // (bounded, loop_quiesce.hpp) and then join the loop, so nothing runs on it after this point: the
  // pump destroyed just below, and the storage_error pipe destroyed with the members, are both things
  // a late callback would touch.
  (void)run_on_loop_and_wait(loop_thread.event_loop(), []() {});
  loop_thread.stop_and_join();
  if (owns_pump)
    dew_pump_destroy(pump);
}

void dataset_impl_t::publish(const std::shared_ptr<dew_request_t> &request)
{
  {
    std::unique_lock<std::mutex> lock(dispatch_mutex);
    awaiting_dispatch.push_back(request);
  }
  pump_fire(pump);
}

uint32_t dataset_impl_t::drain_fn(void *ctx)
{
  auto *self = static_cast<dataset_impl_t *>(ctx);
  std::vector<std::shared_ptr<dew_request_t>> batch;
  {
    std::unique_lock<std::mutex> lock(self->dispatch_mutex);
    batch.swap(self->awaiting_dispatch);
  }
  uint32_t dispatched = 0;
  for (auto &request : batch)
  {
    dispatched++;
    // claim_callback is the exactly-once guard: dew_request_wait may already have delivered this one
    // from the waiting thread.
    if (request->claim_callback() && request->done)
      request->done(request.get(), request->status.load(std::memory_order_acquire), request->done_user_ptr);
  }
  return dispatched;
}

uint32_t dataset_impl_t::pending_fn(void *ctx)
{
  auto *self = static_cast<dataset_impl_t *>(ctx);
  std::unique_lock<std::mutex> lock(self->dispatch_mutex);
  return uint32_t(self->awaiting_dispatch.size());
}

void dataset_impl_t::on_storage_error(const dew_error_t &&e)
{
  std::unique_lock<std::mutex> lock(mutex);
  if (error.code == 0)
    error = e;
}

// Walk, load whatever sub-trees the walk asked for, walk again. The walk itself never reads storage,
// so this loop is the only place a region query blocks.
vio::task_t<bool> dataset_impl_t::co_walk_to_convergence(const region_query_t &query, region_result_t &out)
{
  constexpr int max_rounds = 64;
  for (int round = 0; round < max_rounds; round++)
  {
    region_walk(registry(), query, out);
    if (out.trees_to_load.empty())
      co_return true;
    for (auto id : out.trees_to_load)
    {
      if (!co_await trees->load(id, error))
        co_return false;
    }
  }
  error = {1, "region walk did not converge"};
  co_return false;
}

void dataset_impl_t::info(dew_dataset_info_t &out) const
{
  memset(&out, 0, sizeof(out));
  out.scale = registry().tree_config.scale;
  for (int i = 0; i < 3; i++)
    out.offset[i] = registry().tree_config.offset[i];
  out.node_point_limit = registry().tree_config.node_point_limit;
  out.attribute_count = attributes.attrib_name_registry_count();

  if (registry().root.data < registry().tree_id_initialized.size() && registry().tree_id_initialized[registry().root.data])
  {
    const auto *root = registry().get(registry().root);
    if (root)
    {
      // The root octree CELL, which is a power-of-two cube that can be considerably larger than the
      // data inside it.
      //
      // It is tempting to report points_collection_t::min/max instead, since those bound the actual
      // points -- but they are MORTON CODES, and the smallest morton code in a set is not the
      // per-axis minimum of that set. Decoding them yields two points on the Z-order curve, not the
      // corners of a box, and the result is not even guaranteed to satisfy min <= max. (The renderer
      // decodes them the same way for frustum culling, where a slightly wrong box only costs
      // accuracy.) A caller that needs the real extent should run a coarse query and take the bounds
      // of what comes back.
      double lo[3];
      double hi[3];
      convert_morton_to_pos(registry().tree_config.scale, registry().tree_config.offset, root->morton_min, lo);
      convert_morton_to_pos(registry().tree_config.scale, registry().tree_config.offset, root->morton_max, hi);
      for (int i = 0; i < 3; i++)
      {
        out.aabb_min[i] = lo[i];
        out.aabb_max[i] = hi[i];
      }
      out.root_lod = morton::morton_magnitude_to_lod(root->magnitude);
      out.deepest_lod = out.root_lod;
      if (!root->data[0].empty())
        out.point_count = root->data[0][0].point_count;
    }
  }
  out.point_count_exact = 0; // no total is persisted; only a full enumeration could make this exact
}

} // namespace dew::access
