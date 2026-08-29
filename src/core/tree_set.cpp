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

#include "tree_set.hpp"

#include "morton_tree_coordinate_transform.hpp"

#include <atomic>
#include <utility>
#include <cstring>

namespace dew::core
{

tree_set_t::tree_set_t(blob_reader_t &reader, vio::event_loop_t &loop)
  : _reader(reader)
  , _loop(loop)
{
}

dew_error_t tree_set_t::initialize(const std::unique_ptr<uint8_t[]> &data, uint32_t size)
{
  dew_error_t error = tree_registry_deserialize(data, size, _registry);
  if (error.code != 0)
    return error;
  // Sized ONCE here and never grown. That is what lets the residency checks below (and a walker on
  // another thread) read the vectors without a lock: slots are only ever filled in, never moved.
  _requested.assign(_registry.data.size(), 0);
  return {};
}

bool tree_set_t::resident(tree_id_t id) const
{
  if (id.data >= _registry.tree_id_initialized.size())
    return false;
  // const_cast because std::atomic_ref has no const specialization; the access is still a pure load.
  // Acquire pairs with the release store in install(), so a walker on another thread that sees the 1
  // also sees the tree behind it.
  auto &slot = const_cast<uint8_t &>(_registry.tree_id_initialized[id.data]);
  return std::atomic_ref<uint8_t>(slot).load(std::memory_order_acquire) != 0;
}

bool tree_set_t::install(tree_id_t id, const serialized_tree_t &data, dew_error_t &error)
{
  if (id.data >= _registry.data.size())
  {
    error = {1, "tree id is outside the registry"};
    return false;
  }
  // A concurrent request() and load() of the same id can both get here. Installing twice would
  // replace a tree the walker may already be reading, so the first one wins.
  if (resident(id))
    return true;

  auto tree = std::make_unique<tree_t>();
  if (!tree_deserialize(data, *tree, error))
    return false;
  tree_compute_leaves_collapsed(*tree, _registry);
  #ifndef NDEBUG
  tree_compute_mins(*tree); // debug-only shadow, never serialized -- see tree.hpp
  #endif
  _registry.data[id.data] = std::move(tree);
  if (_registry.tree_id_initialized.size() < _registry.data.size())
    _registry.tree_id_initialized.resize(_registry.data.size());
  // Release store: a walker on another thread that sees the 1 must also see the tree behind it.
  std::atomic_ref<uint8_t>(_registry.tree_id_initialized[id.data]).store(1, std::memory_order_release);
  return true;
}

vio::task_t<bool> tree_set_t::do_load(tree_id_t id, dew_error_t &error)
{
  if (id.data >= _registry.locations.size())
  {
    error = {1, "tree id is outside the registry"};
    co_return false;
  }
  if (resident(id))
    co_return true;
  const auto location = _registry.locations[id.data];
  if (location.size == 0)
  {
    error = {1, "tree has no stored location"};
    co_return false;
  }

  std::shared_ptr<read_request_t> request;
  // decompress_inline: the caller is on the loop, not a pool worker, so decompressing here cannot
  // park a pool thread -- and hopping to a pool that may have no threads (wasm) would never return.
  co_await co_read(_reader, location, read_options_t{false, true, {}}, _loop, request);
  if (request->error.code != 0)
  {
    error = request->error;
    co_return false;
  }

  serialized_tree_t serialized;
  serialized.size = int(request->buffer_info.size);
  serialized.data = request->buffer;
  co_return install(id, serialized, error);
}

vio::task_t<bool> tree_set_t::load(tree_id_t id, dew_error_t &error)
{
  if (resident(id))
    co_return true;
  if (_shutting_down)
  {
    error = {1, "the tree set is shutting down"};
    co_return false;
  }
  if (id.data < _requested.size())
    _requested[id.data] = 1;
  _in_flight.fetch_add(1, std::memory_order_acq_rel);
  _loads_started.fetch_add(1, std::memory_order_acq_rel);
  bool ok = co_await do_load(id, error);
  _in_flight.fetch_sub(1, std::memory_order_acq_rel);
  // std::move, not `co_return ok`: vio's promise takes return_value(T &&), and MSVC will not bind an
  // lvalue to it (clang and gcc happen to accept the same code).
  co_return std::move(ok);
}

void tree_set_t::request(std::vector<tree_id_t> ids)
{
  if (ids.empty() || _shutting_down)
    return;
  // Hop to the loop before touching _requested / _in_flight: the caller may be a render thread, and
  // the loop is where loads complete. By value so the vector outlives the post.
  _loop.run_in_loop([this, ids = std::move(ids)]() { start_requested(ids); });
}

void tree_set_t::start_requested(const std::vector<tree_id_t> &ids)
{
  if (_shutting_down)
    return;
  for (auto id : ids)
  {
    if (id.data >= _requested.size() || _requested[id.data] || resident(id))
      continue;
    _requested[id.data] = 1;
    _in_flight.fetch_add(1, std::memory_order_acq_rel);
    _loads_started.fetch_add(1, std::memory_order_acq_rel);
    // Detached: the caller gets no completion and does not want one -- it re-walks on a later frame
    // and finds the tree resident. Errors are swallowed for the same reason a missing tree is simply
    // not drawn; there is nobody to report to, and the walk stays correct without it.
    [](tree_set_t *self, tree_id_t tree_id) -> vio::detached_task_t {
      dew_error_t error;
      co_await self->do_load(tree_id, error);
      self->_in_flight.fetch_sub(1, std::memory_order_acq_rel);
    }(this, id);
  }
}

void tree_set_t::data_aabb(double min[3], double max[3]) const
{
  double out_min[3] = {0, 0, 0};
  double out_max[3] = {0, 0, 0};
  const auto &offset = _registry.tree_config.offset;
  const auto scale = _registry.tree_config.scale;

  // Guard against a request before any tree exists (would dereference a null/OOB root).
  if (_registry.root.data >= _registry.data.size() || !_registry.data[_registry.root.data])
  {
    memcpy(min, out_min, sizeof(out_min));
    memcpy(max, out_max, sizeof(out_max));
    return;
  }
  const auto *tree = _registry.data[_registry.root.data].get();

  morton::morton192_t morton_max = {};
  morton::morton192_t morton_min = morton::morton_negate(morton_max);
  bool found = false;
  // Points are held at the coarsest node they fit under node_limit -- for a small cloud that is
  // data[0][0], not data[4]. Scan every level, skipping empty collections (whose min/max are
  // uninitialized). Only ever inspect populated collections.
  for (int level = 0; level < 5; level++)
  {
    for (const auto &data : tree->data[level])
    {
      if (data.point_count == 0)
        continue;
      found = true;
      if (data.min < morton_min)
        morton_min = data.min;
      if (morton_max < data.max)
        morton_max = data.max;
    }
  }
  if (!found)
  {
    // Root holds no point collections directly (all data pushed into sub-trees, or none yet). Fall
    // back to the root cell's bounds -- a valid, if loose, box -- never an inverted min>max.
    morton_min = tree->morton_min;
    morton_max = tree->morton_max;
  }
  convert_morton_to_pos(scale, offset, morton_min, out_min);
  convert_morton_to_pos(scale, offset, morton_max, out_max);
  memcpy(min, out_min, sizeof(out_min));
  memcpy(max, out_max, sizeof(out_max));
}

} // namespace dew::core
