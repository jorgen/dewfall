/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2022  Jørgen Lind
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

#include <vio/event_loop.h>
#include <vio/event_pipe.h>
#include <vio/thread_pool.h>

#include "attributes_configs.hpp"
#include "perf_stats.hpp"
#include "tree.hpp"

#include <ankerl/unordered_dense.h>
#include <memory>
#include <deque>

namespace dew::core
{
class attributes_configs_t;
}

namespace dew::converter
{
using namespace dew::core;
class storage_handler_t;

struct lod_child_storage_info_t
{
  attributes_id_t attributes_id;
  std::vector<storage_location_t> locations;
  // False when this parent consumes the child's unit WHOLE, i.e. it is the blob's only reader and
  // there is no point keeping it at the hot end of the cache. Decided in get_storage_info, where the
  // subset counts and the storage locations are both in scope.
  bool retain_hot = true;
};

using child_storage_map_t = ankerl::unordered_dense::map<input_data_id_t, lod_child_storage_info_t, input_data_id_hash_t>;

// Shared with the leaf-collapse path: slice a subset window out of a whole (decoded) buffer, and
// scatter source attributes into destination buffers by an (input id, source index) permutation.
dew_blob_t morton_buffer_for_subset(const dew_blob_t &buffer, dew_type_t format, offset_in_subset_t offset, point_count_t count);
void quantize_attributres(storage_handler_t &cache, const child_storage_map_t &child_storage_map, const std::vector<std::pair<input_data_id_t, uint32_t>> &indecies,
                          const attribute_lod_mapping_t &lod_attrib_mapping, attribute_buffers_t &buffers);
struct lod_node_worker_data_t
{
  morton::morton192_t node_min;
  uint16_t id;
  uint16_t lod;
  input_data_id_t storage_name;
  std::vector<points_collection_t> child_data;
  std::vector<tree_id_t> child_trees;
  child_storage_map_t child_storage_info;
  point_count_t generated_point_count;
  morton::morton192_t generated_min;
  morton::morton192_t generated_max;
  attributes_id_t generated_attributes_id;
  std::vector<storage_location_t> generated_locations;
};

struct lod_tree_worker_data_t
{
  tree_id_t tree_id;
  int magnitude;
  std::vector<lod_node_worker_data_t> nodes[5];
};

class tree_lod_generator_t;
struct lod_worker_batch_t;
class lod_worker_t
{
public:
  lod_worker_t(tree_lod_generator_t &lod_generator, lod_worker_batch_t &batch, uint32_t tree_index, storage_handler_t &cache, attributes_configs_t &attributes_configs, lod_node_worker_data_t &data,
               const std::vector<float> &random_offsets);
  void work();
  void enqueue_lod(vio::thread_pool_t &pool)
  {
    pool.enqueue([this] { this->work(); });
  }
private:
  tree_lod_generator_t &lod_generator;
  lod_worker_batch_t &batch;
  uint32_t tree_index; // which tree in batch.worker_data this node belongs to
  storage_handler_t &cache;
  attributes_configs_t &attributes_configs;
  lod_node_worker_data_t &data;
  const std::vector<float> &random_offsets;
};

// One magnitude's worth of trees, each walking its own five levels.
//
// The levels of a SINGLE tree are strictly ordered -- a node samples its children, so level L waits
// for L+1. Trees in the same batch are NOT ordered against each other: a node's children live either
// in its own tree or, at level 4, in a SUB-tree, and a sub-tree has magnitude-1 and therefore lives
// in an earlier batch. Sibling trees at one magnitude never reference each other.
//
// The scheduler used to barrier on the whole batch at every level, so every tree waited for the
// slowest one five times over, and each tree's level 0 is a single node -- so the tail of every
// batch ran nearly serial with the pool idle. Per-tree progression removes that without touching the
// dependency order.
struct lod_worker_batch_t
{
  std::vector<lod_tree_worker_data_t> worker_data;

  // Parallel to worker_data. Kept here rather than inside lod_tree_worker_data_t because that type is
  // moved into the batch (std::make_move_iterator) and std::atomic is not movable.
  std::vector<std::vector<lod_worker_t>> tree_workers;
  std::vector<int> tree_level;     // next level to descend to; starts at 5, done at < 0
  std::vector<int> tree_in_flight; // workers enqueued for the current level; written on the tree loop before enqueue
  std::unique_ptr<std::atomic_int[]> tree_completed;
  int trees_active = 0;
  bool started = false;
};

class tree_lod_generator_t
{
public:
  tree_lod_generator_t(vio::event_loop_t &loop, vio::thread_pool_t &thread_pool, tree_registry_t &tree_cache, storage_handler_t &file_cache, attributes_configs_t &attributes_configs, perf_stats_t &perf_stats, vio::event_pipe_t<void> &lod_done);
  void generate_lods(tree_id_t &tree_id, const morton::morton192_t &max);

  // Resume: seed the already-LOD'd floor from the persisted registry watermark, so the first pass
  // of a reopened conversion doesn't re-walk (and re-LOD) nodes below it -- those belong to trees
  // that may already be finalized (immutable).
  void restore_lod_complete_morton(const morton::morton192_t &m)
  {
    _lod_complete_morton = m;
  }

  perf_stats_t &stats() { return _perf_stats; }
  void iterate_workers();

  // Called from the STORAGE loop (the write completion callback). Only ever touches this tree's own
  // counter, and only the completion that finishes a tree's level wakes the tree loop -- so the wake
  // rate is unchanged from the old per-batch-level scheme, just attributed per tree.
  void add_worker_done(lod_worker_batch_t &batch, uint32_t tree_index)
  {
    if (++batch.tree_completed[tree_index] == batch.tree_in_flight[tree_index])
    {
      _iterate_workers.post_event();
    }
  }

private:
  vio::event_loop_t &_loop;
  vio::thread_pool_t &_thread_pool;
public:
  const tree_config_t &generation_tree_config() const
  {
    return _tree_cache.tree_config;
  }

private:
  tree_registry_t &_tree_cache;
  storage_handler_t &_file_cache;
  attributes_configs_t &_attributes_configs;
  perf_stats_t &_perf_stats;
  vio::event_pipe_t<void> &_lod_done;
  vio::event_pipe_t<void> _iterate_workers;

  std::vector<float> _random_offsets;

  std::deque<std::unique_ptr<lod_worker_batch_t>> _lod_batches;
  morton::morton192_t _lod_complete_morton = {};
};

} // namespace dew::converter
