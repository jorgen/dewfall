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

#include <vector>

#include "dataset_types.hpp"
#include "input_storage_map.hpp"
#include "morton.hpp"


namespace dew::core
{
struct serialized_tree_t
{
  std::shared_ptr<uint8_t[]> data;
  int size;
};

struct serialized_tree_registry_t
{
  std::shared_ptr<uint8_t[]> data;
  int size;
};

struct points_collection_t
{
  uint64_t point_count = 0;
  morton::morton192_t min;
  morton::morton192_t max;
  int min_lod;
  std::vector<points_subset_t> data;
};

inline void points_data_initialize(points_collection_t &to_init, const storage_header_t &header)
{
  to_init.point_count = header.point_count;
  to_init.data.emplace_back(header.input_id, offset_in_subset_t(0), point_count_t(uint32_t(header.point_count)));
  to_init.max = header.morton_max;
  to_init.min = header.morton_min;
  to_init.min_lod = header.lod_span;
  assert(morton::morton_lod(to_init.min, to_init.max) == to_init.min_lod);
}

inline void points_data_add(points_collection_t &dest, points_collection_t &&to_add)
{
  if (dest.point_count == 0)
  {
    dest = std::move(to_add);
    return;
  }
  for (auto &p : to_add.data)
    dest.data.emplace_back(std::move(p));
  dest.point_count += to_add.point_count;
  if (to_add.min < dest.min)
    dest.min = to_add.min;
  if (dest.max < to_add.max)
    dest.max = to_add.max;
  dest.min_lod = morton::morton_lod(dest.min, dest.max);
}

inline void points_data_add(points_collection_t &dest, const storage_header_t &to_add)
{
  if (dest.point_count == 0)
  {
    points_data_initialize(dest, to_add);
    return;
  }
  dest.data.emplace_back(to_add.input_id, offset_in_subset_t(0), point_count_t(uint32_t(to_add.point_count)));
  dest.point_count += to_add.point_count;
  if (to_add.morton_min < dest.min)
    dest.min = to_add.morton_min;
  if (dest.max < to_add.morton_max)
    dest.max = to_add.morton_max;
  dest.min_lod = morton::morton_lod(dest.min, dest.max);
}

struct tree_id_t
{
  tree_id_t() = default;
  explicit tree_id_t(uint32_t a_data)
    : data(a_data)
  {
  }
  uint32_t data;
};

struct tree_t
{
  morton::morton192_t morton_min;
  morton::morton192_t morton_max;
  std::vector<uint8_t> nodes[5];
  std::vector<int16_t> skips[5];
  std::vector<uint16_t> node_ids[5];
  std::vector<points_collection_t> data[5];
  std::vector<tree_id_t> sub_trees;
#ifndef NDEBUG
  std::vector<morton::morton192_t> mins[5];
#endif
  tree_id_t id;
  uint8_t magnitude;
  bool is_dirty;
  // Transient (recomputed on deserialize, never serialized): every leaf collection is a single
  // whole-unit subset -- the shape collapse produces and finality requires. See tree_collapse.
  bool leaves_collapsed = false;
  input_storage_map_t storage_map;
};

// Per-tree lifecycle for incremental finalization/upload (registry v2). `building`: the tree may
// still receive points or LOD writes. `final`: its morton_max is proven below a committed
// done-morton watermark, so no current or future input can touch it -- the tree (blobs, serialized
// form, storage map) is immutable from that checkpoint on. `uploaded`: informational mirror of the
// destination-bucket state (the bucket's manifests are authoritative; this may lag one checkpoint).
enum class tree_state_t : uint8_t
{
  building = 0,
  final = 1,
  uploaded = 2,
};
// tree_band value for "not assigned to an upload band yet".
inline constexpr uint32_t tree_band_none = 0xFFFFFFFFu;

struct tree_registry_t
{
  tree_registry_t() = default;
  tree_registry_t(uint32_t a_node_limit, tree_config_t a_tree_config)
    : node_limit(a_node_limit)
    , tree_config(a_tree_config)
  {
    data.reserve(a_node_limit);
    locations.reserve(a_node_limit);
    tree_id_initialized.resize(a_node_limit, 0);
  }
  uint32_t node_limit = 0;
  uint32_t current_id = 0;
  tree_id_t root = {};
  tree_config_t tree_config;
  uint64_t current_lod_node_id = uint64_t(1) << 63;
  // Collapsed-leaf unit ids (registry v3; see input_data_id_is_collapsed_leaf). Persisted so a
  // resumed conversion never reuses an id already bound to storage.
  uint64_t current_collapsed_node_id = uint64_t(1) << 62;
  // Registry-global lifetime of each READER-CHUNK unit (registry v3). Chunk units are shared
  // across trees (subtree moves copy map entries), so a per-tree refcount hitting zero does not
  // prove the blob is globally unreferenced -- collapse frees a chunk's blobs only when
  // tree_count drops to zero. point_count is the unit's total (whole-unit subset detection).
  // Missing entry on a v2-resumed cache = lifetime unknown, never free.
  struct chunk_ref_t
  {
    uint32_t tree_count;
    uint32_t point_count;
  };
  ankerl::unordered_dense::map<input_data_id_t, chunk_ref_t, input_data_id_hash_t> chunk_tree_refs;
  // The done-morton watermark as of the last committed checkpoint (registry v2): restores
  // _lod_done_morton / finality decisions on resume. Zero = nothing final yet.
  morton::morton192_t lod_watermark = {};
  std::vector<std::unique_ptr<tree_t>> data;
  std::vector<storage_location_t> locations;
  std::vector<uint8_t> tree_id_initialized;
  // Parallel to `locations`, indexed by tree id (registry v2; defaulted for v1 files).
  std::vector<uint8_t> tree_state;  // tree_state_t values
  std::vector<uint32_t> tree_band;  // upload band id, tree_band_none until banded
  // Opaque serialized input_data_source_registry snapshot, persisted inside the registry blob so a
  // resumed conversion can skip re-added done inputs and restore the watermark. Refreshed by the
  // tree handler right before each checkpoint (via the processor-installed provider); handed back
  // to the processor after deserialize.
  std::vector<uint8_t> input_registry_snapshot;

  tree_t *get(tree_id_t id)
  {
    return data[id.data].get();
  }

  const tree_t *get(tree_id_t id) const
  {
    return data[id.data].get();
  }
};

// Tree CONSTRUCTION (tree_initialize / tree_add_points) lives in tree_build.hpp: it is the only
// part of the tree that needs a storage_handler_t, so keeping it out of this header leaves the
// format code free of any storage dependency.

serialized_tree_t tree_serialize(const tree_t &tree);

bool tree_deserialize(const serialized_tree_t &serialized_tree, tree_t &tree, dew_error_t &error);

// Recompute tree_t::leaves_collapsed after deserialization (needs the registry's chunk table).
void tree_compute_leaves_collapsed(tree_t &tree, const tree_registry_t &tree_registry);
#ifndef NDEBUG
// Rebuild tree_t::mins, the debug-only shadow of each node's minimum morton.
//
// mins is maintained alongside the tree while it is BUILT and is never serialized (it exists only to
// assert the structure stays consistent). A tree loaded from disk therefore has empty mins arrays,
// and inserting points into one indexes them out of range -- which only became reachable once a
// dataset could be reopened and added to. Recompute it on load, exactly as tree_compute_leaves_collapsed
// recomputes the other transient.
void tree_compute_mins(tree_t &tree);
#endif

serialized_tree_registry_t tree_registry_serialize(const tree_registry_t &tree_registry);

[[nodiscard]] dew_error_t tree_registry_deserialize(const std::unique_ptr<uint8_t[]> &data, uint32_t data_size, tree_registry_t &tree_registry);
} // namespace dew::core

