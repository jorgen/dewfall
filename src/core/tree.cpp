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
#include "tree.hpp"

#include "memory_writer.hpp"

#include <cassert>

namespace dew::core
{
static uint32_t points_collection_serialize_size(const points_collection_t &points)
{
  uint32_t size = 0;
  size += sizeof(points.point_count);
  size += sizeof(points.min);
  size += sizeof(points.max);
  size += sizeof(points.min_lod);
  size += sizeof(uint32_t(points.data.size()));
  size += sizeof(points.data[0]) * uint32_t(points.data.size());
  return size;
}

static uint32_t points_collections_serialize_size(const std::vector<points_collection_t> &points)
{
  uint32_t size = 0;
  size += sizeof(uint32_t); // points.size()
  for (auto &p : points)
  {
    size += points_collection_serialize_size(p);
  }
  return int(size);
}

static std::pair<bool, uint8_t *> points_collection_serialize(const points_collection_t &points, uint8_t *buffer, const uint8_t *end_ptr)
{
  uint8_t *ptr = buffer;

  if (!write_memory(ptr, end_ptr, points.point_count))
    return {false, ptr};
  if (!write_memory(ptr, end_ptr, points.min))
    return {false, ptr};
  if (!write_memory(ptr, end_ptr, points.max))
    return {false, ptr};
  if (!write_memory(ptr, end_ptr, points.min_lod))
    return {false, ptr};

  auto data_size = uint32_t(points.data.size());
  if (!write_memory(ptr, end_ptr, data_size))
    return {false, ptr};

  if (!write_vec_type(ptr, end_ptr, points.data))
    return {false, ptr};
  return {true, ptr};
}

static std::pair<bool, const uint8_t *> points_collection_deserialize(const uint8_t *buffer, const uint8_t *end_ptr, points_collection_t &points)
{
  auto ptr = buffer;
  if (!read_memory(ptr, end_ptr, points.point_count))
    return {false, ptr};
  if (!read_memory(ptr, end_ptr, points.min))
    return {false, ptr};
  if (!read_memory(ptr, end_ptr, points.max))
    return {false, ptr};
  if (!read_memory(ptr, end_ptr, points.min_lod))
    return {false, ptr};

  uint32_t data_size = 0;
  if (!read_memory(ptr, end_ptr, data_size))
    return {false, ptr};

  if (!read_vec_type(ptr, end_ptr, points.data, data_size))
    return {false, ptr};

  return {true, ptr};
}

static std::pair<bool, uint8_t *> points_collections_serialize(const std::vector<points_collection_t> &points, uint8_t *buffer, const uint8_t *end_ptr)
{
  uint8_t *ptr = buffer;

  auto size = uint32_t(points.size());
  if (!write_memory(ptr, end_ptr, size))
    return {false, ptr};

  for (auto &p : points)
  {
    auto result = points_collection_serialize(p, ptr, end_ptr);
    ptr = result.second;
    if (!result.first)
      return {false, ptr};
  }
  return {true, ptr};
}

static std::pair<bool, const uint8_t *> points_collections_deserialize(const uint8_t *start, const uint8_t *end, std::vector<points_collection_t> &points)
{
  auto ptr = start;
  uint32_t size = 0;
  if (!read_memory(ptr, end, size))
    return {false, ptr};

  points.resize(size);
  for (auto &p : points)
  {
    auto result = points_collection_deserialize(ptr, end, p);
    if (!result.first)
      return result;
    ptr = result.second;
  }
  return {true, ptr};
}

serialized_tree_t tree_serialize(const tree_t &tree)
{
  size_t tree_size = 0;
  tree_size += sizeof(tree.morton_min);
  tree_size += sizeof(tree.morton_max);
  tree_size += sizeof(tree.id);
  tree_size += sizeof(tree.magnitude);
  for (int i = 0; i < 5; i++)
  {
    assert(tree.nodes[i].size() == tree.skips[i].size());
    assert(tree.nodes[i].size() == tree.node_ids[i].size());
    assert(tree.nodes[i].size() == tree.data[i].size());
    auto level_size = uint32_t(tree.nodes[i].size());
    tree_size += sizeof(level_size);
    tree_size += level_size * sizeof(tree.nodes[i][0]);
    tree_size += level_size * sizeof(tree.skips[i][0]);
    tree_size += level_size * sizeof(tree.node_ids[i][0]);
    tree_size += points_collections_serialize_size(tree.data[i]);
  }
  tree_size += sizeof(uint32_t); // tree.sub_trees.size()
  tree_size += tree.sub_trees.size() * sizeof(tree.sub_trees[0]);
  tree_size += tree.storage_map.serialized_size();

  auto data = std::make_shared<uint8_t[]>(tree_size);
  uint8_t *ptr = data.get();
  uint8_t *end_ptr = ptr + tree_size;

  if (!write_memory(ptr, end_ptr, tree.morton_min))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree.morton_max))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree.id))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree.magnitude))
    return {nullptr, 0};

  for (int i = 0; i < 5; i++)
  {
    auto level_size = uint32_t(tree.nodes[i].size());
    if (!write_memory(ptr, end_ptr, level_size))
      return {nullptr, 0};
    if (level_size == 0)
      continue;
    if (!write_vec_type(ptr, end_ptr, tree.nodes[i]))
      return {nullptr, 0};
    if (!write_vec_type(ptr, end_ptr, tree.skips[i]))
      return {nullptr, 0};
    if (!write_vec_type(ptr, end_ptr, tree.node_ids[i]))
      return {nullptr, 0};

    auto result = points_collections_serialize(tree.data[i], ptr, end_ptr);
    ptr = result.second;
    if (!result.first)
      return {nullptr, 0};
  }

  auto sub_trees_size = uint32_t(tree.sub_trees.size());
  if (!write_memory(ptr, end_ptr, sub_trees_size))
    return {nullptr, 0};

  if (sub_trees_size > 0 && !write_vec_type(ptr, end_ptr, tree.sub_trees))
    return {nullptr, 0};

  auto result = tree.storage_map.serialize(ptr, end_ptr);
  ptr = result.second;
  if (!result.first)
    return {nullptr, 0};

  return {std::move(data), int(tree_size)};
}

bool tree_deserialize(const serialized_tree_t &serialized_tree, tree_t &tree, dew_error_t &error)
{
  const uint8_t *ptr = serialized_tree.data.get();
  const uint8_t *end_ptr = ptr + serialized_tree.size;

  if (!read_memory(ptr, end_ptr, tree.morton_min))
  {
    error = {1, "Invalid tree data"};
    return false;
  }

  if (!read_memory(ptr, end_ptr, tree.morton_max))
  {
    error = {1, "Invalid tree data"};
    return false;
  }

  if (!read_memory(ptr, end_ptr, tree.id))
  {
    error = {1, "Invalid tree data"};
    return false;
  }

  if (!read_memory(ptr, end_ptr, tree.magnitude))
  {
    error = {1, "Invalid tree data"};
    return false;
  }

  for (int i = 0; i < 5; i++)
  {
    uint32_t level_size = 0;
    if (!read_memory(ptr, end_ptr, level_size))
    {
      error = {1, "Invalid tree data"};
      return false;
    }
    if (level_size == 0)
      continue;
    if (!read_vec_type(ptr, end_ptr, tree.nodes[i], level_size))
    {
      error = {1, "Invalid tree data"};
      return false;
    }

    if (!read_vec_type(ptr, end_ptr, tree.skips[i], level_size))
    {
      error = {1, "Invalid tree data"};
      return false;
    }

    if (!read_vec_type(ptr, end_ptr, tree.node_ids[i], level_size))
    {
      error = {1, "Invalid tree data"};
      return false;
    }

    auto result = points_collections_deserialize(ptr, end_ptr, tree.data[i]);
    ptr = result.second;
    if (!result.first)
    {
      error = {1, "Invalid tree data"};
      return false;
    }
  }

  uint32_t sub_trees_size = 0;
  if (!read_memory(ptr, end_ptr, sub_trees_size))
  {
    error = {1, "Invalid tree data"};
    return false;
  }

  if (sub_trees_size > 0 && !read_vec_type(ptr, end_ptr, tree.sub_trees, sub_trees_size))
  {
    error = {1, "Invalid tree data"};
    return false;
  }

  auto result = tree.storage_map.deserialize(ptr, end_ptr);
  ptr = result.second;
  if (!result.first)
  {
    error = {1, "Invalid tree data"};
    return false;
  }



  return true;
}

// Recompute the transient collapse flag for a (de)serialized tree: every leaf must be a single
// WHOLE-unit subset. A subset {id, 0, n} alone is ambiguous (the first octant child of a split
// also starts at 0), so whole-unit means: a collapsed-leaf id (their units are exact by
// construction), or a reader chunk whose registry-recorded point count equals the subset's. A
// chunk with no refs entry (cache written before registry v3) counts as NOT collapsed -- the
// collapse pass rewrites it, which is always safe.
#ifndef NDEBUG
void tree_compute_mins(tree_t &tree)
{
  // The walk mirrors tree_get_work_items (tree_lod_generator.cpp): a node's minimum morton is
  // set_name_in_morton(magnitude, tree.morton_min, name), where the root's name is the tree's own
  // child mask shifted into the top slot and each child's name comes from its PARENT'S node id --
  // not from the parent's name. Getting that wrong is loud rather than silent: the LOD walk asserts
  // mins[level][skip] == the derived node_min on every node it visits.
  for (int level = 0; level < 5; level++)
    tree.mins[level].assign(tree.nodes[level].size(), morton::morton192_t{});
  if (tree.nodes[0].empty())
    return;

  struct entry_t
  {
    uint16_t skip;
    uint16_t name;
  };
  std::vector<entry_t> current;
  std::vector<entry_t> next;
  const uint16_t root_name = morton::morton_get_child_mask(morton::morton_magnitude_to_lod(tree.magnitude), tree.morton_min);
  current.push_back({uint16_t(0), uint16_t(root_name << (4 * 3))});

  for (int level = 0; level < 5 && !current.empty(); level++)
  {
    next.clear();
    for (const auto &entry : current)
    {
      if (entry.skip >= tree.mins[level].size())
        continue; // a truncated/foreign blob: leave the rest zeroed rather than walk off the end
      tree.mins[level][entry.skip] = morton::set_name_in_morton(tree.magnitude, tree.morton_min, entry.name);
      if (level == 4)
        continue; // level 4's children live in sub-trees, which carry their own mins
      const uint8_t node = tree.nodes[level][entry.skip];
      const uint16_t parent_node_name = uint16_t(tree.node_ids[level][entry.skip] << 3);
      int child_count = 0;
      for (int child_index = 0; child_index < 8; child_index++)
      {
        if (!(node & uint8_t(1 << child_index)))
          continue;
        next.push_back({uint16_t(tree.skips[level][entry.skip] + child_count), morton::morton_get_name(parent_node_name, level, child_index)});
        child_count++;
      }
    }
    current.swap(next);
  }
}
#endif

void tree_compute_leaves_collapsed(tree_t &tree, const tree_registry_t &tree_registry)
{
  tree.leaves_collapsed = true;
  for (int level = 0; level < 5; level++)
  {
    for (uint32_t skip = 0; skip < uint32_t(tree.nodes[level].size()); skip++)
    {
      auto &collection = tree.data[level][skip];
      if (tree.nodes[level][skip] != 0 || collection.point_count == 0)
        continue;
      if (collection.data.size() != 1 || collection.data[0].offset.data != 0 || uint64_t(collection.data[0].count.data) != collection.point_count)
      {
        tree.leaves_collapsed = false;
        return;
      }
      auto id = collection.data[0].input_id;
      if (input_data_id_is_collapsed_leaf(id))
        continue;
      auto refs = tree_registry.chunk_tree_refs.find(id);
      if (refs == tree_registry.chunk_tree_refs.end() || refs->second.point_count != collection.data[0].count.data)
      {
        tree.leaves_collapsed = false;
        return;
      }
    }
  }
}

// Registry blob v3 ('TRG3'): v2 added current_lod_node_id (v1 dropped it -- a resumed conversion
// would reuse LOD input_data_ids and corrupt storage maps), the done-morton watermark, per-tree
// state/band (incremental finalization/upload), and the opaque input-registry snapshot (resume).
// v3 adds current_collapsed_node_id, the registry-global chunk_tree_refs table, and grows the
// serialized tree_config_t (read_chunk_byte_target). v2 and v1 blobs (v1: no magic; first u32 is
// node_limit) still deserialize -- a realistic node_limit can never equal either magic.
static constexpr uint32_t k_tree_registry_magic_v2 = 0x32475254u; // 'TRG2' little-endian
static constexpr uint32_t k_tree_registry_magic_v3 = 0x33475254u; // 'TRG3' little-endian
static constexpr uint32_t k_tree_registry_magic_v4 = 0x34475254u; // 'TRG4' little-endian (grown tree_config: lod flags)

// The v2 on-disk layout of tree_config_t (before read_chunk_byte_target). Field order matches the
// live struct so the memcpy'd bytes line up.
struct tree_config_v2_t
{
  double scale;
  double offset[3];
  bool store_original_order;
  uint32_t node_point_limit;
};
static_assert(sizeof(tree_config_v2_t) == 40, "v2 registry blobs serialized a 40-byte tree_config");

// The exact 48-byte layout v3 registry blobs serialized (v4 grew the struct with the lod flags).
struct tree_config_v3_t
{
  double scale;
  double offset[3];
  bool store_original_order;
  uint32_t node_point_limit;
  uint64_t read_chunk_byte_target;
};
static_assert(sizeof(tree_config_v3_t) == 48, "v3 registry blobs serialized a 48-byte tree_config");
static_assert(sizeof(tree_config_t) == 56, "v4 registry blobs serialize a 56-byte tree_config");

serialized_tree_registry_t tree_registry_serialize(const tree_registry_t &tree_registry)
{
  auto tree_registry_count = uint32_t(tree_registry.locations.size());
  assert(tree_registry.tree_state.size() == tree_registry_count);
  assert(tree_registry.tree_band.size() == tree_registry_count);
  auto input_snapshot_size = uint32_t(tree_registry.input_registry_snapshot.size());

  auto chunk_refs_count = uint32_t(tree_registry.chunk_tree_refs.size());

  uint32_t tree_registry_size = 0;
  tree_registry_size += sizeof(k_tree_registry_magic_v3);
  tree_registry_size += sizeof(tree_registry.node_limit);
  tree_registry_size += sizeof(tree_registry.current_id);
  tree_registry_size += sizeof(tree_registry.root);
  tree_registry_size += sizeof(tree_registry.tree_config);
  tree_registry_size += sizeof(tree_registry.current_lod_node_id);
  tree_registry_size += sizeof(tree_registry.current_collapsed_node_id);
  tree_registry_size += sizeof(tree_registry.lod_watermark);
  tree_registry_size += sizeof(chunk_refs_count);
  tree_registry_size += (uint32_t(sizeof(input_data_id_t)) + 2 * uint32_t(sizeof(uint32_t))) * chunk_refs_count;
  tree_registry_size += sizeof(tree_registry_count);
  tree_registry_size += sizeof(storage_location_t) * tree_registry_count;
  tree_registry_size += uint32_t(sizeof(uint8_t)) * tree_registry_count;  // tree_state
  tree_registry_size += uint32_t(sizeof(uint32_t)) * tree_registry_count; // tree_band
  tree_registry_size += sizeof(input_snapshot_size);
  tree_registry_size += input_snapshot_size;

  auto data = std::make_shared<uint8_t[]>(tree_registry_size);
  uint8_t *ptr = data.get();
  uint8_t *end_ptr = ptr + tree_registry_size;

  if (!write_memory(ptr, end_ptr, k_tree_registry_magic_v4))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree_registry.node_limit))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree_registry.current_id))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree_registry.root))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree_registry.tree_config))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree_registry.current_lod_node_id))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree_registry.current_collapsed_node_id))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, tree_registry.lod_watermark))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, chunk_refs_count))
    return {nullptr, 0};
  for (auto &[chunk_id, chunk_ref] : tree_registry.chunk_tree_refs)
  {
    if (!write_memory(ptr, end_ptr, chunk_id))
      return {nullptr, 0};
    if (!write_memory(ptr, end_ptr, chunk_ref.tree_count))
      return {nullptr, 0};
    if (!write_memory(ptr, end_ptr, chunk_ref.point_count))
      return {nullptr, 0};
  }
  if (!write_memory(ptr, end_ptr, tree_registry_count))
    return {nullptr, 0};
  if (!write_vec_type(ptr, end_ptr, tree_registry.locations))
    return {nullptr, 0};
  if (!write_vec_type(ptr, end_ptr, tree_registry.tree_state))
    return {nullptr, 0};
  if (!write_vec_type(ptr, end_ptr, tree_registry.tree_band))
    return {nullptr, 0};
  if (!write_memory(ptr, end_ptr, input_snapshot_size))
    return {nullptr, 0};
  if (input_snapshot_size && !write_vec_type(ptr, end_ptr, tree_registry.input_registry_snapshot))
    return {nullptr, 0};
  return {std::move(data), int(tree_registry_size)};
}

dew_error_t tree_registry_deserialize(const std::unique_ptr<uint8_t[]> &data, uint32_t data_size, tree_registry_t &tree_registry)
{
  const uint8_t *ptr = data.get();
  const uint8_t *end_ptr = ptr + data_size;
  uint32_t first_word = 0;
  if (!read_memory(ptr, end_ptr, first_word))
    return {1, "Invalid tree registry data"};
  const bool v4 = first_word == k_tree_registry_magic_v4;
  const bool v3 = v4 || first_word == k_tree_registry_magic_v3;
  const bool v2 = v3 || first_word == k_tree_registry_magic_v2;
  // A future 'TRG5'+ blob would otherwise fall through to the v1 branch (first_word treated as
  // node_limit) and silently misparse. Any 'TRG?' word that is not a known version is a
  // newer-format dataset; refuse it explicitly. (Real node_limit values never reach ~0x54475254.)
  if (!v2 && (first_word & 0x00ffffffu) == 0x00475254u)
    return {1, "Tree registry was written by a newer version of dewfall (unknown 'TRG' format); upgrade to read it"};
  if (v2)
  {
    if (!read_memory(ptr, end_ptr, tree_registry.node_limit))
      return {1, "Invalid tree registry data"};
  }
  else
  {
    tree_registry.node_limit = first_word; // v1: no magic, first u32 IS node_limit
  }
  if (!read_memory(ptr, end_ptr, tree_registry.current_id))
    return {1, "Invalid tree registry data"};
  if (!read_memory(ptr, end_ptr, tree_registry.root))
    return {1, "Invalid tree registry data"};
  if (v4)
  {
    if (!read_memory(ptr, end_ptr, tree_registry.tree_config))
      return {1, "Invalid tree registry data"};
  }
  else if (v3)
  {
    // v3 serialized the 48-byte config. A resumed pre-v4 conversion keeps the CLASSIC lod behavior
    // (all attributes, fixed sampling): its earlier passes already wrote LOD that way.
    tree_config_v3_t old_config;
    if (!read_memory(ptr, end_ptr, old_config))
      return {1, "Invalid tree registry data"};
    tree_registry.tree_config.scale = old_config.scale;
    memcpy(tree_registry.tree_config.offset, old_config.offset, sizeof(old_config.offset));
    tree_registry.tree_config.store_original_order = old_config.store_original_order;
    tree_registry.tree_config.node_point_limit = old_config.node_point_limit;
    tree_registry.tree_config.read_chunk_byte_target = old_config.read_chunk_byte_target;
    tree_registry.tree_config.lod_all_attributes = 1;
    tree_registry.tree_config.lod_adaptive_sampling = 0;
  }
  else
  {
    // v1/v2 serialized the 40-byte config (no read_chunk_byte_target); newer fields keep classic
    // behavior for the same resume reason as v3.
    tree_config_v2_t old_config;
    if (!read_memory(ptr, end_ptr, old_config))
      return {1, "Invalid tree registry data"};
    tree_registry.tree_config.scale = old_config.scale;
    memcpy(tree_registry.tree_config.offset, old_config.offset, sizeof(old_config.offset));
    tree_registry.tree_config.store_original_order = old_config.store_original_order;
    tree_registry.tree_config.node_point_limit = old_config.node_point_limit;
    tree_registry.tree_config.lod_all_attributes = 1;
    tree_registry.tree_config.lod_adaptive_sampling = 0;
  }
  if (v2)
  {
    if (!read_memory(ptr, end_ptr, tree_registry.current_lod_node_id))
      return {1, "Invalid tree registry data"};
    if (v3)
    {
      if (!read_memory(ptr, end_ptr, tree_registry.current_collapsed_node_id))
        return {1, "Invalid tree registry data"};
    }
    if (!read_memory(ptr, end_ptr, tree_registry.lod_watermark))
      return {1, "Invalid tree registry data"};
    if (v3)
    {
      uint32_t chunk_refs_count = 0;
      if (!read_memory(ptr, end_ptr, chunk_refs_count))
        return {1, "Invalid tree registry data"};
      for (uint32_t i = 0; i < chunk_refs_count; i++)
      {
        input_data_id_t chunk_id;
        tree_registry_t::chunk_ref_t chunk_ref;
        if (!read_memory(ptr, end_ptr, chunk_id))
          return {1, "Invalid tree registry data"};
        if (!read_memory(ptr, end_ptr, chunk_ref.tree_count))
          return {1, "Invalid tree registry data"};
        if (!read_memory(ptr, end_ptr, chunk_ref.point_count))
          return {1, "Invalid tree registry data"};
        tree_registry.chunk_tree_refs[chunk_id] = chunk_ref;
      }
    }
  }
  uint32_t tree_registry_count = 0;
  if (!read_memory(ptr, end_ptr, tree_registry_count))
    return {1, "Invalid tree registry data"};
  if (!read_vec_type(ptr, end_ptr, tree_registry.locations, tree_registry_count))
    return {1, "Invalid tree registry data"};
  if (v2)
  {
    if (!read_vec_type(ptr, end_ptr, tree_registry.tree_state, tree_registry_count))
      return {1, "Invalid tree registry data"};
    if (!read_vec_type(ptr, end_ptr, tree_registry.tree_band, tree_registry_count))
      return {1, "Invalid tree registry data"};
    uint32_t input_snapshot_size = 0;
    if (!read_memory(ptr, end_ptr, input_snapshot_size))
      return {1, "Invalid tree registry data"};
    if (!read_vec_type(ptr, end_ptr, tree_registry.input_registry_snapshot, input_snapshot_size))
      return {1, "Invalid tree registry data"};
  }
  else
  {
    // v1 file: every tree is considered still building, none banded, no snapshot/watermark.
    tree_registry.tree_state.assign(tree_registry_count, uint8_t(tree_state_t::building));
    tree_registry.tree_band.assign(tree_registry_count, tree_band_none);
  }

  tree_registry.data.resize(tree_registry_count);
  tree_registry.tree_id_initialized.resize(tree_registry_count);
  return {};
}
} // namespace dew::core
