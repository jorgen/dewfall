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
#include "tree_build.hpp"

#include "memory_writer.hpp"
#include "point_buffer_splitter.hpp"
#include "tree.hpp"

#include <cassert>

namespace dew::converter
{
using namespace dew::core;
static tree_t &tree_cache_create_root_tree(tree_registry_t &tree_cache)
{
  tree_cache.data.emplace_back(new tree_t());
  tree_cache.locations.emplace_back();
  tree_cache.tree_id_initialized.push_back(1);
  tree_cache.tree_state.push_back(uint8_t(tree_state_t::building));
  tree_cache.tree_band.push_back(tree_band_none);
  tree_cache.data.back()->id.data = tree_cache.current_id++;
  return *tree_cache.data.back();
}

static tree_t &tree_cache_add_tree(tree_registry_t &tree_cache, tree_t *(&parent))
{
  auto id = parent->id;
  tree_cache.data.emplace_back(new tree_t());
  tree_cache.locations.emplace_back();
  tree_cache.tree_id_initialized.push_back(1);
  tree_cache.tree_state.push_back(uint8_t(tree_state_t::building));
  tree_cache.tree_band.push_back(tree_band_none);
  tree_cache.data.back()->id.data = tree_cache.current_id++;
  parent = tree_cache.data[id.data].get();
  return *tree_cache.data.back();
}

tree_id_t tree_initialize(tree_registry_t &tree_registry, storage_handler_t &cache, const storage_header_t &header, attributes_id_t attributes, std::vector<storage_location_t> &&locations)
{
  tree_t &tree = tree_cache_create_root_tree(tree_registry);
  morton::morton192_t mask = morton::morton_xor(header.morton_min, header.morton_max);
  int magnitude = morton::morton_magnitude_from_bit_index(morton::morton_msb(mask));
  morton::morton192_t new_tree_mask = morton::morton_mask_create<uint64_t, 3>(morton::morton_magnitude_to_lod(magnitude));
  morton::morton192_t new_tree_mask_inv = morton::morton_negate(new_tree_mask);
  tree.morton_min = morton::morton_and(header.morton_min, new_tree_mask_inv);
  tree.morton_max = morton::morton_or(header.morton_min, new_tree_mask);
  tree.is_dirty = true;
  tree.magnitude = uint8_t(magnitude);
  tree.nodes[0].push_back(0);
  tree.skips[0].push_back(int16_t(0));
  tree.data[0].emplace_back();
  uint16_t root_name = morton::morton_get_name(0, 0, morton::morton_get_child_mask(morton::morton_magnitude_to_lod(magnitude) + 1, header.morton_min));
  tree.node_ids[0].emplace_back(root_name);
#ifndef NDEBUG
  tree.mins[0].push_back(tree.morton_min);
  assert((tree.morton_min.data[0] & 1) == 0);
#endif

  auto id = tree_add_points(tree_registry, cache, tree.id, header, attributes, std::move(locations));
  return id;
}

static void tree_initialize_sub(const tree_t &parent_tree, tree_registry_t &tree_cache, const morton::morton192_t &morton, tree_t &sub_tree)
{
  (void)tree_cache;
  sub_tree.magnitude = parent_tree.magnitude - 1;
  morton::morton192_t new_tree_mask = morton::morton_mask_create<uint64_t, 3>(morton::morton_magnitude_to_lod(sub_tree.magnitude));
  morton::morton192_t new_tree_mask_inv = morton::morton_negate(new_tree_mask);
  sub_tree.morton_min = morton::morton_and(morton, new_tree_mask_inv);
  sub_tree.morton_max = morton::morton_or(morton, new_tree_mask);

  sub_tree.nodes[0].push_back(0);
  sub_tree.skips[0].push_back(int16_t(0));
  sub_tree.data[0].emplace_back();
  uint16_t root_name = morton::morton_get_name(0, 0, morton::morton_get_child_mask(morton::morton_magnitude_to_lod(sub_tree.magnitude) + 1, morton));
  sub_tree.node_ids[0].emplace_back(root_name);
#ifndef NDEBUG
  sub_tree.mins[0].push_back(sub_tree.morton_min);
  assert((sub_tree.morton_min.data[0] & 1) == 0);
#endif
}

static void tree_initialize_new_parent(const tree_t &some_child, const morton::morton192_t possible_min, const morton::morton192_t possible_max, tree_t &new_parent)
{
  morton::morton192_t new_min = some_child.morton_min < possible_min ? some_child.morton_min : possible_min;
  morton::morton192_t new_max = some_child.morton_max < possible_max ? possible_max : some_child.morton_max;

  auto min_max_msb = morton::morton_msb(morton::morton_xor(new_min, new_max));
  new_parent.magnitude = uint8_t(morton::morton_magnitude_from_bit_index(min_max_msb));
  int lod = morton::morton_magnitude_to_lod(new_parent.magnitude);
  morton::morton192_t new_tree_mask = morton::morton_mask_create<uint64_t, 3>(lod);
  morton::morton192_t new_tree_mask_inv = morton::morton_negate(new_tree_mask);
  new_parent.morton_min = morton::morton_and(new_tree_mask_inv, new_min);
  new_parent.morton_max = morton::morton_or(new_parent.morton_min, new_tree_mask);
  new_parent.nodes[0].push_back(0);
  new_parent.skips[0].push_back(int16_t(0));
  new_parent.data[0].emplace_back();
  uint16_t root_name = morton::morton_get_name(0, 0, morton::morton_get_child_mask(morton::morton_magnitude_to_lod(new_parent.magnitude) + 1, some_child.morton_min));
  new_parent.node_ids[0].emplace_back(root_name);
#ifndef NDEBUG
  new_parent.mins[0].push_back(new_parent.morton_min);
#endif
}

static int sub_tree_count_skips(uint8_t node, int index)
{
  int node_skips = 0;
  for (int i = 0; i < index; i++)
  {
    if (node & uint8_t(1 << i))
      node_skips++;
  }
  return node_skips;
}

static void sub_tree_alloc_children(tree_t &tree, int level, int skip)
{
  assert(skip <= int(tree.skips[level].size()));
  int old_skip = 0;
  if (skip < int(tree.skips[level].size()))
    old_skip = tree.skips[level][skip];
  else if (skip > 0 && skip == int(tree.skips[level].size()))
    old_skip = tree.skips[level].back() + sub_tree_count_skips(tree.nodes[level].back(), 8);

  tree.nodes[level].emplace(tree.nodes[level].begin() + skip);
  tree.skips[level].emplace(tree.skips[level].begin() + skip, uint16_t(old_skip));
  tree.data[level].emplace(tree.data[level].begin() + skip);
  tree.node_ids[level].emplace(tree.node_ids[level].begin() + skip);
#ifndef NDEBUG
  tree.mins[level].emplace(tree.mins[level].begin() + skip);
#endif
}

static void sub_tree_increase_skips(tree_t &tree, int level, int skip)
{
  auto &skips = tree.skips[level];
  auto skips_size = skips.size();
  for (int i = skip + 1; i < int(skips_size); i++)
  {
    skips[i]++;
  }
}

static void sub_tree_split_points_to_children(storage_handler_t &cache, input_storage_map_t &storage_map, points_collection_t &&points, int lod, const morton::morton192_t &node_min, points_collection_t (&children)[8])
{
  deref_on_destruct_t to_deref(storage_map);
  for (auto &p : points.data)
  {
    to_deref.add(p.input_id);
    read_only_points_t p_read(cache, storage_map.location(p.input_id, 0));
    assert(p_read.data.size);
    // Failed read: conversion is flagged (storage error pipe); skip rather than crash on null data.
    if (p_read.error.code != 0)
      continue;

    point_buffer_subdivide(p_read, storage_map, p, lod, node_min, children);
  }
  points = points_collection_t();
}

static void move_storage_locations_to_subtree(tree_registry_t &tree_cache, const points_collection_t &collection, tree_t &parent, tree_t &sub_tree)
{
  for (auto &p : collection.data)
  {
    const bool parent_had = parent.storage_map.contains(p.input_id);
    auto attrib_locations_pair = parent.storage_map.dereference(p.input_id);
    const bool parent_erased = parent_had && !parent.storage_map.contains(p.input_id);
    bool child_created = false;
    if (sub_tree.storage_map.contains(p.input_id))
    {
      // Second subset of the same unit in this collection: only bump the child's refcount --
      // add_storage on an existing id would DISCARD the very locations it re-references.
      sub_tree.storage_map.add_ref(p.input_id);
    }
    else
    {
      sub_tree.storage_map.add_storage(p.input_id, attrib_locations_pair.first, std::move(attrib_locations_pair.second));
      child_created = true;
    }
    // Registry-global chunk lifetime: +1 when a tree map gains the unit, -1 when one loses it
    // (net zero for a plain hand-off). LOD/collapsed units are per-tree, never tracked.
    if (input_data_id_is_leaf(p.input_id) && !input_data_id_is_collapsed_leaf(p.input_id))
    {
      auto refs = tree_cache.chunk_tree_refs.find(p.input_id);
      if (refs != tree_cache.chunk_tree_refs.end())
      {
        if (child_created)
          refs->second.tree_count++;
        if (parent_erased)
        {
          assert(refs->second.tree_count > 0);
          refs->second.tree_count--;
        }
        assert(refs->second.tree_count > 0 && "the sub tree references the unit, so the count cannot hit zero here");
      }
      // Missing entry: cache written before registry v3 -- chunk lifetime unknown, never freed.
    }
  }
}

static void sub_tree_insert_points(tree_registry_t &tree_cache, storage_handler_t &cache, tree_id_t tree_id, const morton::morton192_t &min, int current_level, int skip, uint16_t current_name,
                                   points_collection_t &&points) // NOLINT(*-no-recursion)
{
  auto *tree = tree_cache.get(tree_id);
  // A finalized tree is immutable: its morton_max was proven below a committed done-morton
  // watermark, so no input may route points into it ever again. Hitting this assert means the
  // watermark overclaimed (registry ordering bug) -- fail loudly instead of corrupting a tree the
  // upload/eviction tiers treat as frozen.
  assert(tree_cache.tree_state[tree_id.data] == uint8_t(tree_state_t::building) && "point insert into finalized tree");
  tree->is_dirty = true;
  // Every tree the points pass through, not just the one tree_add_points was handed. Its caller
  // clears this on the ROOT tree only, and the leaf data actually lands in whichever sub-tree the
  // routing reaches -- so clearing it there left the sub-trees claiming to be collapsed.
  //
  // Invisible in a fresh conversion, where a newly created tree starts false and only turns true
  // once collapse has run. On a reopened dataset tree_compute_leaves_collapsed sets it true for
  // every loaded tree, so collapse then SKIPPED every sub-tree that gained data: its leaves kept the
  // new chunk as a second subset instead of merging it in. Measured on two inputs added in two
  // sessions -- 80 leaf subsets where one session produces 40, with 40 leaves holding two each.
  // That also shifts the LOD sampling, because find_indices_to_quantize runs per subset.
  tree->leaves_collapsed = false;
  assert(tree->id.data < tree_cache.current_id);
  assert(current_level != 0 || tree->morton_min == min);
  assert(tree->mins[current_level][skip] == min);
  assert(tree->node_ids[current_level][skip] == current_name);
  assert(skip == 0 || tree->mins[current_level][skip - 1] < tree->mins[current_level][skip]);
  assert(int(tree->mins[current_level].size() - 1) == skip || tree->mins[current_level][skip] < tree->mins[current_level][skip + 1]);

  auto &node = tree->nodes[current_level][skip];
  int lod = morton::morton_tree_level_to_lod(tree->magnitude, current_level);
  auto child_mask = morton::morton_get_child_mask(lod, points.min);
  assert(child_mask < 8);
  assert(!(points.min < min));
  assert(!(morton::morton_or(min, morton::morton_mask_create<uint64_t, 3>(lod)) < points.max));
  assert(morton::get_name_from_morton(lod, points.min) == current_name);
  assert(morton::get_name_from_morton(lod, tree->mins[current_level][skip]) == current_name);
  if (lod > points.min_lod)
  {
    morton::morton192_t new_min = min;
    morton::morton_set_child_mask(lod, child_mask, new_min);
    int sub_skip = tree->skips[current_level][skip] + sub_tree_count_skips(node, child_mask);
    if (node & (1 << child_mask))
    {
      if (current_level == 4)
      {
        uint16_t sub_tree_name = morton::morton_get_name(0, 0, child_mask);
        auto sub_tree_id = tree->sub_trees[sub_skip];
        auto *sub_tree = tree_cache.get(sub_tree_id);
        move_storage_locations_to_subtree(tree_cache, points, *tree, *sub_tree);
        sub_tree_insert_points(tree_cache, cache, sub_tree_id, new_min, 0, 0, sub_tree_name, std::move(points));
      }
      else
      {
        auto child_name = morton::morton_get_name(current_name, current_level + 1, child_mask);
        sub_tree_insert_points(tree_cache, cache, tree_id, new_min, current_level + 1, sub_skip, child_name, std::move(points));
      }
      return;
    }
    else if (node)
    {
      node |= uint8_t(1) << child_mask;

      if (current_level == 4)
      {
        auto &sub_tree = tree_cache_add_tree(tree_cache, tree);
        tree->sub_trees.emplace(tree->sub_trees.begin() + sub_skip, sub_tree.id);
        sub_tree_increase_skips(*tree, current_level, skip);
        tree_initialize_sub(*tree, tree_cache, points.min, sub_tree);
        uint16_t sub_tree_name = morton::morton_get_name(0, 0, child_mask);
        move_storage_locations_to_subtree(tree_cache, points, *tree, sub_tree);
        sub_tree_insert_points(tree_cache, cache, sub_tree.id, new_min, 0, 0, sub_tree_name, std::move(points));
      }
      else
      {
        sub_tree_alloc_children(*tree, current_level + 1, sub_skip);
        sub_tree_increase_skips(*tree, current_level, skip);
        auto child_name = morton::morton_get_name(current_name, current_level + 1, child_mask);
        tree->node_ids[current_level + 1][sub_skip] = child_name;
#ifndef NDEBUG
        tree->mins[current_level + 1][sub_skip] = new_min;
        assert((new_min.data[0] & 1) == 0);
#endif
        sub_tree_insert_points(tree_cache, cache, tree_id, new_min, current_level + 1, sub_skip, child_name, std::move(points));
      }
      return;
    }
  }

  if (node == 0 && tree->data[current_level][skip].point_count + points.point_count <= tree_cache.node_limit)
  {
    assert(!(points.min < min));
    points_data_add(tree->data[current_level][skip], std::move(points));
    return;
  }
  assert(points.point_count);
  assert(tree->magnitude > 0 || current_level < 4);
  {
    points_collection_t children_data[8];
    if (!node && tree->data[current_level][skip].point_count)
    {
      sub_tree_split_points_to_children(cache, tree->storage_map, std::move(tree->data[current_level][skip]), lod, min, children_data);
      tree->data[current_level][skip].point_count = 0;
    }
    if (points.point_count)
    {
      sub_tree_split_points_to_children(cache, tree->storage_map, std::move(points), lod, min, children_data);
    }

    int child_count = 0;
    for (int i = 0; i < 8; i++)
    {
      auto &child_data = children_data[i];
      const bool has_this_child = node & (1 << i);
      if (child_data.data.empty())
      {
        if (has_this_child)
          child_count++;
        continue;
      }
      morton::morton192_t new_min = min;
      morton::morton_set_child_mask(lod, uint8_t(i), new_min);
      assert(child_data.min_lod <= lod);

      tree = tree_cache.get(tree_id);
      int sub_skip = tree->skips[current_level][skip] + child_count;

      if (current_level == 4)
      {
        uint16_t sub_tree_name = morton::morton_get_name(0, 0, i);
        if (!has_this_child)
        {
          node |= uint8_t(1) << i;
          auto &sub_tree = tree_cache_add_tree(tree_cache, tree);
          tree->sub_trees.emplace(tree->sub_trees.begin() + sub_skip, sub_tree.id);
          sub_tree_increase_skips(*tree, current_level, skip);
          tree_initialize_sub(*tree, tree_cache, child_data.min, sub_tree);
          move_storage_locations_to_subtree(tree_cache, child_data, *tree, sub_tree);
          sub_tree_insert_points(tree_cache, cache, sub_tree.id, new_min, 0, 0, sub_tree_name, std::move(child_data));
        }
        else
        {
          tree_t *sub_tree = tree_cache.get(tree->sub_trees[sub_skip]);
          move_storage_locations_to_subtree(tree_cache, child_data, *tree, *sub_tree);
          sub_tree_insert_points(tree_cache, cache, sub_tree->id, new_min, 0, 0, sub_tree_name, std::move(child_data));
        }
      }
      else
      {
        auto child_name = morton::morton_get_name(current_name, current_level + 1, i);
        if (has_this_child)
        {
          sub_tree_insert_points(tree_cache, cache, tree_id, new_min, current_level + 1, sub_skip, child_name, std::move(child_data));
        }
        else
        {
          node |= uint8_t(1) << i;
          sub_tree_alloc_children(*tree, current_level + 1, sub_skip);
          sub_tree_increase_skips(*tree, current_level, skip);
          tree->node_ids[current_level + 1][sub_skip] = child_name;
#ifndef NDEBUG
          tree->mins[current_level + 1][sub_skip] = new_min;
          assert((new_min.data[0] & 1) == 0);
#endif
          sub_tree_insert_points(tree_cache, cache, tree_id, new_min, current_level + 1, sub_skip, child_name, std::move(child_data));
        }
      }
      child_count++;
    }
  }
}

static void insert_tree_in_tree(tree_registry_t &tree_registry, tree_id_t &parent_id, const tree_id_t &child_id)
// NOLINT(*-no-recursion)
{
  tree_t *parent = tree_registry.get(parent_id);
  tree_t *child = tree_registry.get(child_id);
  assert(memcmp(morton::morton_and(morton::morton_negate(morton::morton_xor(parent->morton_min, parent->morton_max)), child->morton_min).data, parent->morton_min.data, sizeof(parent->morton_min)) == 0);
  int current_skip = 0;
  int lod = morton::morton_magnitude_to_lod(parent->magnitude);

  auto current_name = parent->node_ids[0][0];
  [[maybe_unused]] morton::morton192_t new_min = parent->morton_min; // only read inside #ifndef NDEBUG below
  for (int i = 0; i < 4; i++, lod--)
  {
    auto &node = parent->nodes[i][current_skip];
    auto child_mask = morton::morton_get_child_mask(lod, child->morton_min);
    int node_skips = sub_tree_count_skips(node, child_mask);
    if (node & (1 << child_mask))
    {
      current_skip = parent->skips[i][current_skip] + node_skips;
    }
    else
    {
      node |= 1 << child_mask;
      int new_child_skip = parent->skips[i][current_skip] + node_skips;
      sub_tree_alloc_children(*parent, i + 1, new_child_skip);
      sub_tree_increase_skips(*parent, i, current_skip);
      current_name = morton::morton_get_name(current_name, i + 1, child_mask);
      parent->node_ids[i + 1][new_child_skip] = current_name;
#ifndef NDEBUG
      morton::morton_set_child_mask(lod, child_mask, new_min);
      parent->mins[i + 1][new_child_skip] = new_min;
      assert((new_min.data[0] & 1) == 0);
#endif
      current_skip = new_child_skip;
    }
  }

  auto &node = parent->nodes[4][current_skip];
  auto child_mask = morton::morton_get_child_mask(lod, child->morton_min);
  int node_skips = sub_tree_count_skips(node, child_mask);
  int current_skip_and_node = current_skip + node_skips;
  if (node & (1 << child_mask))
  {
    auto *sub_tree = tree_registry.get(parent->sub_trees[current_skip_and_node]);
    insert_tree_in_tree(tree_registry, sub_tree->id, child->id);
  }
  else
  {
    node |= 1 << child_mask;
    if (parent->magnitude - 1 == child->magnitude)
    {
      parent->sub_trees.emplace(parent->sub_trees.begin() + current_skip_and_node, child_id);
      sub_tree_increase_skips(*parent, 4, current_skip);
    }
    else
    {
      auto &sub_tree = tree_cache_add_tree(tree_registry, parent);
      parent = tree_registry.get(parent_id);
      child = tree_registry.get(child_id);
      parent->sub_trees.emplace(parent->sub_trees.begin() + current_skip_and_node, sub_tree.id);
      sub_tree_increase_skips(*parent, 4, current_skip);
      tree_initialize_sub(*parent, tree_registry, child->morton_min, sub_tree);
      insert_tree_in_tree(tree_registry, sub_tree.id, child->id);
    }
  }
}

static tree_id_t reparent_tree(tree_registry_t &tree_registry, tree_id_t tree_id, const morton::morton192_t &possible_min, const morton::morton192_t &possible_max)
{
  tree_t *tree = tree_registry.get(tree_id);
  tree_t &new_parent = tree_cache_add_tree(tree_registry, tree);
  tree_initialize_new_parent(*tree, possible_min, possible_max, new_parent);
  assert(new_parent.magnitude != tree->magnitude);

  insert_tree_in_tree(tree_registry, new_parent.id, tree->id);
  return new_parent.id;
}

tree_id_t tree_add_points(tree_registry_t &tree_registry, storage_handler_t &cache, const tree_id_t &tree_id, const storage_header_t &header, attributes_id_t attributes_id, std::vector<storage_location_t> &&locations)
{
  tree_id_t ret = tree_id;
  auto *tree = tree_registry.get(tree_id);
  // assert(validate_points_offset(header));
  if (header.morton_min < tree->morton_min || header.morton_max > tree->morton_max)
  {
    ret = reparent_tree(tree_registry, tree_id, header.morton_min, header.morton_max);
    tree = tree_registry.get(ret);
  }

  points_collection_t points_data;
  points_data_initialize(points_data, header);
  auto min = tree->morton_min;
  uint16_t name = morton::morton_get_name(0, 0, morton::morton_get_child_mask(morton::morton_magnitude_to_lod(tree->magnitude) + 1, points_data.min));
  assert(name == tree->node_ids[0][0]);
  tree->storage_map.add_storage(header.input_id, attributes_id, std::move(locations));
  // Registry-global chunk lifetime: this tree's map now holds the chunk unit. Subtree moves
  // adjust the count; collapse frees the chunk's blobs when it drops to zero.
  if (input_data_id_is_leaf(header.input_id) && !input_data_id_is_collapsed_leaf(header.input_id))
    tree_registry.chunk_tree_refs[header.input_id] = {1, uint32_t(header.point_count)};
  tree->leaves_collapsed = false; // fresh subset-shaped leaf data
  sub_tree_insert_points(tree_registry, cache, tree->id, min, 0, 0, name, std::move(points_data));
  return ret;
}
} // namespace dew::converter
