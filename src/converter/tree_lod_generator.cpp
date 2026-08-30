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
#include "tree_lod_generator.hpp"

#include "attributes_configs.hpp"
#include "input_header.hpp"
#include "lod_quantize.hpp"
#include "morton.hpp"
#include "morton_tree_coordinate_transform.hpp"
#include "storage_handler.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fixed_size_vector.hpp>
#include <algorithm>
#include <numeric>
#include <random>

namespace dew::converter
{
using namespace dew::core;

struct children_subset_t
{
  std::vector<points_subset_t> data;
  std::vector<int> data_skips;
  std::vector<int> skips;
  std::vector<int> lods;
  std::vector<tree_id_t> tree_id;
};

static input_data_id_t get_next_input_id(tree_registry_t &tree_cache)
{
  input_data_id_t ret; // NOLINT(*-pro-type-member-init)
  static_assert(sizeof(ret) == sizeof(tree_cache.current_lod_node_id), "input_data_id_t is incompatible with tree_registry_t::current_lod_node_id");
  memcpy(&ret, &tree_cache.current_lod_node_id, sizeof(ret));
  tree_cache.current_lod_node_id++;
  return ret;
}

std::pair<int, int> find_missing_lod(tree_registry_t &tree_cache, storage_handler_t &cache, tree_id_t tree_id, const morton::morton192_t &min, const morton::morton192_t &max, const morton::morton192_t &parent_min,
                                     const morton::morton192_t &parent_max, int current_level, int skip, children_subset_t &to_lod) // NOLINT(*-no-recursion)
{
  auto tree = tree_cache.get(tree_id);
  assert(skip < int(tree->nodes[current_level].size()));
  auto &node = tree->nodes[current_level][skip];
  assert(node || (tree->data[current_level][skip].point_count > 0 && tree->data[current_level][skip].point_count < uint64_t(-1)));
  int lod = morton::morton_tree_level_to_lod(tree->magnitude, current_level);
  if (!node)
  {
    const auto &data = tree->data[current_level][skip];
    assert(data.data.size());
    to_lod.data.insert(to_lod.data.end(), data.data.cbegin(), data.data.cend());
    int to_ret = int(data.data.size());
    to_lod.data_skips.push_back(to_ret);
    to_lod.skips.push_back(1);
    to_lod.lods.push_back(lod);
    return std::make_pair(1, to_ret);
  }

  int skip_index = 0;
  auto ret_pair = std::make_pair(0, 0);
  if (min <= parent_min && parent_max <= max)
  {
    auto &node_data = tree->data[current_level][skip];
    assert(node_data.data.size() <= 1);
    if (node_data.data.size() == 1)
    {
      assert(node_data.data.back().offset.data == (~uint32_t(0)));
      to_lod.data.emplace_back(node_data.data.back());
      to_lod.data_skips.emplace_back(1);
      to_lod.skips.emplace_back(1);
      to_lod.lods.push_back(lod);
      return std::make_pair(1, 1);
    }
    skip_index = int(to_lod.skips.size());
    node_data.data.emplace_back(get_next_input_id(tree_cache), offset_in_subset_t(~uint32_t(0)), point_count_t(0));
    node_data.min = parent_min;
    node_data.max = parent_max;
    to_lod.data.emplace_back(node_data.data.back());
    to_lod.data_skips.emplace_back(1);
    to_lod.skips.emplace_back(1);
    to_lod.lods.push_back(lod);
    ret_pair = std::make_pair(1, 1);
  }
  int child_count = 0;
  int sub_skip_parent = tree->skips[current_level][skip];
  for (int i = 0; i < 8; i++)
  {
    const bool has_this_child = node & (1 << i);
    if (has_this_child)
    {
      child_count++;
      morton::morton192_t child_min = parent_min;
      morton::morton_set_child_mask(lod, uint8_t(i), child_min);
      if (max < child_min)
        break;
      morton::morton192_t child_max = parent_max;
      morton::morton_set_child_mask(lod, uint8_t(i), child_max);
      if (child_max < min)
        continue;
      int sub_skip = sub_skip_parent + child_count - 1;
      std::pair<int, int> adjust = {};
      if (current_level == 4)
      {
        assert(sub_skip < int(tree->sub_trees.size()));
        tree_t *sub_tree = tree_cache.get(tree->sub_trees[sub_skip]);
        adjust = find_missing_lod(tree_cache, cache, sub_tree->id, min, max, child_min, child_max, 0, 0, to_lod);
      }
      else
      {
        adjust = find_missing_lod(tree_cache, cache, tree_id, min, max, child_min, child_max, current_level + 1, sub_skip, to_lod);
      }
      ret_pair.first += adjust.first;
      ret_pair.second += adjust.second;
      if (ret_pair.first > 0)
      {
        to_lod.skips[skip_index] += adjust.first;
        to_lod.data_skips[skip_index] += adjust.second;
      }
    }
  }
  return ret_pair;
}

static lod_tree_worker_data_t make_tree_worker_data(const tree_t &tree)
{
  lod_tree_worker_data_t ret;
  ret.tree_id = tree.id;
  ret.magnitude = tree.magnitude;
  for (int i = 0; i < 5; i++)
  {
    ret.nodes[i].reserve(tree.skips[i].size());
  }
  return ret;
}

struct tree_iterator_t
{
  tree_iterator_t(size_t capasity)
  {
    parent_indecies.reserve(capasity);
    skips.reserve(capasity);
    names.reserve(capasity);
    parents.reserve(capasity / 2);
  }

  void clear()
  {
    parent_indecies.clear();
    skips.clear();
    names.clear();
    parents.clear();
  }
  std::vector<uint16_t> parent_indecies;
  std::vector<uint16_t> skips;
  std::vector<uint16_t> names;
  std::vector<lod_node_worker_data_t> parents;
};

static void tree_get_work_items(tree_registry_t &tree_cache, storage_handler_t &cache, tree_id_t &tree_id, lod_node_worker_data_t &parent_node, std::vector<lod_tree_worker_data_t> &to_lod,
                                const morton::morton192_t &max_morton, const morton::morton192_t &already_lod_morton)
{
  auto tree = tree_cache.get(tree_id);
  assert(tree->morton_min >= parent_node.node_min);
  auto lod_tree_worker_data = make_tree_worker_data(*tree);

  if (tree->nodes->empty())
    return;

  bool buffer_index = false;

  size_t capasity = std::max(tree->sub_trees.size(), std::max(std::max(tree->skips[4].size(), tree->skips[3].size()), tree->skips[2].size()));
  tree_iterator_t tree_iterator[2] = {capasity, capasity};

  auto *parent_buffer = &parent_node;
  tree_iterator[buffer_index].parent_indecies.emplace_back(0);
  tree_iterator[buffer_index].skips.emplace_back(0);
  uint16_t root_name = morton::morton_get_child_mask(morton::morton_magnitude_to_lod(tree->magnitude), tree->morton_min);
  tree_iterator[buffer_index].names.emplace_back(root_name << 4 * 3);

  for (int level = 0; level < 5 && tree_iterator[buffer_index].parent_indecies.size(); level++)
  {
    for (int to_process_index = 0; to_process_index < int(tree_iterator[buffer_index].skips.size()); to_process_index++)
    {
      auto tree_skip = tree_iterator[buffer_index].skips[to_process_index];
      auto node = tree->nodes[level][tree_skip];
      auto &parent = parent_buffer[tree_iterator[buffer_index].parent_indecies[to_process_index]];
      auto &data = tree->data[level][tree_skip];
      auto name = tree_iterator[buffer_index].names[to_process_index];
      morton::morton192_t node_min = morton::set_name_in_morton(tree->magnitude, tree->morton_min, name);
      assert(!(node_min < parent.node_min));
#ifndef NDEBUG
      auto min_from_mins = tree->mins[level][tree_skip];
      auto max_from_mins = morton::create_max(morton::morton_tree_level_to_lod(tree->magnitude, level), min_from_mins);
      assert(min_from_mins == node_min);
      morton::morton192_t parent_max = morton::create_max(morton::morton_tree_level_to_lod(tree->magnitude + (level == 0), level == 0 ? 4 : level - 1), node_min);
      morton::morton192_t node_max_debug = morton::create_max(morton::morton_tree_level_to_lod(tree->magnitude, level), node_min);
      assert(max_from_mins == node_max_debug);
      assert(!(parent_max < node_max_debug));
#endif
      {
        int lod = morton::morton_tree_level_to_lod(tree->magnitude, level);
        morton::morton192_t node_max = morton::create_max(lod, node_min);
        // Node's range is not strictly below the done boundary — skip entirely
        if (!(node_max < max_morton))
          continue;
        // Node fully within already-LODed range AND has LOD data — skip but add to parent
        if (node && node_max < already_lod_morton && data.point_count > 0 && data.data.size() == 1)
        {
          parent.child_data.push_back(data);
          parent.child_trees.push_back(tree_id);
          continue;
        }
      }
      if (node)
      {
        assert(data.data.size() <= 1);
        if (data.data.empty())
        {
          data.data.emplace_back(get_next_input_id(tree_cache), offset_in_subset_t(~uint32_t(0)), point_count_t(0));
        }
        uint16_t parent_index = uint16_t(tree_iterator[!buffer_index].parents.size());
        tree_iterator[!buffer_index].parents.emplace_back();
        auto &this_node = tree_iterator[!buffer_index].parents.back();
        this_node.id = tree->node_ids[level][tree_skip];
        this_node.lod = uint16_t(morton::morton_tree_level_to_lod(tree->magnitude, level));
        this_node.node_min = node_min;
        this_node.storage_name = data.data.front().input_id;
        this_node.generated_point_count.data = 0;
        int child_count = 0;
        uint16_t parent_node_name = level == 4 ? 0 : this_node.id << 3;
        int node_name_level = level == 4 ? 0 : level;
        for (int child_index = 0; child_index < 8; child_index++)
        {
          if (!(node & uint8_t(1 << child_index)))
            continue;
          auto &sub_level = tree_iterator[!buffer_index];
          sub_level.names.push_back(morton::morton_get_name(parent_node_name, node_name_level, child_index));
          sub_level.skips.push_back(uint16_t(tree->skips[level][tree_skip] + child_count));
          sub_level.parent_indecies.push_back(parent_index);
          child_count++;
        }
      }
      if (data.data.size())
      {
        parent.child_data.push_back(data);
        parent.child_trees.push_back(tree_id);
      }
    }

    if (level > 0)
    {
      lod_tree_worker_data.nodes[level - 1] = std::move(tree_iterator[buffer_index].parents);
    }
    tree_iterator[buffer_index].clear();
    buffer_index = !buffer_index;
    parent_buffer = tree_iterator[buffer_index].parents.data();
  }
  // sub-trees
  for (int to_process_index = 0; to_process_index < int(tree_iterator[buffer_index].skips.size()); to_process_index++)
  {
    auto tree_skip = tree_iterator[buffer_index].skips[to_process_index];
    auto subtree_id = tree->sub_trees[tree_skip];
    auto &parent = parent_buffer[tree_iterator[buffer_index].parent_indecies[to_process_index]];
    tree_get_work_items(tree_cache, cache, subtree_id, parent, to_lod, max_morton, already_lod_morton);
  }
  lod_tree_worker_data.nodes[4] = std::move(tree_iterator[buffer_index].parents);
  if (lod_tree_worker_data.nodes[0].size())
    to_lod.push_back(std::move(lod_tree_worker_data));
}

lod_worker_t::lod_worker_t(tree_lod_generator_t &a_lod_generator, lod_worker_batch_t &a_batch, uint32_t a_tree_index, storage_handler_t &a_cache, attributes_configs_t &a_attributes_configs,
                           lod_node_worker_data_t &a_data,
                           const std::vector<float> &a_random_offsets)
  : lod_generator(a_lod_generator)
  , batch(a_batch)
  , tree_index(a_tree_index)
  , cache(a_cache)
  , attributes_configs(a_attributes_configs)
  , data(a_data)
  , random_offsets(a_random_offsets)
{
}

template <typename S_M, typename D_M>
static typename std::enable_if<sizeof(S_M) == sizeof(D_M), void>::type copy_morton(const S_M &s, const morton::morton192_t &morton_min, const morton::morton192_t &morton_max, D_M &d)
{
  (void)morton_min;
  memcpy(&d, &s, sizeof(d));
#ifndef NDEBUG
  S_M morton_min_downcasted;
  morton::morton_downcast(morton_min, morton_min_downcasted);
  assert(!(s < morton_min_downcasted));
  S_M morton_max_downcasted;
  morton::morton_downcast(morton_max, morton_max_downcasted);
  assert(!(morton_max_downcasted < s));
#endif
}

template <size_t A, size_t B>
struct less_than
{
  enum the_value
  {
    value = A < B
  };
};
template <size_t A, size_t B>
struct greater_than
{
  enum the_value
  {
    value = A > B
  };
};

template <typename S_M, typename D_M>
static typename std::enable_if<less_than<sizeof(S_M), sizeof(D_M)>::value, void>::type copy_morton(const S_M &s, const morton::morton192_t &morton_min, const morton::morton192_t &morton_max, D_M &d)
{
  (void)morton_min;
  (void)morton_max;
  morton::morton_upcast(s, morton_min, d);
}

template <typename S_M, typename D_M>
static typename std::enable_if<greater_than<sizeof(S_M), sizeof(D_M)>::value, void>::type copy_morton(const S_M &s, const morton::morton192_t &morton_min, const morton::morton192_t &morton_max, D_M &d)
{
  (void)morton_min;
  (void)morton_max;
  morton::morton_downcast(s, d);
#ifndef NDEBUG
  S_M upcasted;
  morton::morton_upcast(d, morton_min, upcasted);
  assert(upcasted == s);

  for (int i = int(std::size(d.data)); i < int(std::size(s.data)); i++)
  {
    assert(s.data[i] == morton_min.data[i]);
  }
#endif
}

// morton_to_lod_t + find_indices_to_quantize moved to lod_quantize.hpp (shared with the renderer's virtual
// LOD so a virtual LOD node is generated by the exact same scheme as a stored one).
template <typename S_M, typename D_M>
static void quantize_morton_two(const morton::morton192_t &morton_min, const morton::morton192_t &morton_max, dew_type_t source_type, const std::vector<uint32_t> &indecies_to_quantize, const dew_blob_t &source,
                                dew_type_t destination_type, dew_blob_t &destination)
{
  (void)source_type;
  (void)destination_type;
  auto *source_it = reinterpret_cast<const S_M *>(source.data);
  auto *destination_it = reinterpret_cast<D_M *>(destination.data);
  assert(source.size % sizeof(S_M) == 0);
  assert(destination.size % sizeof(D_M) == 0);
  assert(indecies_to_quantize.back() < source.size / sizeof(S_M));
  for (uint32_t i = 0; i < uint32_t(indecies_to_quantize.size()); i++)
  {
    copy_morton<S_M, D_M>(source_it[indecies_to_quantize[i]], morton_min, morton_max, destination_it[i]);
  }
}

template <typename S_M>
static void quantize_morton_one(const morton::morton192_t &morton_min, const morton::morton192_t &morton_max, dew_type_t source_type, const std::vector<uint32_t> &indecies_to_quantize, const dew_blob_t &source,
                                dew_type_t destination_type, dew_blob_t &destination)
{
  assert(destination_type == dew_type_m32 || destination_type == dew_type_m64 || destination_type == dew_type_m128 || destination_type == dew_type_m192);
  switch (destination_type)
  {
  case dew_type_m32:
    quantize_morton_two<S_M, morton::morton32_t>(morton_min, morton_max, source_type, indecies_to_quantize, source, destination_type, destination);
    break;
  case dew_type_m64:
    quantize_morton_two<S_M, morton::morton64_t>(morton_min, morton_max, source_type, indecies_to_quantize, source, destination_type, destination);
    break;
  case dew_type_m128:
    quantize_morton_two<S_M, morton::morton128_t>(morton_min, morton_max, source_type, indecies_to_quantize, source, destination_type, destination);
    break;
  case dew_type_m192:
    quantize_morton_two<S_M, morton::morton192_t>(morton_min, morton_max, source_type, indecies_to_quantize, source, destination_type, destination);
    break;
  default:
    break;
  }
}

dew_blob_t morton_buffer_for_subset(const dew_blob_t &buffer, dew_type_t format, offset_in_subset_t offset, point_count_t count)
{
  auto format_byte_size = size_for_format(format);
  dew_blob_t ret;
  auto offset_bytes = offset.data * format_byte_size;
  ret.data = ((uint8_t *)buffer.data) + offset_bytes;
  ret.size = count.data * format_byte_size;
  assert(ret.size + offset.data <= buffer.size);
  return ret;
}

#ifndef NDEBUG
static const void *buffer_end(const dew_blob_t &buffer)
{
  return ((const uint8_t *)buffer.data) + buffer.size;
}
static bool buffer_is_subset(const dew_blob_t &super, const dew_blob_t &sub)
{
  return super.data <= sub.data && buffer_end(sub) <= buffer_end(super);
}
#endif

template <typename T, size_t S>
struct structured_data_t
{
  T data[S];
};

template <size_t S_S, typename S, size_t D_S, typename D>
void convert_points_impl(input_data_id_t input_id, const std::vector<std::pair<input_data_id_t, uint32_t>> &indecies, const point_format_t &source_format, const dew_blob_t &source, const point_format_t &destination_format,
                         dew_blob_t &destination)
{
  (void)source_format;
  (void)destination_format;
  using source_data_type_t = structured_data_t<S, S_S>;
  using destination_data_type_t = structured_data_t<D, D_S>;
  auto *source_begin = reinterpret_cast<const source_data_type_t *>(source.data);
  auto *destination_begin = reinterpret_cast<destination_data_type_t *>(destination.data);
  // assert(source.size % sizeof(source_data_type_t) == 0);
  assert(destination.size % sizeof(destination_data_type_t) == 0);
  assert(indecies.size() <= destination.size / sizeof(destination_data_type_t));

  constexpr auto copy_components = std::min(S_S, D_S);
  for (uint32_t destination_index = 0; destination_index < uint32_t(indecies.size()); destination_index++)
  {
    if (indecies[destination_index].first != input_id)
    {
      continue;
    }

    auto source_index = indecies[destination_index].second;
    for (int i = 0; i < int(copy_components); i++)
    {
      destination_begin[destination_index].data[i] = D(source_begin[source_index].data[i]);
    }
    for (int i = int(copy_components); i < int(D_S); i++)
    {
      destination_begin[destination_index].data[i] = D();
    }
  }
}

template <size_t S_S, typename S, size_t D_S>
void convert_points_three(input_data_id_t input_id, const std::vector<std::pair<input_data_id_t, uint32_t>> &indecies, const point_format_t &source_format, const dew_blob_t &source,
                          const point_format_t &destination_format, dew_blob_t &destination)
{
  switch (destination_format.type)
  {
  case dew_type_u8:
    convert_points_impl<S_S, S, D_S, uint8_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_i8:
    convert_points_impl<S_S, S, D_S, int8_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_u16:
    convert_points_impl<S_S, S, D_S, uint16_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_i16:
    convert_points_impl<S_S, S, D_S, int16_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_u32:
    convert_points_impl<S_S, S, D_S, uint32_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_i32:
    convert_points_impl<S_S, S, D_S, int32_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_r32:
    convert_points_impl<S_S, S, D_S, float>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_u64:
    convert_points_impl<S_S, S, D_S, uint64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_i64:
    convert_points_impl<S_S, S, D_S, int64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_r64:
    convert_points_impl<S_S, S, D_S, double>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_m32:
    convert_points_impl<S_S, S, 1, uint32_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_m64:
    convert_points_impl<S_S, S, 1, uint64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_m128:
    convert_points_impl<S_S, S, 2, uint64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_m192:
    convert_points_impl<S_S, S, 3, uint64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  }
}

template <size_t S_S, typename S>
void convert_points_two(input_data_id_t input_id, const std::vector<std::pair<input_data_id_t, uint32_t>> &indecies, const point_format_t &source_format, const dew_blob_t &source, const point_format_t &destination_format,
                        dew_blob_t &destination)
{
  switch (destination_format.components)
  {
  case dew_components_1:
    convert_points_three<S_S, S, 1>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_components_2:
    convert_points_three<S_S, S, 2>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_components_3:
    convert_points_three<S_S, S, 3>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_components_4:
    convert_points_three<S_S, S, 4>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_components_4x4:
    convert_points_three<S_S, S, 4 * 4>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  }
}
template <size_t S_S>
void convert_points_one(input_data_id_t input_id, const std::vector<std::pair<input_data_id_t, uint32_t>> &indecies, const point_format_t &source_format, const dew_blob_t &source, const point_format_t &destination_format,
                        dew_blob_t &destination)
{
  switch (source_format.type)
  {
  case dew_type_u8:
    convert_points_two<S_S, uint8_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_i8:
    convert_points_two<S_S, int8_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_u16:
    convert_points_two<S_S, uint16_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_i16:
    convert_points_two<S_S, int16_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_u32:
    convert_points_two<S_S, uint32_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_i32:
    convert_points_two<S_S, int32_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_r32:
    convert_points_two<S_S, float>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_u64:
    convert_points_two<S_S, uint64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_i64:
    convert_points_two<S_S, int64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_r64:
    convert_points_two<S_S, double>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_m32:
    convert_points_two<1, uint32_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_m64:
    convert_points_two<1, uint64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_m128:
    convert_points_two<2, uint64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  case dew_type_m192:
    convert_points_two<3, uint64_t>(input_id, indecies, source_format, source, destination_format, destination);
    break;
  }
}

static void copy_attribute_for_input(input_data_id_t input_id, const std::vector<std::pair<input_data_id_t, uint32_t>> &indecies, const point_format_t &source_format, const dew_blob_t &source_buffer,
                                     const point_format_t &target_format, dew_blob_t &target_buffer)
{
  switch (source_format.components)
  {
  case dew_components_1:
    convert_points_one<1>(input_id, indecies, source_format, source_buffer, target_format, target_buffer);
    break;
  case dew_components_2:
    convert_points_one<2>(input_id, indecies, source_format, source_buffer, target_format, target_buffer);
    break;
  case dew_components_3:
    convert_points_one<3>(input_id, indecies, source_format, source_buffer, target_format, target_buffer);
    break;
  case dew_components_4:
    convert_points_one<4>(input_id, indecies, source_format, source_buffer, target_format, target_buffer);
    break;
  case dew_components_4x4:
    convert_points_one<4 * 4>(input_id, indecies, source_format, source_buffer, target_format, target_buffer);
    break;
  }
}

void quantize_attributres(storage_handler_t &cache, const child_storage_map_t &child_storage_map, const std::vector<std::pair<input_data_id_t, uint32_t>> &indecies,
                          const attribute_lod_mapping_t &lod_attrib_mapping, attribute_buffers_t &buffers)
{
  fixed_capacity_vector_t<input_data_id_t> inputs(indecies, [](const std::pair<input_data_id_t, uint32_t> &a) { return a.first; });
  std::sort(inputs.begin(), inputs.end());
  auto inputs_end = std::unique(inputs.begin(), inputs.end());
  for (auto inputs_it = inputs.begin(); inputs_it != inputs_end; ++inputs_it)
  {
    const auto &storage_info = child_storage_map.at(*inputs_it);
    auto mapping = lod_attrib_mapping.get_source_mapping(storage_info.attributes_id);
    for (int destination_buffer_index = 1; destination_buffer_index < int(buffers.buffers.size()); destination_buffer_index++)
    {
      auto attr_mapping = mapping.source_attributes[destination_buffer_index];
      // The destination attribute set is the UNION of all child inputs' attributes. An input
      // that lacks a given unioned attribute yields source_index == -1; indexing locations[-1]
      // is out of bounds. Such an input contributes nothing to this destination buffer.
      if (attr_mapping.source_index < 0)
        continue;
      read_attribute_t source_attrib_data(cache, storage_info.locations[attr_mapping.source_index], storage_info.retain_hot);
      // A failed source read (e.g. unreachable destination for a spilled blob) already flagged the
      // conversion through the storage error pipe; skip the contribution instead of dereferencing.
      if (source_attrib_data.error.code != 0)
        continue;
      copy_attribute_for_input(*inputs_it, indecies, attr_mapping.source_format, source_attrib_data.data, lod_attrib_mapping.destination[destination_buffer_index], buffers.buffers[destination_buffer_index]);
    }
  }
}

template <typename T, size_t N>
static void quantize_subset(storage_handler_t &cache, const points_subset_t &subset, const lod_child_storage_info_t &storage_info, int lod, const std::vector<float> &random_offsets,
                            std::vector<morton_to_lod_t<T, N>> &morton_to_lod)
{
  read_only_points_t subset_data(cache, storage_info.locations[0], storage_info.retain_hot);
  // Failed read: conversion is flagged (storage error pipe); contribute nothing rather than crash.
  if (subset_data.error.code != 0)
    return;
  offset_in_subset_t offset;
  point_count_t point_count;
  if (subset.count.data == uint32_t(0))
  {
    offset = offset_in_subset_t(0);
    point_count = point_count_t(subset_data.header.point_count);
  }
  else
  {
    offset = subset.offset;
    point_count = subset.count;
  }
  const dew_blob_t source_buffer = morton_buffer_for_subset(subset_data.data, subset_data.header.point_format.type, offset, point_count);

  assert(buffer_is_subset(subset_data.data, source_buffer));
  morton::morton192_t subset_min = morton::morton_and(morton::morton_negate(morton::morton_mask_create<uint64_t, 3>(lod - 1)), subset_data.header.morton_min);
  find_indices_to_quantize(subset.input_id, subset_min, subset_data.header.point_format.type, source_buffer, offset, point_count, lod_quantize_mask_width(lod), random_offsets, morton_to_lod);
}

template <typename T, size_t N>
static void quantize_points_collection(storage_handler_t &cache, const points_collection_t &point_collection, const child_storage_map_t &child_storage_map, int lod, const std::vector<float> &random_offsets,
                                       std::vector<morton_to_lod_t<T, N>> &morton_to_lod)
{
  for (int i = 0; i < int(point_collection.data.size()); i++)
  {
    auto &subset = point_collection.data[i];
    const auto &storage = child_storage_map.at(subset.input_id);
    quantize_subset(cache, subset, storage, lod, random_offsets, morton_to_lod);
  }
}

struct calculate_child_buffer_size_t
{
  calculate_child_buffer_size_t(const std::vector<points_collection_t> &child_data)
  {
    for (auto &child : child_data)
    {
      storage_range_size += int(child.data.size());
      for (auto &subset : child.data)
      {
        morton_to_lod_size += int(subset.count.data / 8 + 1);
      }
    }
  }

  int morton_to_lod_size = 0;
  int storage_range_size = 0;
};

template <typename T, size_t N>
static void quantize_morton_remember_indecies_t(storage_handler_t &cache, const morton::morton192_t &node_min, const std::vector<points_collection_t> &child_data, const child_storage_map_t &child_storage_map, int lod,
                                                const std::vector<float> &random_offsets, bool adaptive_sampling, std::unique_ptr<uint8_t[]> &morton_data, std::vector<std::pair<input_data_id_t, uint32_t>> &indecies, morton::morton192_t &min,
                                                morton::morton192_t &max)
{
  std::vector<morton_to_lod_t<T, N>> morton_to_lod;
  int maskWidth = lod_quantize_mask_width(lod);
  {
    calculate_child_buffer_size_t child_buffer_sizes(child_data);
    morton_to_lod.reserve(child_buffer_sizes.morton_to_lod_size);
  }

  for (const auto &points_collection : child_data)
  {
    quantize_points_collection(cache, points_collection, child_storage_map, lod, random_offsets, morton_to_lod);
  }

  std::sort(morton_to_lod.begin(), morton_to_lod.end(), [](const morton_to_lod_t<T, N> &a, const morton_to_lod_t<T, N> &b) { return a.morton < b.morton; });

  // Adaptive density: the classic cell width (lod - 9) barely thins sparse data -- the finest LOD
  // levels came out as near-copies of their children and the pyramid exceeded the source size.
  // Coarsen the cell until this node keeps at most a quarter of its children's points (floored at
  // the classic width, capped by what the morton type can express), which bounds the whole pyramid
  // at ~1/3 of the source on any density. rep_level/prefix decoding is self-describing (derived
  // from the stored codes), so the renderer needs no knowledge of the chosen width.
  if (adaptive_sampling && !morton_to_lod.empty())
  {
    uint64_t child_point_total = 0;
    for (const auto &points_collection : child_data)
      child_point_total += points_collection.point_count;
    const uint64_t rep_target = std::max<uint64_t>(1, child_point_total / 4);
    constexpr int max_mask_for_type = (int(sizeof(morton::morton_t<T, N>) * 8) - 4) / 3;
    while (maskWidth < max_mask_for_type)
    {
      uint64_t cells = 1;
      auto probe_max = morton::create_max(maskWidth, morton_to_lod.front().morton);
      for (uint32_t i = 1; i < uint32_t(morton_to_lod.size()) && cells <= rep_target; i++)
      {
        if (morton_to_lod[i].morton <= probe_max)
          continue;
        cells++;
        probe_max = morton::create_max(maskWidth, morton_to_lod[i].morton);
      }
      if (cells <= rep_target)
        break;
      ++maskWidth;
    }
  }
  morton::morton_upcast(morton_to_lod.front().morton, node_min, min);
  morton::morton_upcast(morton_to_lod.back().morton, node_min, max);

  morton_data.reset(new uint8_t[sizeof(morton::morton_t<T, N>) * morton_to_lod.size()]);
  auto target_morton_buffer = reinterpret_cast<morton::morton_t<T, N> *>(morton_data.get());
  uint32_t current_target_morton_buffer_index = 0;
  indecies.reserve(morton_to_lod.size());
  uint32_t range_start = 0;
  morton::morton_t<T, N> currentMaxVal = morton::create_max(maskWidth, morton_to_lod.front().morton);
  for (uint32_t i = 1; i < uint32_t(morton_to_lod.size()); i++)
  {
    if (morton_to_lod[i].morton <= currentMaxVal)
      continue;

    auto range_size = i - range_start;
    auto index = range_start + (range_size / 2);
    target_morton_buffer[current_target_morton_buffer_index++] = morton_to_lod[index].morton;
    indecies.emplace_back(morton_to_lod[index].id, uint32_t(morton_to_lod[index].index.data));

    range_start = i;
    currentMaxVal = morton::create_max(maskWidth, morton_to_lod[i].morton);
  }
  auto index = range_start + ((morton_to_lod.size() - range_start) / 2);
  target_morton_buffer[current_target_morton_buffer_index++] = morton_to_lod[index].morton;
  indecies.emplace_back(morton_to_lod[index].id, uint32_t(morton_to_lod[index].index.data));
}

static void quantize_morton_remember_indecies(storage_handler_t &cache, const morton::morton192_t &node_min, const std::vector<points_collection_t> &child_data, const child_storage_map_t &child_storage_map, int lod,
                                              const std::vector<float> &random_offsets, bool adaptive_sampling, std::unique_ptr<uint8_t[]> &morton_data, std::vector<std::pair<input_data_id_t, uint32_t>> &indecies, morton::morton192_t &min,
                                              morton::morton192_t &max)
{
  auto lod_format = morton_type_from_lod(lod);
  switch (lod_format)
  {
  case dew_type_m32:
    quantize_morton_remember_indecies_t<uint32_t, 1>(cache, node_min, child_data, child_storage_map, lod, random_offsets, adaptive_sampling, morton_data, indecies, min, max);
    break;
  case dew_type_m64:
    quantize_morton_remember_indecies_t<uint64_t, 1>(cache, node_min, child_data, child_storage_map, lod, random_offsets, adaptive_sampling, morton_data, indecies, min, max);
    break;
  case dew_type_m128:
    quantize_morton_remember_indecies_t<uint64_t, 2>(cache, node_min, child_data, child_storage_map, lod, random_offsets, adaptive_sampling, morton_data, indecies, min, max);
    break;
  case dew_type_m192:
    quantize_morton_remember_indecies_t<uint64_t, 3>(cache, node_min, child_data, child_storage_map, lod, random_offsets, adaptive_sampling, morton_data, indecies, min, max);
    break;
  default:
    assert("This should not happen");
  }
}

namespace
{
// Where LOD workers spend their time, accumulated across every worker on every pool thread. Set
// DEW_DEBUG_LOD to print it when a pass finishes. The counters are CUMULATIVE over the run; only the
// pass wall time is per-pass.
//
// READ IT AS A RATIO. in-worker CPU divided by pass wall time is how many cores the phase actually
// kept busy, and the worker count is the most jobs the pool could ever have had to choose from. The
// first measurement it produced -- 271 workers and 121.5s of CPU across 207.5s of wall, on 70M
// points -- said the phase is limited by having too few, too coarse jobs, not by contention:
// attrib_map (the attributes_configs mutex) and write_post (handing off to the storage loop) were
// both 0.0s.
//
// It exists because three plausible explanations for the low utilisation -- the level barrier, cache
// pollution, and the reader loop -- were each argued from the code, each implemented or measured,
// and each turned out not to be it. Counting settled in one run what reading had not in three.
struct lod_probe_t
{
  std::atomic<uint64_t> workers{0};
  std::atomic<uint64_t> attrib_map_us{0}; // get_lod_attribute_mapping: takes the attributes_configs mutex
  std::atomic<uint64_t> morton_us{0};     // reading children's positions + choosing the sample
  std::atomic<uint64_t> attributes_us{0}; // reading and sampling the children's other attributes
  std::atomic<uint64_t> write_post_us{0}; // handing the result to the storage loop
  std::atomic<uint64_t> total_us{0};
};
lod_probe_t g_lod_probe;
const bool g_lod_probe_on = std::getenv("DEW_DEBUG_LOD") != nullptr;

struct scoped_us_t
{
  std::atomic<uint64_t> &sink;
  std::chrono::steady_clock::time_point start;
  explicit scoped_us_t(std::atomic<uint64_t> &s)
    : sink(s)
    , start(std::chrono::steady_clock::now())
  {
  }
  ~scoped_us_t()
  {
    sink.fetch_add(uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count()), std::memory_order_relaxed);
  }
};
} // namespace

void lod_worker_t::work()
{
  const auto worker_start = std::chrono::steady_clock::now();
  dew_attributes_t attributes;
  std::unique_ptr<attributes_id_t[]> attribute_ids(new attributes_id_t[data.child_storage_info.size()]);
  int child_data_count = 0;
  for (auto const &storage_info : data.child_storage_info)
  {
    attribute_ids[child_data_count++] = storage_info.second.attributes_id;
  }

  auto lod_format = morton_type_from_lod(data.lod);
  const auto &generation_config = lod_generator.generation_tree_config();
  auto lod_attrib_mapping = [&] {
    scoped_us_t timer(g_lod_probe.attrib_map_us);
    return attributes_configs.get_lod_attribute_mapping(data.lod, attribute_ids.get(), attribute_ids.get() + data.child_storage_info.size(),
                                                        /*keep_original_order=*/false, generation_config.lod_all_attributes != 0);
  }();

  storage_header_t destination_header;
  storage_header_initialize(destination_header);
  destination_header.input_id = data.storage_name;
  attribute_buffers_t buffers;

  std::vector<std::pair<input_data_id_t, uint32_t>> indecies;
  {
    scoped_us_t timer(g_lod_probe.morton_us);
    std::unique_ptr<uint8_t[]> morton_attribute_buffer;
    quantize_morton_remember_indecies(cache, data.node_min, data.child_data, data.child_storage_info, data.lod, random_offsets, generation_config.lod_adaptive_sampling != 0, morton_attribute_buffer, indecies, destination_header.morton_min,
                                      destination_header.morton_max);
    attribute_buffers_initialize(lod_attrib_mapping.destination, buffers, uint32_t(indecies.size()), std::move(morton_attribute_buffer));
  }

  {
    scoped_us_t timer(g_lod_probe.attributes_us);
    quantize_attributres(cache, data.child_storage_info, indecies, lod_attrib_mapping, buffers);
  }

  assert(!indecies.empty());

  attribute_buffers_adjust_buffers_to_size(lod_attrib_mapping.destination, buffers, uint32_t(indecies.size()));
  destination_header.point_count = uint32_t(indecies.size());
  destination_header.point_format = {lod_format, dew_components_1};
  destination_header.lod_span = data.lod;
  // PUBLISHED BEFORE THE WRITE IS POSTED, not after. cache.write's completion callback runs on the
  // storage loop and calls add_worker_done, which wakes the tree loop to run adjust_tree_after_lod --
  // and that reads exactly these three fields. Setting them after the post left nothing ordering the
  // pool thread's stores against the tree loop's reads; only the latency of compress-allocate-write
  // kept the window from ever being observed.
  data.generated_point_count.data = uint32_t(indecies.size());
  data.generated_min = destination_header.morton_min;
  data.generated_max = destination_header.morton_max;
  scoped_us_t write_timer(g_lod_probe.write_post_us);
  cache.write(destination_header, lod_attrib_mapping.destination_id, std::move(buffers),
              [this](const storage_header_t &storageheader, attributes_id_t attrib_id, std::vector<storage_location_t> locations, const dew_error_t &error)
              {
                (void)storageheader;
                (void)error;
                this->data.generated_attributes_id = attrib_id;
                this->data.generated_locations = std::move(locations);
                this->lod_generator.add_worker_done(this->batch, this->tree_index);
              });
  g_lod_probe.workers.fetch_add(1, std::memory_order_relaxed);
  g_lod_probe.total_us.fetch_add(uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - worker_start).count()), std::memory_order_relaxed);
}

static void get_storage_info(tree_registry_t &tree_cache, lod_node_worker_data_t &node)
{
  for (int i = 0; i < int(node.child_data.size()); i++)
  {
    auto tree_id = node.child_trees[i];
    auto tree = tree_cache.get(tree_id);
    for (int j = 0; j < int(node.child_data[i].data.size()); j++)
    {
      auto &child_data = node.child_data[i].data[j];

      auto &storage_info = node.child_storage_info[child_data.input_id];
      if (storage_info.locations.empty())
      {
        auto info = tree->storage_map.info(child_data.input_id);
        storage_info.attributes_id = info.first;
        storage_info.locations = std::move(info.second);
        // count == 0 is the "consume the whole unit" sentinel (see quantize_subset): this parent is
        // the blob's only reader, so its cache entry is dead the moment the read returns and belongs
        // at the cold end. A real subset means sibling nodes hold the other subsets of the same blob
        // and will be back for it.
        storage_info.retain_hot = child_data.count.data != uint32_t(0);
      }
      else if (child_data.count.data != uint32_t(0))
      {
        // Two subsets of one unit reach this node: it is not a whole-unit consumer after all.
        storage_info.retain_hot = true;
      }
    }
  }
}

// Publish ONE tree's finished level into the tree. Per-tree rather than per-batch, which is what
// lets trees progress independently -- the only thing the old whole-batch version shared across
// trees was a linear cursor into node_ids, and that was an optimisation, not a dependency.
static void adjust_tree_after_lod(tree_registry_t &tree_cache, lod_tree_worker_data_t &adjust_data, int level)
{
  if (adjust_data.nodes[level].empty())
    return;
  tree_t *tree = tree_cache.get(adjust_data.tree_id);
  // A finalized tree's LOD was fully generated in (or before) its finalizing pass; writing LOD
  // data into it now means the watermark/finality logic is broken -- fail loudly (the upload
  // and eviction tiers treat finalized trees as immutable).
  assert(tree_cache.tree_state[adjust_data.tree_id.data] == uint8_t(tree_state_t::building) && "LOD write into finalized tree");
  // Writing LOD point counts / storage locations mutates the tree's serialized state, so mark it dirty.
  // Otherwise a tree that was already serialized-and-cleaned by an earlier (empty) LOD pass — which
  // happens with multi-file input, where LOD is first triggered on a partial done-morton watermark
  // before all files land — never gets re-serialized, and its LOD is silently dropped on disk.
  tree->is_dirty = true;
  int tree_index = 0;
  for (int node_index = 0; node_index < int(adjust_data.nodes[level].size()); node_index++)
  {
    auto current = adjust_data.nodes[level][node_index].id;
    auto &node_ids = tree->node_ids[level];

    while (tree_index < int(node_ids.size()) && node_ids[tree_index] < current)
      tree_index++;
    assert(node_ids[tree_index] == current);
    auto &done_node = adjust_data.nodes[level][node_index];
    auto &points_collection = tree->data[level][tree_index];
    points_collection.point_count = done_node.generated_point_count.data;
    points_collection.min = done_node.generated_min;
    points_collection.max = done_node.generated_max;
    assert(points_collection.data.size() == 1);
    points_collection.data[0].count.data = done_node.generated_point_count.data;
    points_collection.data[0].offset.data = 0;
    tree->storage_map.add_storage(done_node.storage_name, done_node.generated_attributes_id, std::move(done_node.generated_locations));
  }
}

// Move ONE tree to its next level, if the level it was working on has finished. Returns true if the
// tree just retired (no levels left), so the caller can decrement trees_active.
//
// Runs on the tree loop. tree_in_flight is written here, before any worker for the level is
// enqueued, and read on the storage loop by add_worker_done -- so it is stable for the whole time
// any worker could observe it.
static bool advance_tree(const std::vector<float> &random_offsets, tree_lod_generator_t &lod_generator, lod_worker_batch_t &batch, uint32_t tree_index, tree_registry_t &tree_cache,
                         storage_handler_t &cache_file, attributes_configs_t &attributes_configs, vio::thread_pool_t &pool)
{
  auto &tree = batch.worker_data[tree_index];
  int &level = batch.tree_level[tree_index];
  if (level < 0)
    return false; // already retired

  if (batch.tree_in_flight[tree_index] > 0)
  {
    if (batch.tree_completed[tree_index].load(std::memory_order_acquire) < batch.tree_in_flight[tree_index])
      return false; // this tree's level is still running; another tree may still make progress
    adjust_tree_after_lod(tree_cache, tree, level);
    batch.tree_workers[tree_index].clear();
    batch.tree_in_flight[tree_index] = 0;
    batch.tree_completed[tree_index].store(0, std::memory_order_relaxed);
  }

  size_t node_count = 0;
  while (node_count == 0 && level > 0)
  {
    level--;
    node_count = tree.nodes[level].size();
  }
  if (node_count == 0)
  {
    level = -1; // nothing left at any level
    return true;
  }

  batch.tree_in_flight[tree_index] = int(node_count);
  auto &workers = batch.tree_workers[tree_index];
  // Reserved, not grown: workers are enqueued as they are constructed and hold a reference to their
  // own element, so a reallocation mid-loop would dangle every worker already running.
  workers.reserve(node_count);
  for (auto &node : tree.nodes[level])
  {
    assert(!node.child_data.empty());
    get_storage_info(tree_cache, node);
    auto &lod_worker = workers.emplace_back(lod_generator, batch, tree_index, cache_file, attributes_configs, node, random_offsets);
    lod_worker.enqueue_lod(pool);
  }
  return false;
}

static void start_batch(lod_worker_batch_t &batch)
{
  const size_t trees = batch.worker_data.size();
  batch.tree_workers.resize(trees);
  batch.tree_level.assign(trees, 5);
  batch.tree_in_flight.assign(trees, 0);
  batch.tree_completed = std::make_unique<std::atomic_int[]>(trees);
  for (size_t i = 0; i < trees; i++)
    batch.tree_completed[i].store(0, std::memory_order_relaxed);
  batch.trees_active = int(trees);
  batch.started = true;
}

tree_lod_generator_t::tree_lod_generator_t(vio::event_loop_t &loop, vio::thread_pool_t &thread_pool, tree_registry_t &tree_cache, storage_handler_t &file_cache, attributes_configs_t &attributes_configs,
                                           perf_stats_t &perf_stats, vio::event_pipe_t<void> &lod_done)
  : _loop(loop)
  , _thread_pool(thread_pool)
  , _tree_cache(tree_cache)
  , _file_cache(file_cache)
  , _attributes_configs(attributes_configs)
  , _perf_stats(perf_stats)
  , _lod_done(lod_done)
  , _iterate_workers(_loop, vio::event_bind_t::bind(*this, &tree_lod_generator_t::iterate_workers))
{
  _random_offsets = make_lod_random_offsets();
}

void tree_lod_generator_t::generate_lods(tree_id_t &tree_id, const morton::morton192_t &max)
{
  std::vector<lod_tree_worker_data_t> to_lod;
  lod_node_worker_data_t fake_parent;
  fake_parent.node_min = _tree_cache.data[tree_id.data]->morton_min;
  tree_get_work_items(_tree_cache, _file_cache, tree_id, fake_parent, to_lod, max, _lod_complete_morton);
  _lod_complete_morton = max;
  if (!to_lod.empty())
  {
    std::sort(to_lod.begin(), to_lod.end(), [](const lod_tree_worker_data_t &a, const lod_tree_worker_data_t &b) { return a.magnitude < b.magnitude; });
    int current_level = to_lod.front().magnitude;
    int batch_start = 0;
    for (int i = 0; i < int(to_lod.size()); i++)
    {
      auto &current = to_lod[i];
      if (current.magnitude != current_level)
      {
        _lod_batches.emplace_back(new lod_worker_batch_t());
        auto &batch = *_lod_batches.back();
        batch.worker_data.insert(batch.worker_data.end(), std::make_move_iterator(to_lod.begin() + batch_start), std::make_move_iterator(to_lod.begin() + i));
        current_level = current.magnitude;
        batch_start = i;
      }
    }
    if (to_lod.begin() + batch_start != to_lod.end())
    {
      _lod_batches.emplace_back(new lod_worker_batch_t());
      auto &batch = *_lod_batches.back();
      batch.worker_data.insert(batch.worker_data.end(), std::make_move_iterator(to_lod.begin() + batch_start), std::make_move_iterator(to_lod.end()));
    }
  }
  _iterate_workers.post_event();
}

void tree_lod_generator_t::iterate_workers()
{
  // Drive the FRONT batch only. Batches are ordered by magnitude, and a tree's level-4 nodes sample
  // sub-trees, which have magnitude-1 and live in an earlier batch -- so batches must still run in
  // order. Within the batch every tree is independent, so sweep them all and advance whichever can.
  while (!_lod_batches.empty())
  {
    auto &batch = *_lod_batches.front();
    if (!batch.started)
      start_batch(batch);

    for (uint32_t tree_index = 0; tree_index < uint32_t(batch.worker_data.size()); tree_index++)
    {
      if (advance_tree(_random_offsets, *this, batch, tree_index, _tree_cache, _file_cache, _attributes_configs, _thread_pool))
        batch.trees_active--;
    }
    if (batch.trees_active > 0)
      break; // work is in flight; the completion that finishes a tree's level wakes us again
    _lod_batches.pop_front();
  }

  if (_lod_batches.empty())
  {
    auto lod_end = perf_stats_t::clock_t::now();
    auto lod_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(lod_end - _perf_stats.lod_start).count());
    _perf_stats.lod_generation_time_us.store(lod_us, std::memory_order_relaxed);
    if (g_lod_probe_on)
    {
      const auto workers = g_lod_probe.workers.load(std::memory_order_relaxed);
      const auto total = g_lod_probe.total_us.load(std::memory_order_relaxed);
      fmt::print(stderr, "\n[lod] pass {:.1f}s wall | cumulative: {} workers, {:.1f}s in-worker | attrib_map {:.1f}s  morton {:.1f}s  attributes {:.1f}s  write_post {:.1f}s\n",
                 double(lod_us) / 1e6, workers, double(total) / 1e6, double(g_lod_probe.attrib_map_us.load(std::memory_order_relaxed)) / 1e6,
                 double(g_lod_probe.morton_us.load(std::memory_order_relaxed)) / 1e6, double(g_lod_probe.attributes_us.load(std::memory_order_relaxed)) / 1e6,
                 double(g_lod_probe.write_post_us.load(std::memory_order_relaxed)) / 1e6);
    }
    _lod_done.post_event();
  }
}

} // namespace dew::converter
