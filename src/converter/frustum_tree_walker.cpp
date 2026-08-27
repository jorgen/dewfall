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
#include "frustum_tree_walker.hpp"

#include "attributes_configs.hpp"
#include "frustum.hpp"
#include "morton_tree_coordinate_transform.hpp"
#include <atomic>
#include <fmt/printf.h>

namespace dew::converter
{
using namespace dew::core;
frustum_tree_walker_t::frustum_tree_walker_t(const glm::dmat4 view_perspective, lod_params_t lod_params, std::vector<std::string> attribute_names)
  : m_view_perspective(view_perspective)
  , m_lod_params(lod_params)
  , m_attribute_names(std::move(attribute_names))
{
}

bool should_subdivide(const lod_params_t &params, const node_aabb_t &aabb, bool was_subdivided)
{
  glm::dvec3 center = (aabb.min + aabb.max) * 0.5;
  glm::dvec3 extent = aabb.max - aabb.min;
  double node_size = glm::length(extent) * 0.5;

  double distance = glm::length(center - params.camera_position);
  if (distance < node_size)
    return true;

  // projected_fraction: fraction of screen height covered by this node (0..1+)
  double projected_fraction = params.projection[1][1] * 0.5 * node_size / distance;

  // Hysteresis: if the node was subdivided last frame, use a lower threshold
  // to prevent flickering when the camera is near the LOD boundary.
  double threshold = was_subdivided
    ? params.screen_fraction_threshold * (1.0 - params.hysteresis)
    : params.screen_fraction_threshold;
  return projected_fraction > threshold;
}

// static node_id_t create_node_id(tree_id_t tree_id, int level, int index)
//{
//   return {tree_id, uint16_t(level), uint16_t(index)};
// }

struct tree_walker_possible_nodes_t
{
  tree_walker_possible_nodes_t() = default;
  tree_walker_possible_nodes_t(const tree_t *a_tree, node_id_t a_parent, uint16_t a_skip, node_aabb_t a_aabbs, bool a_is_completely_inside_frustum)
    : tree(a_tree)
    , parent(a_parent)
    , skip(a_skip)
    , aabbs(a_aabbs)
    , is_completely_inside_frustum(a_is_completely_inside_frustum)
  {
  }
  const tree_t *tree;
  node_id_t parent;
  uint16_t skip;
  node_aabb_t aabbs;
  bool is_completely_inside_frustum;
};

node_aabb_t make_aabb_from_child_index(const node_aabb_t &parent, int child_index)
{
  node_aabb_t aabb;
  aabb.min = parent.min;
  aabb.max = parent.max;
  for (int i = 0; i < 3; i++)
  {
    if (child_index & (1 << i))
      aabb.min[i] = (aabb.min[i] + aabb.max[i]) / 2;
    else
      aabb.max[i] = (aabb.min[i] + aabb.max[i]) / 2;
  }
  return aabb;
}

static void walk_tree(const tree_registry_t &tree_registry, attribute_index_map_t &attribute_index_map, tree_id_t tree_id, frustum_tree_walker_t &walker)
{
  (void)attribute_index_map;
  (void)walker;
  // const_cast for a load-only atomic view: libc++ (emscripten) rejects atomic_ref over a const type.
  if (!std::atomic_ref<uint8_t>(const_cast<uint8_t &>(tree_registry.tree_id_initialized[tree_id.data])).load(std::memory_order_acquire))
  {
    walker.m_trees_to_load.push_back(tree_id);
    return;
  }
  auto tree = tree_registry.get(tree_id);
  if (tree->data[0].empty() || tree->data[0][0].data.empty())
    return;

  bool current_buffer_index = false;
  std::vector<tree_walker_possible_nodes_t> alternating_possible_nodes[2];

  double min[3];
  double max[3];
  convert_morton_to_pos(tree_registry.tree_config.scale, tree_registry.tree_config.offset, tree->morton_min, min);
  convert_morton_to_pos(tree_registry.tree_config.scale, tree_registry.tree_config.offset, tree->morton_max, max);

  node_aabb_t aabb = {glm::dvec3(min[0], min[1], min[2]), glm::dvec3(max[0], max[1], max[2])};

  if (walker.m_debug)
  {
    glm::dvec3 center = (aabb.min + aabb.max) * 0.5;
    glm::dvec3 extent = aabb.max - aabb.min;
    double magnitude = glm::length(extent);
    double distance = glm::length(center - walker.m_lod_params.camera_position);
    double node_size = magnitude * 0.5;
    double projected_fraction = (distance > node_size) ? walker.m_lod_params.projection[1][1] * 0.5 * node_size / distance : 999.0;
    bool subdivide = should_subdivide(walker.m_lod_params, aabb);
    size_t root_subset_count = (!tree->data[0].empty() && !tree->data[0][0].data.empty()) ? tree->data[0][0].data.size() : 0;
    fmt::print(stderr, "[walker-debug] root tree {}: AABB=[{:.1f},{:.1f},{:.1f}]-[{:.1f},{:.1f},{:.1f}] mag={:.1f}\n",
               tree_id.data, aabb.min.x, aabb.min.y, aabb.min.z, aabb.max.x, aabb.max.y, aabb.max.z, magnitude);
    fmt::print(stderr, "[walker-debug]   camera=[{:.1f},{:.1f},{:.1f}] dist={:.1f} projected_frac={:.4f} threshold={:.4f} subdivide={}\n",
               walker.m_lod_params.camera_position.x, walker.m_lod_params.camera_position.y, walker.m_lod_params.camera_position.z,
               distance, projected_fraction, walker.m_lod_params.screen_fraction_threshold, subdivide);
    fmt::print(stderr, "[walker-debug]   root subsets={}\n", root_subset_count);
  }

  node_id_t empty_node_id = {tree_id_t(UINT32_MAX), UINT16_MAX, UINT16_MAX};
  alternating_possible_nodes[current_buffer_index].emplace_back(tree, empty_node_id, 0, aabb, false);
  render::frustum_t frustum;
  frustum.update(walker.m_view_perspective);
  int lod = morton::morton_magnitude_to_lod(tree->magnitude);
  constexpr int max_depth = 40;
  for (int depth = 0; depth < max_depth; depth++, current_buffer_index = !current_buffer_index, lod--)
  {
    if (alternating_possible_nodes[current_buffer_index].empty())
      break;
    int current_depth_in_tree = depth % 5;
    alternating_possible_nodes[!current_buffer_index].clear();
    for (auto &possible_nodes : alternating_possible_nodes[current_buffer_index])
    {
      auto hit_test = possible_nodes.is_completely_inside_frustum ? render::frustum_intersection_t::inside : frustum.test_aabb(possible_nodes.aabbs.min, possible_nodes.aabbs.max);
      bool visible = (hit_test != render::frustum_intersection_t::outside);
      auto current_tree = possible_nodes.tree;
      auto node_name = current_tree->node_ids[current_depth_in_tree][possible_nodes.skip];
      node_id_t node_id = {current_tree->id, uint16_t(current_depth_in_tree), node_name};
      auto &points_collection = current_tree->data[current_depth_in_tree][possible_nodes.skip];
      double tight_min[3], tight_max[3];
      convert_morton_to_pos(tree_registry.tree_config.scale, tree_registry.tree_config.offset, points_collection.min, tight_min);
      convert_morton_to_pos(tree_registry.tree_config.scale, tree_registry.tree_config.offset, points_collection.max, tight_max);
      node_aabb_t tight_aabb = {glm::dvec3(tight_min[0], tight_min[1], tight_min[2]), glm::dvec3(tight_max[0], tight_max[1], tight_max[2])};
      for (auto &points : points_collection.data)
      {
        auto attr_id = current_tree->storage_map.attribute_id(points.input_id);
        attribute_index_t attrib_indexes[] = {{-2, {}}, {-2, {}}, {-2, {}}, {-2, {}}};
        bool valid = true;
        for (int i = 0; i < 4 && i < attribute_index_map.get_attribute_count(); i++)
        {
          auto index = attribute_index_map.get_index(attr_id, i);
          attrib_indexes[i] = index;
          if (index.index == -1 && i == 0)
          {
            // Only the position attribute is mandatory. Slimmed LOD nodes lack non-visual
            // attributes; when the selected color is absent the node still renders (the decoder
            // substitutes a constant) instead of vanishing from the frame.
            valid = false;
            break;
          }
        }
        if (!valid)
        {
          if (walker.m_debug)
          {
            fmt::print(stderr, "[walker-debug] attribute lookup FAILED: tree={} level={} node={} input_id={}.{}\n",
                       current_tree->id.data, current_depth_in_tree, node_name, points.input_id.data, points.input_id.sub);
            for (int ai = 0; ai < 4 && ai < attribute_index_map.get_attribute_count(); ai++)
            {
              auto idx = attribute_index_map.get_index(attr_id, ai);
              if (idx.index == -1)
                fmt::print(stderr, "[walker-debug]   attrib[{}] MISSING (index=-1)\n", ai);
            }
          }
          continue;
        }
        auto &to_add = walker.m_new_nodes.point_subsets.emplace_back();
        for (auto &loc : to_add.locations)
          loc = storage_location_t();
        to_add.parent = possible_nodes.parent;
        to_add.lod = lod;
        to_add.node = node_id;
        to_add.aabb = possible_nodes.aabbs;
        to_add.tight_aabb = tight_aabb;
        to_add.frustum_visible = visible;
        to_add.is_leaf = (current_tree->nodes[current_depth_in_tree][possible_nodes.skip] == 0);
        to_add.offset_in_subset = points.offset;
        to_add.point_count = points.count;
        to_add.input_id = points.input_id;
        for (int i = 0; i < 4 && i < attribute_index_map.get_attribute_count(); i++)
        {
          auto attrib_index = attrib_indexes[i];
          if (attrib_index.index == -1)
          {
            to_add.locations[i] = storage_location_t(); // absent: size 0, decoder substitutes
            to_add.format[i] = point_format_t(dew_type_u8, dew_components_1);
            continue;
          }
          auto location = current_tree->storage_map.location(points.input_id, attrib_index.index);
          to_add.locations[i] = location;
          to_add.format[i] = attrib_index.format;
        }
      }

      bool was_subdivided = walker.m_previously_subdivided.count(node_id) > 0;
      if (!visible || !should_subdivide(walker.m_lod_params, possible_nodes.aabbs, was_subdivided))
        continue;

      auto children = current_tree->nodes[current_depth_in_tree][possible_nodes.skip];
      int child_count = 0;
      for (int i = 0; i < 8 && children; i++, children >>= 1)
      {
        if (children & 1)
        {
          auto child_aabb = make_aabb_from_child_index(possible_nodes.aabbs, i);
          bool is_completely_inside_frustum = hit_test == render::frustum_intersection_t::inside;
          auto next_tree = current_tree;
          auto next_skip = current_tree->skips[current_depth_in_tree][possible_nodes.skip] + child_count;
          if (current_depth_in_tree == 4)
          {
            auto next_sub_tree_skip = current_tree->skips[4][possible_nodes.skip] + child_count;
            auto next_tree_id = current_tree->sub_trees[next_sub_tree_skip];

            if (!std::atomic_ref<uint8_t>(const_cast<uint8_t &>(tree_registry.tree_id_initialized[next_tree_id.data])).load(std::memory_order_acquire))
            {
              walker.m_trees_to_load.push_back(next_tree_id);
              // child_count indexes into the contiguous sub_trees[] array by set-bit ordinal
              // (one entry per set child bit). It must advance for EVERY set bit, including
              // unresident ones, or later resident siblings resolve to the wrong sub_tree entry
              // and are silently dropped from traversal/rendering.
              child_count++;
              continue;
            }
            next_tree = tree_registry.get(next_tree_id);
            next_skip = 0;
          }
          alternating_possible_nodes[!current_buffer_index].emplace_back(next_tree, node_id, next_skip, child_aabb, is_completely_inside_frustum);
          int child_depth = (current_depth_in_tree == 4) ? 0 : current_depth_in_tree + 1;
          node_id_t child_node_id = {next_tree->id, uint16_t(child_depth), next_tree->node_ids[child_depth][next_skip]};
          walker.m_new_nodes.parent_child_edges.emplace_back(node_id, child_node_id);
          child_count++;
        }
      }
    }
  }
}

void walk_tree_direct(const tree_registry_t &tree_registry, attribute_index_map_t &attribute_index_map, frustum_tree_walker_t &walker)
{
  auto root = tree_registry.root;
  walk_tree(tree_registry, attribute_index_map, root, walker);
  // Capture the count here, while the vector is still ours: the caller moves it into
  // request_trees_async. An unresident tree is pushed again on every frame it is missing, so this is a
  // live "the walk is waiting on N sub-trees" signal, not a one-shot request count.
  walker.m_trees_requested = walker.m_trees_to_load.size();
}

} // namespace dew::converter
