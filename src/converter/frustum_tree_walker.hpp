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

#include <attributes_configs.hpp>
#include <glm_include.hpp>
#include "tree.hpp"

#include <cstring>
#include <unordered_set>


namespace dew::converter
{
using namespace dew::core;

struct node_id_t
{
  tree_id_t tree_id;
  uint16_t level;
  uint16_t index;
};

inline std::strong_ordering operator<=>(const node_id_t &a, const node_id_t &b)
{
  if (a.tree_id.data != b.tree_id.data)
  {
    return a.tree_id.data < b.tree_id.data ? std::strong_ordering::less : std::strong_ordering::greater;
  }
  if (a.level != b.level)
  {
    return a.level < b.level ? std::strong_ordering::less : std::strong_ordering::greater;
  }
  return a.index < b.index ? std::strong_ordering::less : (a.index == b.index ? std::strong_ordering::equal : std::strong_ordering::greater);
}

struct node_id_hash
{
  size_t operator()(const node_id_t &id) const
  {
    uint64_t v;
    memcpy(&v, &id, sizeof(v));
    return std::hash<uint64_t>()(v);
  }
};

struct node_id_equal
{
  bool operator()(const node_id_t &a, const node_id_t &b) const
  {
    return (a <=> b) == std::strong_ordering::equal;
  }
};

struct node_aabb_t
{
  glm::dvec3 min;
  glm::dvec3 max;
};

static_assert(sizeof(node_id_t) == sizeof(uint64_t), "size mismatch");

struct tree_walker_data_t
{
  node_id_t parent;
  int lod;
  node_id_t node;
  node_aabb_t aabb;
  node_aabb_t tight_aabb;
  offset_in_subset_t offset_in_subset;
  point_count_t point_count;
  input_data_id_t input_id;
  point_format_t format[4];
  storage_location_t locations[4];
  bool frustum_visible = true;
  bool is_leaf = false; // the tree stops here (no child nodes) -> a candidate for virtual subdivision
};

struct tree_walker_nodes_t
{
  std::vector<tree_walker_data_t> point_subsets;
  std::vector<std::pair<node_id_t, node_id_t>> parent_child_edges;
};

class attribute_index_map_t
{
public:
  attribute_index_map_t(const attributes_configs_t &a_attributes_configs, std::vector<std::string> attribute_names)
    : m_attributes_configs(a_attributes_configs)
    , m_attribute_names(std::move(attribute_names))
  {
  }

  [[nodiscard]] int get_attribute_count() const
  {
    return int(m_attribute_names.size());
  }

  attribute_index_t get_index(attributes_id_t id, int attribute_name_index)
  {
    assert(attribute_name_index < int(m_attribute_names.size()));
    auto it = m_map.find(id);
    if (it != m_map.end())
    {
      auto ret = it->second[attribute_name_index];
      if (ret.index >= -1)
      {
        return ret;
      }
    }
    auto &value = it != m_map.end() ? it->second : m_map[id];
    value.resize(m_attribute_names.size(), {-2, {}});
    auto index = m_attributes_configs.get_attribute_index(id, m_attribute_names[attribute_name_index]);
    value[attribute_name_index] = index;
    return index;
  }

private:
  struct index_pair_t
  {
    int attribute_name_index;
    int index;
  };
  struct hash
  {
    using is_avalanching = void;
    auto operator()(attributes_id_t id) const noexcept -> uint64_t
    {
      uint32_t data;
      static_assert(sizeof(data) == sizeof(id), "size mismatch");
      memcpy(&data, &id, sizeof(data));
      return ankerl::unordered_dense::detail::wyhash::hash(data);
    }
  };
  const attributes_configs_t &m_attributes_configs;
  std::vector<std::string> m_attribute_names;
  ankerl::unordered_dense::map<attributes_id_t, std::vector<attribute_index_t>, hash> m_map;
};

struct lod_params_t
{
  glm::dvec3 camera_position;
  glm::dmat4 projection;
  double screen_fraction_threshold;
  double hysteresis = 0.15;
};

using node_set_t = std::unordered_set<node_id_t, node_id_hash, node_id_equal>;

class frustum_tree_walker_t
{
public:
  frustum_tree_walker_t(glm::dmat4 view_perspective, lod_params_t lod_params, std::vector<std::string> attribute_names);

  glm::dmat4 m_view_perspective;
  lod_params_t m_lod_params;
  std::vector<std::string> m_attribute_names;
  tree_walker_nodes_t m_new_nodes;
  double m_tree_offset[3];
  std::vector<tree_id_t> m_trees_to_load;
  // How many entries m_trees_to_load ended the walk with. Kept separately because the caller MOVES the
  // vector out (processor_t::walk_tree hands it to request_trees_async, which takes it by value), so
  // reading .size() after the walk reports 0 no matter how many trees the walk was waiting for -- which
  // is what the walker_trees_to_load frame timing, and the public getter over it, used to report.
  size_t m_trees_requested = 0;
  node_set_t m_previously_subdivided;
  bool m_debug = false;
};

bool should_subdivide(const lod_params_t &params, const node_aabb_t &aabb, bool was_subdivided = false);

// Halve `parent` into octant `child_index`'s sub-cube (bit i of child_index selects the high half on axis i).
// Shared with the virtual subdivision so a virtual node's loose cube matches the real walker's.
node_aabb_t make_aabb_from_child_index(const node_aabb_t &parent, int child_index);

void walk_tree_direct(const tree_registry_t &tree_registry, attribute_index_map_t &attribute_index_map, frustum_tree_walker_t &walker);

} // namespace dew::converter

