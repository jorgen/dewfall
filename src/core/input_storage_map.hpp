/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2024  Jørgen Lind
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
#include <ankerl/unordered_dense.h>
#include <dataset_types.hpp>
#include <cstring>
#include <vector>

namespace dew::core
{

// Shared hasher for input_data_id_t keyed maps (storage maps, LOD child maps, registry chunk refs).
struct input_data_id_hash_t
{
  using is_avalanching = void;
  auto operator()(input_data_id_t id) const noexcept -> uint64_t
  {
    uint64_t data;
    static_assert(sizeof(data) == sizeof(id), "size mismatch");
    memcpy(&data, &id, sizeof(data));
    return ankerl::unordered_dense::detail::wyhash::hash(data);
  }
};

class input_storage_map_t
{
public:
  void add_storage(input_data_id_t id, attributes_id_t attributes_id, std::vector<storage_location_t> &&storage);
  // A unit GAINS an attribute: keep every blob it already has and append the new one, moving it to
  // `attributes_id` (which must be its old config with the attribute appended, so existing slot
  // indices still address the same blobs). Distinct from add_storage, which replaces the vector and
  // discards what was there. Does not change the reference count.
  void append_storage(input_data_id_t id, attributes_id_t attributes_id, std::vector<storage_location_t> &&extra);
  std::pair<attributes_id_t, std::vector<storage_location_t>> dereference(input_data_id_t id);
  // As dereference, but when the last reference drops the locations are recorded as discarded
  // (see take_discarded) instead of handed to the caller. For call sites that abandon the blobs.
  void dereference_discard(input_data_id_t id);
  std::pair<attributes_id_t, std::vector<storage_location_t>> info(input_data_id_t id) const;
  [[nodiscard]] bool contains(input_data_id_t id) const
  {
    return _map.contains(id);
  }
  [[nodiscard]] uint32_t ref_count(input_data_id_t id) const
  {
    auto it = _map.find(id);
    return it == _map.end() ? 0 : it->second.ref_count;
  }
  attributes_id_t attribute_id(input_data_id_t id) const;
  [[nodiscard]] storage_location_t location(input_data_id_t id, int attribute_index) const;
  void add_ref(input_data_id_t id);

  template <typename Fn>
  void for_each(Fn &&fn) const
  {
    for (auto &[id, value] : _map)
      fn(id, value.attributes_id, value.storage);
  }

  // Rewrite each entry's storage locations in place, leaving attributes_id and ref_count untouched. Used
  // by the copy tool to remap blob locations without disturbing the (serialized) reference counts.
  template <typename Fn>
  void remap_storage(Fn &&fn)
  {
    for (auto &[id, value] : _map)
      fn(value.storage);
  }

  // Blobs no longer referenced by this map: locations replaced by add_storage (LOD regeneration)
  // or dropped by dereference_discard. Drained by the tree handler into the next checkpoint's
  // freed list -- the backend returns the space to the allocator only after that checkpoint
  // commits, so the previous checkpoint's references stay readable until superseded atomically.
  std::vector<storage_location_t> take_discarded()
  {
    return std::move(_discarded);
  }
  void restore_discarded(std::vector<storage_location_t> &&discarded)
  {
    if (_discarded.empty())
      _discarded = std::move(discarded);
    else
      _discarded.insert(_discarded.end(), discarded.begin(), discarded.end());
  }

  uint32_t serialized_size() const;
  std::pair<bool, uint8_t *> serialize(uint8_t *buffer, const uint8_t *end) const;
  std::pair<bool, const uint8_t *> deserialize(const uint8_t *buffer, const uint8_t *end);

private:
  struct value_t
  {
    attributes_id_t attributes_id;
    std::vector<storage_location_t> storage;
    uint32_t ref_count;
  };
  ankerl::unordered_dense::map<input_data_id_t, value_t, input_data_id_hash_t> _map;
  std::vector<storage_location_t> _discarded;
};

class deref_on_destruct_t
{
public:
  explicit deref_on_destruct_t(input_storage_map_t &map)
    : _map(map)
  {
    _ids.reserve(16);
  }

  ~deref_on_destruct_t()
  {
    for (auto id : _ids)
    {
      _map.dereference_discard(id); // dropped blobs feed the next checkpoint's freed list
    }
  }

  void add(input_data_id_t id)
  {
    _ids.push_back(id);
  }

  deref_on_destruct_t(const deref_on_destruct_t &) = delete;
  deref_on_destruct_t &operator=(const deref_on_destruct_t &) = delete;
  deref_on_destruct_t(deref_on_destruct_t &&other) noexcept = delete;
  deref_on_destruct_t &operator=(deref_on_destruct_t &&other) noexcept = delete;

private:
  input_storage_map_t &_map;
  std::vector<input_data_id_t> _ids;
};

} // namespace dew::core

