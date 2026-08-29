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

#ifndef DEW_ATTRIBUTE_AMEND_HPP
#define DEW_ATTRIBUTE_AMEND_HPP

#include <string>
#include <vector>

#include <ankerl/unordered_dense.h>

#include <attributes_configs.hpp>
#include <dataset_types.hpp>
#include <dew/core/error.h>

namespace dew::converter
{
class tree_handler_t;
class storage_handler_t;

// Buffered attribute values waiting to be joined onto an existing dataset.
//
// Deliberately keyed rather than ordered. The converter reorders every point by morton code, splits
// it across nodes, collapses chunks into per-node units and samples it into LOD levels -- so a
// caller holding "the values for my Nth point" has no way to say where that point ended up. It
// names an attribute the dataset already carries instead, and supplies (key, value) pairs.
//
// The key attribute must be a single-component integer; values are widened to uint64 so one code
// path serves a u32 point index and a packed (scan, index) u64 alike.
struct pending_attribute_t
{
  std::string name;
  std::string key_attribute;
  dew_type_t type = dew_type_u8;
  dew_components_t components = dew_components_1;
  uint32_t element_size = 0;

  // key -> byte offset into `values`. A repeated key overwrites, so redelivering a range is safe.
  ankerl::unordered_dense::map<uint64_t, uint64_t> index;
  std::vector<uint8_t> values;

  void put(uint64_t key, const uint8_t *value);
};

class attribute_amend_t
{
public:
  // Declare a new attribute and the existing one to join it on. False if the name is taken by
  // another pending declaration, or the type is not a real point type.
  bool declare(const std::string &name, const std::string &key_attribute, dew_type_t type, dew_components_t components, std::string &error_message);
  // Buffer `count` values against `count` keys. False if `name` was never declared.
  bool add_data(const std::string &name, const uint64_t *keys, const void *values, uint64_t count);

  [[nodiscard]] bool empty() const
  {
    return _pending.empty();
  }
  [[nodiscard]] const std::vector<pending_attribute_t> &pending() const
  {
    return _pending;
  }
  void clear()
  {
    _pending.clear();
  }

  // Apply every declaration to every unit of the dataset. Blocks. Requires every tree resident
  // (mutable mode loads them all at open) and must NOT run on the tree loop or the storage loop --
  // it blocks on reads and writes serviced by both.
  //
  // Per unit: read its key blob, build one new buffer, write it, append it to the unit's storage.
  // Nothing the unit already holds is read, recompressed or moved.
  [[nodiscard]] dew_error_t commit(tree_handler_t &tree_handler, storage_handler_t &storage_handler, core::attributes_configs_t &attributes_configs);

private:
  std::vector<pending_attribute_t> _pending;
};

} // namespace dew::converter

#endif // DEW_ATTRIBUTE_AMEND_HPP
