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

#include "attribute_amend.hpp"

#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <mutex>

#include <fmt/format.h>

#include <format_util.hpp>

#include "storage_handler.hpp"
#include "tree_handler.hpp"

namespace dew::converter
{
using namespace dew::core;

namespace
{
// Units per write batch. Bounds the memory the amend holds at once: a batch's key blobs, its value
// buffers and their compressed forms all coexist. The whole dataset in one batch would be the whole
// new attribute in RAM, which for a 40M-point dataset and a u32 attribute is 160 MB before
// compression -- workable, but it scales with the dataset and this does not.
constexpr size_t k_batch_units = 256;

bool is_integer_type(dew_type_t type)
{
  switch (type)
  {
  case dew_type_u8:
  case dew_type_i8:
  case dew_type_u16:
  case dew_type_i16:
  case dew_type_u32:
  case dew_type_i32:
  case dew_type_u64:
  case dew_type_i64:
    return true;
  default:
    return false;
  }
}

// Widen one element of an integer key buffer to uint64. Signed types are sign-extended and then
// reinterpreted, so -1 and 0xFFFFFFFFFFFFFFFF collide -- consistent between the writer of the key
// and this reader, which is all the join needs.
uint64_t widen_key(const uint8_t *raw, dew_type_t type)
{
  switch (type)
  {
  case dew_type_u8:
    return *raw;
  case dew_type_i8:
  {
    int8_t v;
    memcpy(&v, raw, sizeof(v));
    return uint64_t(int64_t(v));
  }
  case dew_type_u16:
  {
    uint16_t v;
    memcpy(&v, raw, sizeof(v));
    return v;
  }
  case dew_type_i16:
  {
    int16_t v;
    memcpy(&v, raw, sizeof(v));
    return uint64_t(int64_t(v));
  }
  case dew_type_u32:
  {
    uint32_t v;
    memcpy(&v, raw, sizeof(v));
    return v;
  }
  case dew_type_i32:
  {
    int32_t v;
    memcpy(&v, raw, sizeof(v));
    return uint64_t(int64_t(v));
  }
  case dew_type_u64:
  case dew_type_i64:
  {
    uint64_t v;
    memcpy(&v, raw, sizeof(v));
    return v;
  }
  default:
    return 0;
  }
}

// The unit's current config with one attribute appended. Existing slots keep their indices, which is
// the whole contract of append_storage: readers resolve a name against the config and index the
// location vector with the result.
attributes_id_t config_with_attribute_appended(attributes_configs_t &configs, attributes_id_t base, const pending_attribute_t &attribute)
{
  dew_attributes_t copy;
  attributes_copy(configs.get(base), copy);
  copy.attribute_names.push_back(std::unique_ptr<char[]>(new char[attribute.name.size() + 1]));
  memcpy(copy.attribute_names.back().get(), attribute.name.data(), attribute.name.size());
  copy.attribute_names.back().get()[attribute.name.size()] = 0;
  copy.attributes.emplace_back(copy.attribute_names.back().get(), uint32_t(attribute.name.size()), attribute.type, attribute.components);
  return configs.get_attribute_config_index(std::move(copy));
}

} // namespace

void pending_attribute_t::put(uint64_t key, const uint8_t *value)
{
  auto it = index.find(key);
  if (it != index.end())
  {
    memcpy(values.data() + it->second, value, element_size);
    return;
  }
  const uint64_t offset = values.size();
  values.resize(values.size() + element_size);
  memcpy(values.data() + offset, value, element_size);
  index.emplace(key, offset);
}

bool attribute_amend_t::declare(const std::string &name, const std::string &key_attribute, dew_type_t type, dew_components_t components, std::string &error_message)
{
  if (name.empty() || key_attribute.empty())
  {
    error_message = "an attribute and the attribute it joins on both need names";
    return false;
  }
  for (auto &pending : _pending)
  {
    if (pending.name == name)
    {
      error_message = fmt::format("attribute '{}' is already declared for this amend", name);
      return false;
    }
  }
  const int element_size = size_for_format(type, components);
  if (element_size <= 0)
  {
    error_message = fmt::format("attribute '{}' has no representable element size", name);
    return false;
  }
  pending_attribute_t pending;
  pending.name = name;
  pending.key_attribute = key_attribute;
  pending.type = type;
  pending.components = components;
  pending.element_size = uint32_t(element_size);
  _pending.push_back(std::move(pending));
  return true;
}

bool attribute_amend_t::add_data(const std::string &name, const uint64_t *keys, const void *values, uint64_t count)
{
  for (auto &pending : _pending)
  {
    if (pending.name != name)
      continue;
    if (!count)
      return true;
    if (!keys || !values)
      return false;
    pending.index.reserve(pending.index.size() + size_t(count));
    pending.values.reserve(pending.values.size() + size_t(count) * pending.element_size);
    const auto *raw = static_cast<const uint8_t *>(values);
    for (uint64_t i = 0; i < count; i++)
      pending.put(keys[i], raw + i * pending.element_size);
    return true;
  }
  return false;
}

dew_error_t attribute_amend_t::commit(tree_handler_t &tree_handler, storage_handler_t &storage_handler, attributes_configs_t &attributes_configs)
{
  dew_error_t error;
  if (_pending.empty())
    return error;

  for (auto &attribute : _pending)
  {
    // Each attribute is a separate pass over the dataset. Doing all of them per unit would halve the
    // reads of the key blob, but only when they share a key -- and it would make a failure partway
    // through leave units carrying some of the new attributes and not others.
    //
    // Re-snapshotted per attribute, not hoisted: landing an attribute moves every amended unit to a
    // new config, and building the second attribute's config on top of a stale one would drop the
    // first attribute's slot from the mapping while its blob stayed in the location vector -- every
    // attribute of that unit mis-resolved from then on.
    auto units = tree_handler.snapshot_storage_units();
    if (units.empty())
    {
      error.code = -1;
      error.msg = "attribute amend found no units to amend (is every tree loaded?)";
      return error;
    }
    size_t amended_units = 0;
    size_t skipped_no_key = 0;
    size_t skipped_have_it = 0;
    uint64_t matched_points = 0;
    uint64_t total_points = 0;

    for (size_t batch_start = 0; batch_start < units.size(); batch_start += k_batch_units)
    {
      const size_t batch_end = std::min(batch_start + k_batch_units, units.size());
      std::vector<attribute_write_t> writes;
      std::vector<storage_unit_append_t> appends;
      writes.reserve(batch_end - batch_start);
      appends.reserve(batch_end - batch_start);

      for (size_t i = batch_start; i < batch_end; i++)
      {
        auto &unit = units[i];

        // Already has it: an interrupted amend re-run, or a name the dataset genuinely carries.
        // Either way appending a second slot of the same name would leave the config ambiguous.
        if (attributes_configs.get_attribute_index(unit.attributes_id, attribute.name).index >= 0)
        {
          skipped_have_it++;
          continue;
        }
        const auto key_index = attributes_configs.get_attribute_index(unit.attributes_id, attribute.key_attribute);
        // A unit without the key cannot be joined. Legal and expected: LOD levels drop most
        // attributes (tree_lod_generator's keep-list), so a key that is not on that list simply
        // stops the amend at the boundary where provenance stops.
        if (key_index.index < 0 || key_index.index >= int(unit.storage.size()))
        {
          skipped_no_key++;
          continue;
        }
        if (key_index.format.components != dew_components_1 || !is_integer_type(key_index.format.type))
        {
          skipped_no_key++;
          continue;
        }
        const auto key_location = unit.storage[size_t(key_index.index)];
        if (key_location.size == 0)
        {
          skipped_no_key++;
          continue;
        }

        auto request = storage_handler.read(key_location);
        request->wait_for_read();
        if (request->error.code != 0)
        {
          error = request->error;
          return error;
        }
        const auto key_element_size = uint32_t(size_for_format(key_index.format.type, key_index.format.components));
        const uint32_t point_count = request->buffer_info.size / key_element_size;
        if (!point_count)
        {
          skipped_no_key++;
          continue;
        }

        // Zero-initialized, deliberately: a point with no entry in the table reads back as zero,
        // which is exactly what a node that lacks the attribute entirely reads back as. The two
        // states are indistinguishable to a reader, which is what makes a partial amend legal.
        const uint32_t buffer_size = point_count * attribute.element_size;
        std::shared_ptr<uint8_t[]> buffer(new uint8_t[buffer_size]());
        const auto *keys = static_cast<const uint8_t *>(request->buffer_info.data);
        for (uint32_t p = 0; p < point_count; p++)
        {
          const uint64_t key = widen_key(keys + size_t(p) * key_element_size, key_index.format.type);
          auto it = attribute.index.find(key);
          if (it == attribute.index.end())
            continue;
          memcpy(buffer.get() + size_t(p) * attribute.element_size, attribute.values.data() + it->second, attribute.element_size);
          matched_points++;
        }
        total_points += point_count;

        attribute_write_t write;
        write.data = std::move(buffer);
        write.size = buffer_size;
        write.point_count = point_count;
        write.format = {attribute.type, attribute.components};
        write.attribute_name = attribute.name;
        write.is_lod = !input_data_id_is_leaf(unit.id);
        writes.push_back(std::move(write));

        storage_unit_append_t append;
        append.tree = unit.tree;
        append.id = unit.id;
        append.attributes_id = config_with_attribute_appended(attributes_configs, unit.attributes_id, attribute);
        appends.push_back(std::move(append));
      }

      if (writes.empty())
        continue;

      std::mutex mutex;
      std::condition_variable cv;
      bool done = false;
      std::vector<storage_location_t> locations;
      dew_error_t write_error;
      storage_handler.write_attributes(std::move(writes), [&](std::vector<storage_location_t> &&result, const dew_error_t &write_result_error) {
        std::unique_lock<std::mutex> lock(mutex);
        locations = std::move(result);
        write_error = write_result_error;
        done = true;
        cv.notify_all();
      });
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return done; });
      }
      if (write_error.code != 0)
        return write_error;
      if (locations.size() != appends.size())
      {
        error.code = -1;
        error.msg = "attribute amend wrote a different number of blobs than it asked for";
        return error;
      }

      for (size_t i = 0; i < appends.size(); i++)
        appends[i].locations.push_back(locations[i]);
      amended_units += appends.size();
      if (!tree_handler.append_storage_units(std::move(appends)))
      {
        error.code = -1;
        error.msg = "attribute amend could not reach the tree loop to record the blobs it wrote";
        return error;
      }
    }

    if (std::getenv("DEW_DEBUG_AMEND"))
      fmt::print(stderr, "amend '{}': {} units amended, {} without the key '{}', {} already had it; {}/{} points matched\n", attribute.name, amended_units, skipped_no_key, attribute.key_attribute,
                 skipped_have_it, matched_points, total_points);
  }

  clear();
  return error;
}

} // namespace dew::converter
