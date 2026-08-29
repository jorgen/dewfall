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

#include <algorithm>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <queue>
#include <utility>

#include <fmt/format.h>

#include <format_util.hpp>

#include "storage_handler.hpp"
#include "tree_handler.hpp"

namespace dew::converter
{
using namespace dew::core;

namespace
{
constexpr size_t k_spill_buffer_bytes = 4u << 20;
constexpr uint64_t k_default_memory_budget = 512ull << 20;

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

uint64_t record_key(const uint8_t *record)
{
  uint64_t key;
  memcpy(&key, record, sizeof(key));
  return key;
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

// One (key, unit, slot) address. The A side of the join: where in the dataset a key's value has to
// land. `unit` indexes the current slice, not the dataset.
struct address_t
{
  uint64_t key;
  uint32_t unit;
  uint32_t slot;
};

// Sort spilled records by key, in bounded memory.
//
// Run formation then k-way merge, the ordinary external sort -- with one property the merge relies
// on. Duplicate keys must resolve LAST-WINS (the documented contract of add_data), and no sequence
// number is stored to make that possible. It works out anyway: a run holds a contiguous stretch of
// the input, std::stable_sort preserves input order inside a run, and the merge below breaks ties by
// run index. So equal keys come out in the order they were added, and the consumer takes the last of
// each run. Storing an explicit ordinal would have cost 4 bytes on every record.
bool sort_spill_by_key(const std::string &source_path, uint32_t record_size, uint64_t record_count, const std::string &sorted_path, std::string &error)
{
  const uint64_t budget = amend_memory_budget();
  const uint64_t per_run = std::max<uint64_t>(1, budget / record_size);

  std::vector<std::string> run_paths;
  {
    spill_file_t in;
    if (!in.open_read(source_path, record_size, error))
      return false;
    std::vector<uint8_t> block;
    std::vector<uint32_t> order;
    const uint8_t *record = nullptr;
    bool more = true;
    while (more)
    {
      block.clear();
      order.clear();
      uint64_t staged = 0;
      while (staged < per_run)
      {
        if (!in.next(record, error))
        {
          if (!error.empty())
            return false;
          more = false;
          break;
        }
        block.insert(block.end(), record, record + record_size);
        order.push_back(uint32_t(staged));
        staged++;
      }
      if (!staged)
        break;
      const uint8_t *base = block.data();
      std::stable_sort(order.begin(), order.end(), [base, record_size](uint32_t a, uint32_t b) { return record_key(base + size_t(a) * record_size) < record_key(base + size_t(b) * record_size); });

      spill_file_t run;
      const std::string run_path = fmt::format("{}.run{}", sorted_path, run_paths.size());
      if (!run.open_write(run_path, record_size, error))
        return false;
      for (auto index : order)
      {
        if (!run.append(base + size_t(index) * record_size, error))
          return false;
      }
      if (!run.flush(error))
        return false;
      run.close();
      run.release();
      run_paths.push_back(run_path);
    }
  }
  (void)record_count;

  // One run is already the answer.
  if (run_paths.size() == 1)
  {
    std::error_code ec;
    std::filesystem::rename(run_paths[0], sorted_path, ec);
    if (ec)
    {
      error = fmt::format("could not rename {} to {}: {}", run_paths[0], sorted_path, ec.message());
      return false;
    }
    return true;
  }

  std::vector<std::unique_ptr<spill_file_t>> runs;
  runs.reserve(run_paths.size());
  for (const auto &path : run_paths)
  {
    auto run = std::make_unique<spill_file_t>();
    if (!run->open_read(path, record_size, error))
      return false;
    runs.push_back(std::move(run));
  }

  struct heap_entry_t
  {
    uint64_t key;
    uint32_t run;
    std::vector<uint8_t> record;
  };
  // Ties broken by run index so earlier-added records come out first -- see the note above.
  auto worse = [](const heap_entry_t &a, const heap_entry_t &b) { return a.key != b.key ? a.key > b.key : a.run > b.run; };
  std::priority_queue<heap_entry_t, std::vector<heap_entry_t>, decltype(worse)> heap(worse);

  const uint8_t *record = nullptr;
  for (uint32_t r = 0; r < uint32_t(runs.size()); r++)
  {
    if (!runs[r]->next(record, error))
    {
      if (!error.empty())
        return false;
      continue;
    }
    heap.push({record_key(record), r, std::vector<uint8_t>(record, record + record_size)});
  }

  spill_file_t out;
  if (!out.open_write(sorted_path, record_size, error))
    return false;
  while (!heap.empty())
  {
    auto top = heap.top();
    heap.pop();
    if (!out.append(top.record.data(), error))
      return false;
    if (!runs[top.run]->next(record, error))
    {
      if (!error.empty())
        return false;
      continue;
    }
    heap.push({record_key(record), top.run, std::vector<uint8_t>(record, record + record_size)});
  }
  if (!out.flush(error))
    return false;
  out.close();
  out.release();
  for (auto &run : runs)
    run->remove_file();
  return true;
}

} // namespace

uint64_t amend_memory_budget()
{
  if (const char *override_value = std::getenv("DEW_AMEND_BUDGET_BYTES"))
  {
    const uint64_t parsed = strtoull(override_value, nullptr, 10);
    if (parsed >= 4096)
      return parsed;
  }
  return k_default_memory_budget;
}

spill_file_t::~spill_file_t()
{
  close();
  if (_owns)
    remove_file();
}

void spill_file_t::release()
{
  _owns = false;
}

spill_file_t::spill_file_t(spill_file_t &&other) noexcept
{
  *this = std::move(other);
}

spill_file_t &spill_file_t::operator=(spill_file_t &&other) noexcept
{
  if (this == &other)
    return *this;
  close();
  if (_owns)
    remove_file();
  _path = std::move(other._path);
  _file = other._file;
  _record_size = other._record_size;
  _records = other._records;
  _buffer = std::move(other._buffer);
  _buffer_used = other._buffer_used;
  _read_cursor = other._read_cursor;
  _writing = other._writing;
  _owns = other._owns;
  other._owns = false;
  other._file = nullptr;
  other._path.clear();
  other._buffer_used = 0;
  other._read_cursor = 0;
  other._records = 0;
  return *this;
}

bool spill_file_t::open_write(const std::string &path, uint32_t record_size, std::string &error)
{
  close();
  _path = path;
  _record_size = record_size;
  _records = 0;
  _writing = true;
  _owns = true;
  _buffer.resize(std::max<size_t>(k_spill_buffer_bytes, record_size));
  _buffer_used = 0;
  _file = fopen(path.c_str(), "wb");
  if (!_file)
  {
    error = fmt::format("cannot create spill file {}", path);
    return false;
  }
  return true;
}

bool spill_file_t::append(const uint8_t *record, std::string &error)
{
  if (_buffer_used + _record_size > _buffer.size())
  {
    if (!flush(error))
      return false;
  }
  memcpy(_buffer.data() + _buffer_used, record, _record_size);
  _buffer_used += _record_size;
  _records++;
  return true;
}

bool spill_file_t::flush(std::string &error)
{
  if (!_writing || !_buffer_used)
    return true;
  if (!_file)
  {
    error = "spill file is not open for writing";
    return false;
  }
  if (fwrite(_buffer.data(), 1, _buffer_used, _file) != _buffer_used)
  {
    error = fmt::format("short write to spill file {}", _path);
    return false;
  }
  _buffer_used = 0;
  return true;
}

void spill_file_t::close()
{
  if (_file)
  {
    if (_writing)
    {
      std::string ignored;
      (void)flush(ignored);
    }
    fclose(_file);
    _file = nullptr;
  }
  _buffer_used = 0;
  _read_cursor = 0;
}

void spill_file_t::remove_file()
{
  if (_path.empty())
    return;
  close();
  std::error_code ec;
  std::filesystem::remove(_path, ec);
  _path.clear();
}

bool spill_file_t::open_read(const std::string &path, uint32_t record_size, std::string &error)
{
  close();
  _path = path;
  _record_size = record_size;
  _writing = false;
  _owns = false;
  _buffer.resize(std::max<size_t>(k_spill_buffer_bytes, record_size));
  _buffer_used = 0;
  _read_cursor = 0;
  _file = fopen(path.c_str(), "rb");
  if (!_file)
  {
    error = fmt::format("cannot open spill file {}", path);
    return false;
  }
  return true;
}

bool spill_file_t::next(const uint8_t *&record, std::string &error)
{
  if (_read_cursor + _record_size > _buffer_used)
  {
    // Carry the partial tail (a record straddling the buffer boundary) to the front and refill.
    const size_t tail = _buffer_used - _read_cursor;
    if (tail)
      memmove(_buffer.data(), _buffer.data() + _read_cursor, tail);
    _read_cursor = 0;
    _buffer_used = tail;
    if (!_file)
      return false;
    const size_t got = fread(_buffer.data() + tail, 1, _buffer.size() - tail, _file);
    _buffer_used += got;
    if (_read_cursor + _record_size > _buffer_used)
    {
      if (_buffer_used != 0 && _buffer_used % _record_size != 0)
        error = fmt::format("spill file {} ends mid-record", _path);
      return false;
    }
  }
  record = _buffer.data() + _read_cursor;
  _read_cursor += _record_size;
  return true;
}

void attribute_amend_t::set_spill_directory(std::string directory)
{
  _spill_directory = std::move(directory);
}

void attribute_amend_t::clear()
{
  for (auto &pending : _pending)
    pending.spill.remove_file();
  _pending.clear();
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

  std::string directory = _spill_directory;
  if (directory.empty())
  {
    std::error_code ec;
    directory = std::filesystem::temp_directory_path(ec).string();
    if (ec)
      directory = ".";
  }

  pending_attribute_t pending;
  pending.name = name;
  pending.key_attribute = key_attribute;
  pending.type = type;
  pending.components = components;
  pending.element_size = uint32_t(element_size);
  pending.record_size = uint32_t(sizeof(uint64_t)) + pending.element_size;
  pending.record_scratch.resize(pending.record_size);
  const std::string path = fmt::format("{}/dew_amend_{}_{}.spill", directory, _spill_serial++, name);
  if (!pending.spill.open_write(path, pending.record_size, error_message))
    return false;
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
    const auto *raw = static_cast<const uint8_t *>(values);
    std::string error;
    for (uint64_t i = 0; i < count; i++)
    {
      // Ascending is tracked, not required. A caller whose keys already rise -- a scan-ordered
      // importer, whose key is (scan << k) | index -- hands the commit a sorted value stream and
      // skips the external sort entirely.
      if (pending.have_last && keys[i] < pending.last_key)
        pending.ascending = false;
      pending.last_key = keys[i];
      pending.have_last = true;
      memcpy(pending.record_scratch.data(), &keys[i], sizeof(uint64_t));
      memcpy(pending.record_scratch.data() + sizeof(uint64_t), raw + i * pending.element_size, pending.element_size);
      if (!pending.spill.append(pending.record_scratch.data(), error))
        return false;
    }
    return true;
  }
  return false;
}

dew_error_t attribute_amend_t::commit(tree_handler_t &tree_handler, storage_handler_t &storage_handler, attributes_configs_t &attributes_configs)
{
  dew_error_t error;
  if (_pending.empty())
    return error;

  const bool trace = std::getenv("DEW_DEBUG_AMEND") != nullptr;
  const uint64_t budget = amend_memory_budget();

  for (auto &attribute : _pending)
  {
    std::string message;
    if (!attribute.spill.flush(message))
      return dew_error_t{-1, message};
    const uint64_t value_count = attribute.spill.records();
    const std::string unsorted_path = attribute.spill.path();
    attribute.spill.close();

    // The values must be in key order for the merge below. Free when the caller already emitted them
    // that way, one external sort otherwise.
    std::string sorted_path = unsorted_path;
    if (!attribute.ascending)
    {
      sorted_path = unsorted_path + ".sorted";
      if (!sort_spill_by_key(unsorted_path, attribute.record_size, value_count, sorted_path, message))
        return dew_error_t{-1, message};
    }

    // Re-snapshotted per attribute, not hoisted: landing an attribute moves every amended unit to a
    // new config, and building the second attribute's config on top of a stale one would drop the
    // first attribute's slot from the mapping while its blob stayed in the location vector -- every
    // attribute of that unit mis-resolved from then on.
    auto units = tree_handler.snapshot_storage_units();
    if (units.empty())
      return dew_error_t{-1, "attribute amend found no units to amend (is every tree loaded?)"};

    size_t amended_units = 0;
    size_t skipped_no_key = 0;
    size_t skipped_have_it = 0;
    size_t passes = 0;
    uint64_t matched_points = 0;
    uint64_t total_points = 0;

    // K PASSES OVER THE UNITS. Each pass takes as many units as fit in the budget, builds the
    // addresses for just those, sorts them in RAM, and merges them against ONE sequential scan of the
    // sorted value file. Memory is the slice, not the dataset -- which is the whole point: the
    // previous design held every (key, value) pair in a hash map, measured at 52 bytes per point.
    //
    // Slicing UNITS rather than partitioning keys is deliberate. A unit's keys are scattered across
    // the whole key space by the morton sort, so a key-partitioned scheme would re-read and
    // re-DECOMPRESS every unit's key blob once per partition, and no unit's output could be finished
    // until the last one. This way each key blob is read exactly once, as before, and the extra cost
    // is K sequential scans of a flat uncompressed file.
    size_t unit_cursor = 0;
    while (unit_cursor < units.size())
    {
      std::vector<address_t> addresses;
      std::vector<attribute_write_t> writes;
      std::vector<storage_unit_append_t> appends;
      std::vector<std::shared_ptr<uint8_t[]>> buffers;
      uint64_t slice_bytes = 0;

      while (unit_cursor < units.size() && slice_bytes < budget)
      {
        auto &unit = units[unit_cursor++];

        // Already has it: an interrupted amend re-run, or a name the dataset genuinely carries.
        // Either way appending a second slot of the same name would leave the config ambiguous.
        if (attributes_configs.get_attribute_index(unit.attributes_id, attribute.name).index >= 0)
        {
          skipped_have_it++;
          continue;
        }
        const auto key_index = attributes_configs.get_attribute_index(unit.attributes_id, attribute.key_attribute);
        // A unit without the key cannot be joined. Legal and expected: LOD levels drop most
        // attributes (the keep-list in get_lod_attribute_mapping), so a key that is not on that list
        // stops the amend at the boundary where provenance stops.
        if (key_index.index < 0 || key_index.index >= int(unit.storage.size()) || key_index.format.components != dew_components_1 || !is_integer_type(key_index.format.type))
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
          return request->error;
        const auto key_element_size = uint32_t(size_for_format(key_index.format.type, key_index.format.components));
        const uint32_t point_count = request->buffer_info.size / key_element_size;
        if (!point_count)
        {
          skipped_no_key++;
          continue;
        }

        const uint32_t slice_unit = uint32_t(buffers.size());
        addresses.reserve(addresses.size() + point_count);
        const auto *keys = static_cast<const uint8_t *>(request->buffer_info.data);
        for (uint32_t p = 0; p < point_count; p++)
          addresses.push_back({widen_key(keys + size_t(p) * key_element_size, key_index.format.type), slice_unit, p});
        // The key bytes are not needed past this point; only the addresses are. Releasing the read
        // here is what keeps a slice's memory to the addresses plus the output buffers.
        request.reset();

        // Zero-initialized, deliberately: a point with no entry in the table reads back as zero,
        // which is exactly what a node that lacks the attribute entirely reads back as. The two
        // states are indistinguishable to a reader, which is what makes a partial amend legal.
        const uint32_t buffer_size = point_count * attribute.element_size;
        buffers.emplace_back(new uint8_t[buffer_size]());
        total_points += point_count;

        attribute_write_t write;
        write.data = buffers.back();
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

        slice_bytes += uint64_t(point_count) * (sizeof(address_t) + attribute.element_size);
      }

      if (writes.empty())
        continue;
      passes++;

      std::sort(addresses.begin(), addresses.end(), [](const address_t &a, const address_t &b) { return a.key < b.key; });

      // THE MERGE. Both sides sorted by key, two cursors, one sequential scan.
      if (value_count)
      {
        spill_file_t values;
        if (!values.open_read(sorted_path, attribute.record_size, message))
          return dew_error_t{-1, message};
        size_t cursor = 0;
        const uint8_t *record = nullptr;
        bool have_record = values.next(record, message);
        if (!message.empty())
          return dew_error_t{-1, message};
        while (have_record && cursor < addresses.size())
        {
          const uint64_t key = record_key(record);
          // Consume the whole run of equal keys, keeping the LAST value: add_data documents that a
          // repeated key overwrites its earlier value, and the sort preserves the order they arrived
          // in (see sort_spill_by_key).
          memcpy(attribute.record_scratch.data(), record, attribute.record_size);
          while ((have_record = values.next(record, message)) && record_key(record) == key)
            memcpy(attribute.record_scratch.data(), record, attribute.record_size);
          if (!message.empty())
            return dew_error_t{-1, message};

          while (cursor < addresses.size() && addresses[cursor].key < key)
            cursor++;
          // EVERY address for this key, not just the first. A key appears in its leaf AND in every
          // LOD ancestor that sampled it -- taking one would leave the pyramid unfilled, which shows
          // up as coloured leaves over black LOD levels.
          while (cursor < addresses.size() && addresses[cursor].key == key)
          {
            const auto &address = addresses[cursor++];
            memcpy(buffers[address.unit].get() + size_t(address.slot) * attribute.element_size, attribute.record_scratch.data() + sizeof(uint64_t), attribute.element_size);
            matched_points++;
          }
        }
      }

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
        return dew_error_t{-1, "attribute amend wrote a different number of blobs than it asked for"};

      for (size_t i = 0; i < appends.size(); i++)
        appends[i].locations.push_back(locations[i]);
      amended_units += appends.size();
      if (!tree_handler.append_storage_units(std::move(appends)))
        return dew_error_t{-1, "attribute amend could not reach the tree loop to record the blobs it wrote"};
    }

    if (trace)
      fmt::print(stderr, "amend '{}': {} units in {} passes, {} without the key '{}', {} already had it; {}/{} points matched from {} values\n", attribute.name, amended_units, passes, skipped_no_key,
                 attribute.key_attribute, skipped_have_it, matched_points, total_points, value_count);

    if (sorted_path != unsorted_path)
    {
      std::error_code ec;
      std::filesystem::remove(sorted_path, ec);
    }
  }

  clear();
  return error;
}

} // namespace dew::converter
