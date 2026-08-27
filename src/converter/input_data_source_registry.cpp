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
#include "input_data_source_registry.hpp"

#include "memory_writer.hpp"
#include "morton_tree_coordinate_transform.hpp"

#include <cstring>
#include <mutex>

namespace dew::converter
{
using namespace dew::core;
static auto &get_item(input_data_id_t id, ankerl::unordered_dense::map<uint32_t, input_data_source_impl_t> &registry)
{
  assert(id.data < uint32_t(1) << 31);
  assert(registry.contains(id.data));
  return registry[id.data];
}

input_data_source_registry_t::input_data_source_registry_t()
  : _input_data_with_sub_parts(0)
  , _input_data_inserted_to_tree(0)
  , _input_data_id_done_count(0)
  , _unsorted_input_sources_dirty(true)
{
}

// Process-wide id counter (ids are registry-keyed; sharing across instances only skips values).
// On resume, ensure_next_input_id_above bumps it past every persisted id so re-registered and new
// files never collide with ids already referenced by the tree's storage maps.
static uint32_t g_next_input_id = 0;

input_data_id_t get_next_input_id()
{
  input_data_id_t ret;
  ret.data = g_next_input_id++;
  ret.sub = 0;
  return ret;
}

void ensure_next_input_id_above(uint32_t max_seen_id)
{
  if (g_next_input_id <= max_seen_id)
    g_next_input_id = max_seen_id + 1;
}

input_data_reference_t input_data_source_registry_t::register_file(std::unique_ptr<char[]> &&name, uint32_t name_length, bool *already_done)
{
  std::unique_lock<std::mutex> lock(_mutex);
  if (already_done)
    *already_done = false;
  // Resume: a re-added input whose earlier run fully completed (per the restored snapshot) is
  // skipped -- re-reading it would duplicate its points in the tree. Matched by name.
  for (auto &kv : _registry)
  {
    auto &item = kv.second;
    if (item.name_length == name_length && memcmp(item.name.get(), name.get(), name_length) == 0)
    {
      if (already_done && item.read_finished && item.inserted_into_tree == item.sub_count)
        *already_done = true;
      return {item.input_id, {item.name.get(), item.name_length}};
    }
  }
  auto input_id = get_next_input_id();
  auto &item = _registry[input_id.data];
  // morton_min is a min-accumulator in handle_sorted_points; it must start at the max
  // sentinel (all 0xFF), mirroring storage_header_initialize. Left at 0 it would stay 0
  // forever and stall incremental LOD generation. (morton_max correctly starts at 0.)
  morton::morton_init_max(item.morton_min);
  item.input_id = input_id;
  item.name = std::move(name);
  item.name_length = name_length;
  item.attribute_id = {0};
  item.public_header = {};
  return {input_id, {item.name.get(), item.name_length}};
}

void input_data_source_registry_t::register_pre_init_result(const tree_config_t &tree_config, input_data_id_t id, bool found_min, double (&min)[3], uint64_t approximate_point_count, uint8_t approximate_point_size_bytes, uint64_t input_file_size_bytes)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);
  item.approximate_point_count = approximate_point_count;
  item.approximate_point_size_bytes = approximate_point_size_bytes;
  item.input_file_size_bytes = input_file_size_bytes;
  if (found_min)
    convert_pos_to_morton(tree_config.scale, tree_config.offset, min, item.input_order);
  else
    memset(&item.input_order, 0, sizeof(item.input_order));
  item.pre_init_settled = true;
  _unsorted_input_sources_dirty = true;
}

void input_data_source_registry_t::handle_input_init(input_data_id_t id, attributes_id_t attributes_id, dew_converter_header_t public_header)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);
  item.attribute_id = attributes_id;
  item.public_header = public_header;
}

void input_data_source_registry_t::handle_sub_added(input_data_id_t id)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);
  item.sub_count++;
  _input_data_with_sub_parts++;
}

void input_data_source_registry_t::handle_sorted_points(input_data_id_t id, const morton::morton192_t &min, const morton::morton192_t &max)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);

  if (min < item.morton_min)
    item.morton_min = min;
  if (max > item.morton_max)
    item.morton_max = max;
}

void input_data_source_registry_t::handle_points_written(input_data_id_t id, std::vector<storage_location_t> &&location)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);
  if (id.sub >= item.storage_locations.size())
  {
    item.storage_locations.resize(item.sub_count);
  }
  item.storage_locations = std::move(location);
}

// An input is retired EXACTLY once. Both retire paths can fire for the same input -- a file whose
// pre-init fails after it was already dispatched gets one from the reader and one from the failure
// -- and a double count leaves _input_data_id_done_count above _registry.size(), which makes
// all_inserted_into_tree() false for the rest of the run: the processor never goes idle and
// wait_idle() blocks forever.
void input_data_source_registry_t::handle_reading_done(input_data_id_t id)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);
  if (item.read_finished)
    return;
  item.read_finished = true;
  _input_data_id_done_count++;
}

void input_data_source_registry_t::handle_file_failed(input_data_id_t id)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);
  // A failed pre-init settles the file: never dispatch it, and drop it from a heap already built.
  item.pre_init_settled = true;
  item.read_started = true;
  _unsorted_input_sources_dirty = true;
  if (item.read_finished)
    return;
  item.read_finished = true;
  _input_data_id_done_count++;
}

void input_data_source_registry_t::handle_tree_done_with_input(input_data_id_t id)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);
  item.inserted_into_tree++;
  _input_data_inserted_to_tree++;
}

bool input_data_source_registry_t::all_inserted_into_tree() const
{
  std::unique_lock<std::mutex> lock(_mutex);
  return _registry.size() == _input_data_id_done_count && _input_data_with_sub_parts == _input_data_inserted_to_tree;
}

std::optional<input_data_next_input_t> input_data_source_registry_t::next_input_to_process()
{
  std::unique_lock<std::mutex> lock(_mutex);
  // Min-heap on input_order (the morton of each file's pre-init aabb-min corner): files must be
  // dispatched in rising min-morton order or the done-morton watermark (and everything built on
  // it: incremental LOD, subtree finalization, upload) is meaningless. The SAME comparator must
  // be used by make_heap and pop_heap — pop_heap with the default uint32_t compare re-heapifies
  // by raw file id and silently breaks the ordering.
  auto input_order_cmp = [&](uint32_t a, uint32_t b) { return _registry[a].input_order > _registry[b].input_order; };
  if (_unsorted_input_sources_dirty)
  {
    _unsorted_input_sources_dirty = false;
    _unsorted_input_sources = {};
    for (auto &item : _registry)
    {
      // Only files whose pre-init has answered. Dispatching one while its pre_init is still in
      // flight races the failure path against the reader's completion, and both retire it.
      if (!item.second.read_started && item.second.pre_init_settled)
      {
        _unsorted_input_sources.push_back(item.first);
      }
    }
    std::make_heap(_unsorted_input_sources.begin(), _unsorted_input_sources.end(), input_order_cmp);
  }
  if (_unsorted_input_sources.empty())
    return {};

  std::pop_heap(_unsorted_input_sources.begin(), _unsorted_input_sources.end(), input_order_cmp);
  auto id = _unsorted_input_sources.back();
  _unsorted_input_sources.pop_back();
  _sorted_input_sources.push_back(id);
  auto &item = _registry[id];
  // Mark the source as dispatched so a later dirty-rebuild (triggered by a subsequent
  // pre-init completion) does not re-enqueue and re-read this same file, which would
  // duplicate its points in the tree. Mirrors handle_file_failed setting the same flag.
  item.read_started = true;
  input_data_next_input_t ret;
  ret.approximate_point_count = item.approximate_point_count;
  ret.approximate_point_size_bytes = item.approximate_point_size_bytes;
  ret.id = {id, 0};
  ret.name.name = item.name.get();
  ret.name.name_length = item.name_length;
  return ret;
}

uint64_t input_data_source_registry_t::get_approximate_size(input_data_id_t id)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(id, _registry);
  return item.approximate_point_count * item.approximate_point_size_bytes;
}

std::optional<morton::morton192_t> input_data_source_registry_t::get_done_morton()
{
  std::unique_lock<std::mutex> lock(_mutex);
  if (_sorted_input_sources.empty())
    return {};
  while (_done_prefix_index < _sorted_input_sources.size())
  {
    auto &item = _registry[_sorted_input_sources[_done_prefix_index]];
    if (!item.read_finished || item.inserted_into_tree < item.sub_count)
      break;
    _done_prefix_index++;
  }
  if (_done_prefix_index == 0)
    return {};

  // Everything STRICTLY below the returned boundary is final: no current or future input can add
  // a point there. This is the correctness anchor for incremental LOD, subtree finalization and
  // upload, so the boundary must be a true lower bound on every point of every not-done file.
  morton::morton192_t boundary;
  memset(&boundary, 0xFF, sizeof(boundary));
  if (_done_prefix_index < _sorted_input_sources.size())
  {
    auto &next = _registry[_sorted_input_sources[_done_prefix_index]];
    // morton_min is a min-ACCUMULATOR seeded all-0xFF; it only becomes meaningful once the file's
    // first sorted chunk lands. input_order (morton of the pre-init aabb-min corner) is a true
    // lower bound on every point in the file (morton is monotone per coordinate, so the min
    // corner's code <= any point's code in the box). Take the min so an in-flight file whose
    // chunks haven't streamed yet never lets the watermark overclaim. A file without a found
    // pre-init min has input_order 0 -> boundary 0 -> no progress until it's done: conservative.
    boundary = next.morton_min < next.input_order ? next.morton_min : next.input_order;
  }
  // Clamp by every registered-but-undispatched file too: a file added mid-conversion can carry a
  // LOWER min-morton than anything currently in flight (the heap only orders what exists at
  // dispatch time). Skipping this made the boundary overclaim regions such a file later writes
  // into. Registered files without a pre-init result yet have input_order 0 (value-initialized),
  // which conservatively pins the boundary at 0 until their bounds are known. Iterate _registry
  // directly (not _unsorted_input_sources, which may be stale under the dirty flag); O(#files)
  // per call, called per file-completion / index-write, negligible.
  for (auto &kv : _registry)
  {
    auto &item = kv.second;
    if (!item.read_started && item.input_order < boundary)
      boundary = item.input_order;
  }
  // Boundary 0 proves nothing final; report "no watermark" instead of a zero-advance.
  morton::morton192_t zero;
  memset(&zero, 0, sizeof(zero));
  if (!(zero < boundary))
    return {};
  return boundary;
}

input_data_source_t input_data_source_registry_t::get(input_data_id_t input_id)
{
  std::unique_lock<std::mutex> lock(_mutex);
  auto &item = get_item(input_id, _registry);

  input_data_source_t ret;
  ret.input_id = item.input_id;
  ret.attribute_id = item.attribute_id;
  ret.name = {item.name.get(), item.name_length};
  ret.public_header = item.public_header;
  return ret;
}

// ---------------------------------------------------------------------------------------------
// Registry-blob v2 input-registry section ('ISR1'). Persists exactly what resume needs: per file
// the name (match key for re-added inputs), morton order/bounds, sub/inserted counts and read
// flags; plus the sorted dispatch order and done-prefix so the done-morton watermark restores.
// Not persisted: storage_locations / public_header / attribute_id (the tree's storage maps and
// attributes_configs already hold everything a DONE file contributed; a not-done file is re-read).
static constexpr uint32_t k_input_registry_magic = 0x31525349u; // 'ISR1' little-endian

std::vector<uint8_t> input_data_source_registry_t::serialize() const
{
  std::unique_lock<std::mutex> lock(_mutex);

  uint32_t size = 0;
  size += sizeof(k_input_registry_magic);
  size += sizeof(uint32_t); // file count
  for (auto &kv : _registry)
  {
    size += sizeof(uint32_t);                        // id
    size += sizeof(uint32_t) + kv.second.name_length; // name
    size += 3 * uint32_t(sizeof(morton::morton192_t)); // input_order, morton_min, morton_max
    size += 2 * uint32_t(sizeof(uint32_t));          // sub_count, inserted_into_tree
    size += 2 * uint32_t(sizeof(uint8_t));           // read_started, read_finished
  }
  size += sizeof(uint32_t) + uint32_t(_sorted_input_sources.size()) * uint32_t(sizeof(uint32_t));
  size += sizeof(uint32_t); // done prefix index

  std::vector<uint8_t> out(size);
  uint8_t *ptr = out.data();
  uint8_t *end_ptr = ptr + out.size();
  bool ok = write_memory(ptr, end_ptr, k_input_registry_magic);
  ok = ok && write_memory(ptr, end_ptr, uint32_t(_registry.size()));
  for (auto &kv : _registry)
  {
    auto &item = kv.second;
    ok = ok && write_memory(ptr, end_ptr, kv.first);
    ok = ok && write_memory(ptr, end_ptr, item.name_length);
    if (ok && item.name_length)
    {
      if (ptr + item.name_length > end_ptr)
        ok = false;
      else
      {
        memcpy(ptr, item.name.get(), item.name_length);
        ptr += item.name_length;
      }
    }
    ok = ok && write_memory(ptr, end_ptr, item.input_order);
    ok = ok && write_memory(ptr, end_ptr, item.morton_min);
    ok = ok && write_memory(ptr, end_ptr, item.morton_max);
    ok = ok && write_memory(ptr, end_ptr, item.sub_count);
    ok = ok && write_memory(ptr, end_ptr, item.inserted_into_tree);
    ok = ok && write_memory(ptr, end_ptr, uint8_t(item.read_started ? 1 : 0));
    ok = ok && write_memory(ptr, end_ptr, uint8_t(item.read_finished ? 1 : 0));
  }
  ok = ok && write_memory(ptr, end_ptr, uint32_t(_sorted_input_sources.size()));
  ok = ok && write_vec_type(ptr, end_ptr, _sorted_input_sources);
  ok = ok && write_memory(ptr, end_ptr, _done_prefix_index);
  assert(ok && ptr == end_ptr);
  if (!ok)
    return {};
  return out;
}

dew_error_t input_data_source_registry_t::deserialize(const uint8_t *data, uint32_t size)
{
  std::unique_lock<std::mutex> lock(_mutex);
  const uint8_t *ptr = data;
  const uint8_t *end_ptr = data + size;
  const dew_error_t invalid = {1, "Invalid input registry data"};

  uint32_t magic = 0;
  if (!read_memory(ptr, end_ptr, magic) || magic != k_input_registry_magic)
    return invalid;
  uint32_t file_count = 0;
  if (!read_memory(ptr, end_ptr, file_count))
    return invalid;

  _registry.clear();
  _input_data_with_sub_parts = 0;
  _input_data_inserted_to_tree = 0;
  _input_data_id_done_count = 0;
  uint32_t max_id = 0;
  for (uint32_t i = 0; i < file_count; i++)
  {
    uint32_t id = 0;
    if (!read_memory(ptr, end_ptr, id))
      return invalid;
    auto &item = _registry[id];
    item.input_id = {id, 0};
    if (!read_memory(ptr, end_ptr, item.name_length))
      return invalid;
    if (ptr + item.name_length > end_ptr)
      return invalid;
    item.name.reset(new char[item.name_length]);
    memcpy(item.name.get(), ptr, item.name_length);
    ptr += item.name_length;
    uint8_t read_started = 0;
    uint8_t read_finished = 0;
    bool ok = read_memory(ptr, end_ptr, item.input_order);
    ok = ok && read_memory(ptr, end_ptr, item.morton_min);
    ok = ok && read_memory(ptr, end_ptr, item.morton_max);
    ok = ok && read_memory(ptr, end_ptr, item.sub_count);
    ok = ok && read_memory(ptr, end_ptr, item.inserted_into_tree);
    ok = ok && read_memory(ptr, end_ptr, read_started);
    ok = ok && read_memory(ptr, end_ptr, read_finished);
    if (!ok)
      return invalid;
    item.read_started = read_started != 0;
    item.read_finished = read_finished != 0;
    // The snapshot has no pre-init section, so restored inputs keep today's dispatch semantics:
    // eligible immediately. Re-adding the file by name runs pre_init again anyway, which is what
    // refreshes the approximate sizes.
    item.pre_init_settled = true;
    _input_data_with_sub_parts += item.sub_count;
    _input_data_inserted_to_tree += item.inserted_into_tree;
    if (item.read_finished)
      _input_data_id_done_count++;
    if (id > max_id)
      max_id = id;
  }
  uint32_t sorted_count = 0;
  if (!read_memory(ptr, end_ptr, sorted_count))
    return invalid;
  if (!read_vec_type(ptr, end_ptr, _sorted_input_sources, sorted_count))
    return invalid;
  if (!read_memory(ptr, end_ptr, _done_prefix_index))
    return invalid;

  // A file that was dispatched but not finished when the snapshot was taken must be re-read from
  // scratch on resume: its already-written chunks were only referenced by tree state committed in
  // the same checkpoint (consistent), but the READ never completed -- clear its dispatch flag so
  // next_input_to_process re-enqueues it, and drop it from the sorted (dispatched) list. Its
  // partial counters stay: sub_count/inserted_into_tree recorded at checkpoint describe chunks the
  // committed tree DOES contain. NOTE (phase 7 wires the full resume flow): re-reading a partially
  // inserted file would duplicate those chunks -- the resume path must instead either re-add such
  // files with a fresh id after truncating their partial state, or (v1) refuse resume when a
  // partially-read file exists. Kept conservative here; the format just records the truth.
  std::vector<uint32_t> still_sorted;
  still_sorted.reserve(_sorted_input_sources.size());
  uint32_t new_done_prefix = 0;
  for (uint32_t i = 0; i < uint32_t(_sorted_input_sources.size()); i++)
  {
    auto id = _sorted_input_sources[i];
    auto it = _registry.find(id);
    if (it == _registry.end())
      return invalid;
    auto &item = it->second;
    const bool done = item.read_finished && item.inserted_into_tree == item.sub_count;
    if (done)
    {
      still_sorted.push_back(id);
      if (i < _done_prefix_index)
        new_done_prefix = uint32_t(still_sorted.size());
    }
    else
    {
      item.read_started = false;
    }
  }
  _sorted_input_sources = std::move(still_sorted);
  _done_prefix_index = new_done_prefix;
  _unsorted_input_sources = {};
  _unsorted_input_sources_dirty = true;

  ensure_next_input_id_above(max_id);
  return {};
}

} // namespace dew::converter
