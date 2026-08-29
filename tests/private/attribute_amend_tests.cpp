/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2026  Jørgen Lind
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

// Can a converted dataset GAIN an attribute after the fact?
//
// The intended workflow is iterative: convert positions, look at the result, then add colour or
// intensity to the points already stored rather than reconverting from source. That only works if
// the format and the LOD machinery tolerate a dataset whose nodes do not all carry the same
// attributes -- during such a job they provably do not, and a half-finished job has to remain a
// valid dataset.
//
// The good news is that this is not a new shape. LOD attribute slimming already produces it in
// EVERY dataset: coarse nodes keep only position + rgb/intensity/classification, so leaves and LOD
// nodes have different attribute sets and different attributes_id_t configs. The tests here pin the
// properties an amend depends on, most of which are consequences of that existing design:
//
//   1. the on-disk storage map can hold units with DIFFERENT numbers of attribute slots;
//   2. a unit can be re-described in place (more slots, a new config) without leaking its old
//      blobs and without corrupting its reference count;
//   3. the LOD destination config is the UNION of its children's, and each child's mapping into it
//      is resolved BY NAME, with a clean sentinel for "this child does not have it";
//   4. -- the one that was actually broken -- a destination buffer position that NO child fills is
//      ZERO, not uninitialized heap.
//
// (4) is why this file exists. The union/-1 machinery was already correct; the buffers it wrote
// into were not zeroed, so a point whose source lacked the attribute took whatever bytes happened
// to be there. Reachable today with two input files of differing attribute sets; the normal case
// the moment half a dataset has been amended.

#include <chrono>
#include <tuple>

#include <doctest/doctest.h>

#include "attributes_configs.hpp"
#include "dataset_types.hpp"
#include "input_header.hpp"
#include "input_storage_map.hpp"

#include <dew/core/default_attribute_names.h>
#include <dew/core/format.h>
#include <dew/core/types.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace dew::core;
using namespace dew::converter;

namespace
{

// dew_attributes_t owns its name storage separately from the attribute records, and each record's
// `name` points into it -- so it is built through the public adder, exactly as a file converter's
// init callback does.
dew_attributes_t make_attributes(const std::vector<std::tuple<const char *, dew_type_t, dew_components_t>> &spec)
{
  dew_attributes_t attributes;
  for (const auto &[name, type, components] : spec)
    dew_attributes_add_attribute(&attributes, name, uint32_t(strlen(name)), type, components);
  return attributes;
}

std::vector<std::string> names_of(const dew_attributes_t &attributes)
{
  std::vector<std::string> out;
  out.reserve(attributes.attributes.size());
  for (const auto &attribute : attributes.attributes)
    out.emplace_back(attribute.name, attribute.name_size);
  return out;
}

storage_location_t make_location(uint32_t file_id, uint64_t offset, uint32_t size)
{
  storage_location_t location;
  location.file_id = file_id;
  location.offset = offset;
  location.size = size;
  return location;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// 1. The storage map holds units of differing width.

TEST_CASE("amend: a storage map with differing slot counts round-trips")
{
  // The width is per entry (each carries its own storage_size), so a unit that has gained an
  // attribute can sit beside one that has not. If this ever stopped holding, an amend would need a
  // format version bump and every existing dataset would have to be rewritten -- so it is worth
  // asserting rather than assuming.
  input_storage_map_t map;
  const input_data_id_t narrow{1, 0};
  const input_data_id_t wide{2, 0};
  const input_data_id_t widest{3, 0};

  map.add_storage(narrow, attributes_id_t{7}, {make_location(0, 100, 10)});
  map.add_storage(wide, attributes_id_t{8}, {make_location(0, 200, 20), make_location(0, 300, 30)});
  map.add_storage(widest, attributes_id_t{9}, {make_location(1, 400, 40), make_location(1, 500, 50), make_location(1, 600, 60)});

  std::vector<uint8_t> buffer(map.serialized_size());
  const auto written = map.serialize(buffer.data(), buffer.data() + buffer.size());
  REQUIRE(written.first);
  REQUIRE(written.second == buffer.data() + buffer.size());

  input_storage_map_t restored;
  const auto read = restored.deserialize(buffer.data(), buffer.data() + buffer.size());
  REQUIRE(read.first);

  for (const auto id : {narrow, wide, widest})
  {
    REQUIRE(restored.contains(id));
    REQUIRE(restored.attribute_id(id).data == map.attribute_id(id).data);
    REQUIRE(restored.info(id).second.size() == map.info(id).second.size());
    REQUIRE(restored.ref_count(id) == map.ref_count(id));
  }
  // The widths really are different -- otherwise this test would pass on a map that had silently
  // truncated every entry to a common length.
  REQUIRE(restored.info(narrow).second.size() == 1);
  REQUIRE(restored.info(wide).second.size() == 2);
  REQUIRE(restored.info(widest).second.size() == 3);
  REQUIRE(restored.location(widest, 2).offset == 600);
  MESSAGE("slot counts after round-trip: ", restored.info(narrow).second.size(), " ", restored.info(wide).second.size(), " ", restored.info(widest).second.size());
}

// ---------------------------------------------------------------------------------------------
// 2. A unit can be re-described in place.

TEST_CASE("amend: re-describing a unit keeps one reference and discards the old blobs")
{
  // This is what an amend does to a node -- and what LOD regeneration already does to a node it
  // re-runs. Both need the same two properties, and the second one used to be wrong: the old code
  // incremented the reference count on a replacement, so the matching dereference could never reach
  // zero and the unit's blobs were never freed.
  input_storage_map_t map;
  const input_data_id_t id{42, 0};

  map.add_storage(id, attributes_id_t{1}, {make_location(0, 1000, 64)});
  REQUIRE(map.ref_count(id) == 1);
  REQUIRE(map.take_discarded().empty());

  // The amend: one more slot, and a config that is the old one plus the new attribute.
  map.add_storage(id, attributes_id_t{2}, {make_location(0, 1000, 64), make_location(0, 2000, 32)});

  REQUIRE(map.ref_count(id) == 1); // re-described, NOT referenced twice
  REQUIRE(map.attribute_id(id).data == 2);
  REQUIRE(map.info(id).second.size() == 2);

  // The superseded blob is recorded for the next checkpoint to free rather than leaked.
  auto discarded = map.take_discarded();
  REQUIRE(discarded.size() == 1);
  REQUIRE(discarded[0].offset == 1000);

  // And the single reference really does release it.
  map.dereference(id);
  REQUIRE(!map.contains(id));
}

TEST_CASE("amend: a fresh id still takes its first reference")
{
  // The companion to the test above: the replacement path must not cost the fresh path its
  // reference, or nothing would ever be retained.
  input_storage_map_t map;
  const input_data_id_t id{7, 0};
  map.add_storage(id, attributes_id_t{0}, {make_location(0, 10, 10)});
  REQUIRE(map.ref_count(id) == 1);
  map.add_ref(id);
  REQUIRE(map.ref_count(id) == 2);
  map.dereference(id);
  REQUIRE(map.contains(id));
  map.dereference(id);
  REQUIRE(!map.contains(id));
}

// ---------------------------------------------------------------------------------------------
// 3. The LOD destination is a union, mapped by name.

TEST_CASE("amend: the LOD destination unions its children and maps them by name")
{
  // The amend case exactly: some children have gained rgb, some have not. The destination must
  // carry rgb (or the attribute could never reach the coarse levels at all), and the child that
  // lacks it must map to the -1 sentinel rather than to some other child's slot.
  attributes_configs_t configs;
  const auto plain = configs.get_attribute_config_index(make_attributes({{DEW_ATTRIBUTE_XYZ, dew_type_m32, dew_components_1}}));
  const auto amended = configs.get_attribute_config_index(
    make_attributes({{DEW_ATTRIBUTE_XYZ, dew_type_m32, dew_components_1}, {DEW_ATTRIBUTE_RGB, dew_type_u8, dew_components_3}}));
  REQUIRE(plain.data != amended.data); // differing sets are necessarily different configs

  const attributes_id_t sources[] = {plain, amended};
  const auto mapping = configs.get_lod_attribute_mapping(1, sources, sources + 2);

  const auto destination = names_of(configs.get(mapping.destination_id));
  REQUIRE(destination.size() == 2);
  CHECK(destination[0] == DEW_ATTRIBUTE_XYZ);
  CHECK(destination[1] == DEW_ATTRIBUTE_RGB);

  // Position is always slot 0 and always present; rgb resolves for the amended child only.
  const auto &from_plain = mapping.get_source_mapping(plain);
  const auto &from_amended = mapping.get_source_mapping(amended);
  REQUIRE(from_plain.source_attributes.size() == 2);
  REQUIRE(from_amended.source_attributes.size() == 2);
  CHECK(from_plain.source_attributes[0].source_index == 0);
  CHECK(from_amended.source_attributes[0].source_index == 0);
  CHECK(from_plain.source_attributes[1].source_index == -1); // the sentinel the generator skips on
  CHECK(from_amended.source_attributes[1].source_index == 1);
  MESSAGE("destination = [", destination[0], ", ", destination[1], "]  plain->rgb = ", from_plain.source_attributes[1].source_index);
}

TEST_CASE("amend: the destination keeps the pre-amend attribute order")
{
  // Load-bearing, and the reason this test exists rather than being assumed: readers resolve an
  // attribute by NAME against the node's own config, so a reordered destination is not in itself
  // wrong -- but the mapping and the buffers must agree on the order. Pinning "old order, new
  // attribute appended" keeps the two trivially in step, and makes an accidental reorder (which
  // would silently mis-map every pre-existing attribute of every LOD node) fail here instead.
  attributes_configs_t configs;
  const auto before = configs.get_attribute_config_index(make_attributes({{DEW_ATTRIBUTE_XYZ, dew_type_m32, dew_components_1},
                                                                          {DEW_ATTRIBUTE_INTENSITY, dew_type_u16, dew_components_1},
                                                                          {DEW_ATTRIBUTE_CLASSIFICATION, dew_type_u8, dew_components_1}}));
  const auto after = configs.get_attribute_config_index(make_attributes({{DEW_ATTRIBUTE_XYZ, dew_type_m32, dew_components_1},
                                                                         {DEW_ATTRIBUTE_INTENSITY, dew_type_u16, dew_components_1},
                                                                         {DEW_ATTRIBUTE_CLASSIFICATION, dew_type_u8, dew_components_1},
                                                                         {DEW_ATTRIBUTE_RGB, dew_type_u8, dew_components_3}}));

  const attributes_id_t sources[] = {before, after};
  const auto mapping = configs.get_lod_attribute_mapping(1, sources, sources + 2);
  const auto destination = names_of(configs.get(mapping.destination_id));

  REQUIRE(destination.size() == 4);
  CHECK(destination[0] == DEW_ATTRIBUTE_XYZ);
  CHECK(destination[1] == DEW_ATTRIBUTE_INTENSITY);
  CHECK(destination[2] == DEW_ATTRIBUTE_CLASSIFICATION);
  CHECK(destination[3] == DEW_ATTRIBUTE_RGB);

  // Every pre-existing attribute still maps to the slot it had in the un-amended child.
  const auto &from_before = mapping.get_source_mapping(before);
  CHECK(from_before.source_attributes[0].source_index == 0);
  CHECK(from_before.source_attributes[1].source_index == 1);
  CHECK(from_before.source_attributes[2].source_index == 2);
  CHECK(from_before.source_attributes[3].source_index == -1);
}

TEST_CASE("amend: the union does not depend on the order the children are listed")
{
  // Children reach the LOD generator in whatever order the tree walk produced. Two nodes with the
  // same set of child configs must land on the SAME destination config, or the dataset accumulates
  // gratuitous near-duplicate configs (and a reader that cached a per-config lookup would have to
  // redo it per node).
  attributes_configs_t configs;
  const auto a = configs.get_attribute_config_index(make_attributes({{DEW_ATTRIBUTE_XYZ, dew_type_m32, dew_components_1}, {DEW_ATTRIBUTE_RGB, dew_type_u8, dew_components_3}}));
  const auto b = configs.get_attribute_config_index(make_attributes({{DEW_ATTRIBUTE_XYZ, dew_type_m32, dew_components_1}, {DEW_ATTRIBUTE_INTENSITY, dew_type_u16, dew_components_1}}));

  const attributes_id_t forward[] = {a, b};
  const attributes_id_t reverse[] = {b, a};
  const auto first = configs.get_lod_attribute_mapping(1, forward, forward + 2);
  const auto second = configs.get_lod_attribute_mapping(1, reverse, reverse + 2);
  CHECK(first.destination_id.data == second.destination_id.data);

  // Both visual attributes survive slimming, so the union really is wider than either child.
  const auto destination = names_of(configs.get(first.destination_id));
  REQUIRE(destination.size() == 3);
  MESSAGE("union = [", destination[0], ", ", destination[1], ", ", destination[2], "]");
}

TEST_CASE("amend: LOD slimming keeps the visual attributes and drops the rest")
{
  // Which attributes are worth back-filling into the coarse levels at all. rgb and intensity reach
  // them; anything else (a join key, gps_time) is dropped, so amending it is leaf-only work. This
  // is the property that makes "add scan_id + point_index as ordinary attributes" nearly free.
  attributes_configs_t configs;
  const auto leaf = configs.get_attribute_config_index(make_attributes({{DEW_ATTRIBUTE_XYZ, dew_type_m32, dew_components_1},
                                                                        {DEW_ATTRIBUTE_RGB, dew_type_u8, dew_components_3},
                                                                        {DEW_ATTRIBUTE_INTENSITY, dew_type_u16, dew_components_1},
                                                                        {DEW_ATTRIBUTE_GPS_TIME, dew_type_r64, dew_components_1},
                                                                        {"scan_id", dew_type_u16, dew_components_1}}));
  const attributes_id_t sources[] = {leaf};
  const auto mapping = configs.get_lod_attribute_mapping(1, sources, sources + 1);
  const auto destination = names_of(configs.get(mapping.destination_id));

  REQUIRE(destination.size() == 3);
  CHECK(destination[0] == DEW_ATTRIBUTE_XYZ);
  CHECK(destination[1] == DEW_ATTRIBUTE_RGB);
  CHECK(destination[2] == DEW_ATTRIBUTE_INTENSITY);

  // ...and with lod_all_attributes the whole set is carried instead.
  const auto all = configs.get_lod_attribute_mapping(1, sources, sources + 1, false, true);
  CHECK(names_of(configs.get(all.destination_id)).size() == 5);
}

// ---------------------------------------------------------------------------------------------
// 4. The buffer a missing source leaves behind.

TEST_CASE("amend: attribute buffers start zeroed, not on whatever the heap held")
{
  // THE BUG THIS FILE WAS WRITTEN FOR. A destination attribute set is the union of its sources', and
  // the LOD generator skips a source that lacks one (source_index < 0). Those points' bytes are
  // never written -- so unless the buffer was zeroed at allocation they carry heap garbage into the
  // dataset. `new uint8_t[n]` default-initializes, which for a scalar means no initialization at
  // all; the fix is `new uint8_t[n]()`.
  //
  // Allocating and freeing a dirtied block of the same shape first makes the failure deterministic
  // rather than depending on what the allocator happened to hand back.
  const std::vector<point_format_t> formats = {{dew_type_m32, dew_components_1}, {dew_type_u8, dew_components_3}};
  const uint32_t point_count = 4096;

  for (int attempt = 0; attempt < 4; attempt++)
  {
    {
      attribute_buffers_t dirty;
      attribute_buffers_initialize(formats, dirty, point_count);
      for (auto &blob : dirty.buffers)
        memset(blob.data, 0xAB, blob.size);
    }
    attribute_buffers_t buffers;
    attribute_buffers_initialize(formats, buffers, point_count);
    REQUIRE(buffers.buffers.size() == formats.size());
    for (size_t i = 0; i < buffers.buffers.size(); i++)
    {
      const auto &blob = buffers.buffers[i];
      REQUIRE(blob.size > 0);
      uint64_t non_zero = 0;
      for (uint32_t byte = 0; byte < blob.size; byte++)
        non_zero += static_cast<const uint8_t *>(blob.data)[byte] != 0 ? 1 : 0;
      REQUIRE(non_zero == 0);
    }
  }
}

TEST_CASE("amend: the morton-adopting overload zeroes the attribute buffers too")
{
  // The overload the LOD generator actually uses: slot 0 is the morton buffer it just built (and
  // fully wrote), every later slot is freshly allocated and must be zeroed for the same reason.
  const std::vector<point_format_t> formats = {{dew_type_m32, dew_components_1}, {dew_type_u8, dew_components_3}, {dew_type_u16, dew_components_1}};
  const uint32_t point_count = 1024;
  const uint32_t morton_bytes = size_for_format(dew_type_m32) * uint32_t(dew_components_1) * point_count;

  {
    attribute_buffers_t dirty;
    attribute_buffers_initialize(formats, dirty, point_count);
    for (auto &blob : dirty.buffers)
      memset(blob.data, 0xCD, blob.size);
  }

  std::unique_ptr<uint8_t[]> morton(new uint8_t[morton_bytes]);
  memset(morton.get(), 0x5A, morton_bytes); // slot 0 arrives already populated
  attribute_buffers_t buffers;
  attribute_buffers_initialize(formats, buffers, point_count, std::move(morton));

  REQUIRE(buffers.buffers.size() == formats.size());
  CHECK(static_cast<const uint8_t *>(buffers.buffers[0].data)[0] == 0x5A); // adopted, not cleared
  for (size_t i = 1; i < buffers.buffers.size(); i++)
  {
    const auto &blob = buffers.buffers[i];
    uint64_t non_zero = 0;
    for (uint32_t byte = 0; byte < blob.size; byte++)
      non_zero += static_cast<const uint8_t *>(blob.data)[byte] != 0 ? 1 : 0;
    REQUIRE(non_zero == 0);
  }
}

// ---------------------------------------------------------------------------------------------
// 5. End to end: a dataset whose inputs disagree about their attributes.

// Everything above tests a seam in isolation. This converts a real dataset from two inputs with
// DIFFERENT attribute sets and reads it back through the public query API, which is the shape an
// amend produces: some units carry the new attribute, some do not, and the coarse levels above them
// have to merge the two.
//
// It exercises both merge paths, because both go through the same union mapping and the same buffer
// allocator: leaf COLLAPSE (tree_collapse.cpp merges the subsets of a leaf into one unit) and LOD
// generation (tree_lod_generator.cpp subsamples children into a coarser node).
//
// The assertion is deliberately about VALUES rather than counts: every colour that comes back must
// be either the one the coloured input wrote or all-zero. A third value can only be uninitialized
// heap, which is exactly what this used to return.

#include <dew/access/query.h>
#include <dew/converter/converter.h>

#include <cstdio>

namespace
{

constexpr uint32_t k_amend_grid = 20; // 20^3 = 8000 points per input
constexpr uint32_t k_amend_points = k_amend_grid * k_amend_grid * k_amend_grid;
constexpr const char *k_amend_path = "attribute_amend_mixed.dew";
constexpr const char *k_coloured_name = "coloured";
constexpr const char *k_plain_name = "plain";

// A distinctive triple, every component non-zero so that "all zero" and "the real colour" cannot be
// confused, and neither can be mistaken for a plausible piece of garbage.
constexpr uint8_t k_rgb[3] = {200, 100, 50};

bool is_coloured_input(const char *filename, size_t filename_size)
{
  return filename_size == strlen(k_coloured_name) && memcmp(filename, k_coloured_name, filename_size) == 0;
}

struct emit_state_t
{
  uint32_t emitted = 0;
  bool coloured = false;
};

dew_converter_file_pre_init_info_t amend_pre_init(const char *, size_t, dew_error_t **)
{
  dew_converter_file_pre_init_info_t info{};
  info.approximate_point_count = k_amend_points;
  info.found_point_count = 1;
  info.approximate_point_size_bytes = 16;
  info.scale[0] = info.scale[1] = info.scale[2] = 1.0;
  info.found_scale = 1;
  return info;
}

void amend_init(const char *filename, size_t filename_size, dew_converter_header_t *header, dew_attributes_t *attributes, void **user_ptr, dew_error_t **)
{
  const bool coloured = is_coloured_input(filename, filename_size);
  header->point_count = k_amend_points;
  for (int i = 0; i < 3; i++)
  {
    header->offset[i] = 0.0;
    header->scale[i] = 1.0;
    header->min[i] = 0.0;
    header->max[i] = double(k_amend_grid) * 2.0;
  }
  dew_attributes_add_attribute(attributes, DEW_ATTRIBUTE_XYZ, uint32_t(strlen(DEW_ATTRIBUTE_XYZ)), dew_type_i32, dew_components_3);
  // The whole point: one input declares rgb, the other does not.
  if (coloured)
    dew_attributes_add_attribute(attributes, DEW_ATTRIBUTE_RGB, uint32_t(strlen(DEW_ATTRIBUTE_RGB)), dew_type_u8, dew_components_3);
  auto *state = new emit_state_t();
  state->coloured = coloured;
  *user_ptr = state;
}

void amend_convert_data(void *user_ptr, const dew_converter_header_t *, const dew_attribute_t *, uint32_t, uint32_t max_points, dew_blob_t *buffers, uint32_t buffer_count, uint32_t *points_read,
                        uint8_t *done, dew_error_t **)
{
  auto *state = static_cast<emit_state_t *>(user_ptr);
  const uint32_t remaining = k_amend_points - state->emitted;
  const uint32_t n = remaining < max_points ? remaining : max_points;

  auto *xyz = static_cast<int32_t *>(buffers[0].data);
  for (uint32_t i = 0; i < n; i++)
  {
    const uint32_t p = state->emitted + i;
    // The two inputs INTERLEAVE: coloured points sit on even x, plain points on odd x. Their morton
    // codes therefore alternate, so every node -- collapsed leaf and LOD alike -- ends up with
    // children of both kinds. Two spatially separate blocks would let the tree keep them apart and
    // the merge under test would never happen.
    xyz[i * 3 + 0] = int32_t((p % k_amend_grid) * 2 + (state->coloured ? 0 : 1));
    xyz[i * 3 + 1] = int32_t((p / k_amend_grid) % k_amend_grid);
    xyz[i * 3 + 2] = int32_t(p / (k_amend_grid * k_amend_grid));
  }
  if (state->coloured && buffer_count >= 2)
  {
    auto *rgb = static_cast<uint8_t *>(buffers[1].data);
    for (uint32_t i = 0; i < n; i++)
    {
      rgb[i * 3 + 0] = k_rgb[0];
      rgb[i * 3 + 1] = k_rgb[1];
      rgb[i * 3 + 2] = k_rgb[2];
    }
  }
  state->emitted += n;
  *points_read = n;
  *done = state->emitted >= k_amend_points ? 1 : 0;
}

void amend_destroy_user_ptr(void *user_ptr)
{
  delete static_cast<emit_state_t *>(user_ptr);
}

bool build_mixed_dataset_tuned(const char *path, uint32_t node_point_limit, uint64_t read_chunk_bytes)
{
  std::remove(path);
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_truncate, &error);
  if (!converter)
  {
    if (error)
      dew_error_destroy(error);
    return false;
  }
  dew_converter_file_convert_callbacks_t callbacks{};
  callbacks.pre_init = amend_pre_init;
  callbacks.init = amend_init;
  callbacks.convert_data = amend_convert_data;
  callbacks.destroy_user_ptr = amend_destroy_user_ptr;
  dew_converter_set_file_converter_callbacks(converter, callbacks);
  // Small nodes, so the tree really subdivides and there are interior LOD nodes to merge into.
  dew_converter_set_node_point_limit(converter, node_point_limit);
  if (read_chunk_bytes)
    dew_converter_set_read_chunk_bytes(converter, read_chunk_bytes);

  dew_converter_str_buffer names[2] = {{k_coloured_name, uint32_t(strlen(k_coloured_name))}, {k_plain_name, uint32_t(strlen(k_plain_name))}};
  dew_converter_add_data_file(converter, names, 2);
  dew_converter_wait_idle(converter);
  const bool ok = dew_converter_status(converter) != dew_conversion_status_error;
  dew_converter_destroy(converter);
  return ok;
}

bool build_mixed_dataset(const char *path)
{
  return build_mixed_dataset_tuned(path, 700, 0);
}

// Query the whole extent and check every returned colour, at one LOD mode.
void require_colours_are_clean(const char *path, dew_lod_mode_t lod_mode, int32_t lod, const std::string &what)
{
  dew_error_t *error = nullptr;
  auto *dataset = dew_dataset_create(path, uint32_t(strlen(path)), nullptr, 0, nullptr, nullptr, &error);
  REQUIRE(dataset != nullptr);
  REQUIRE(dew_dataset_wait_ready(dataset, -1) == dew_dataset_ready);

  const char *attributes[] = {DEW_ATTRIBUTE_RGB};
  dew_region_request_t spec{};
  for (int i = 0; i < 3; i++)
  {
    spec.aabb_min[i] = -1.0;
    spec.aabb_max[i] = double(k_amend_grid) * 2.0 + 1.0;
  }
  spec.lod_mode = lod_mode;
  spec.lod = lod; // only consulted for dew_lod_level
  spec.clip_mode = dew_clip_node;
  spec.attribute_names = attributes;
  spec.attribute_count = 1;
  spec.position_format = dew_position_r64_absolute;

  auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
  REQUIRE(request != nullptr);
  REQUIRE(dew_request_wait(request, -1) == dew_request_completed);

  dew_request_result_t result{};
  REQUIRE(dew_request_get_result(request, &result) == 1);
  REQUIRE(result.point_count > 0);

  // Find the rgb buffer by name rather than by position.
  const dew_attribute_buffer_t *rgb_buffer = nullptr;
  for (uint32_t i = 0; i < result.buffer_count; i++)
    if (result.buffers[i].name && strcmp(result.buffers[i].name, DEW_ATTRIBUTE_RGB) == 0)
      rgb_buffer = &result.buffers[i];
  REQUIRE(rgb_buffer != nullptr);
  REQUIRE(rgb_buffer->components == dew_components_3);
  REQUIRE(rgb_buffer->size_bytes == result.point_count * 3);

  const auto *rgb = static_cast<const uint8_t *>(rgb_buffer->data);
  uint64_t coloured = 0, blank = 0, garbage = 0;
  for (uint64_t p = 0; p < result.point_count; p++)
  {
    const uint8_t r = rgb[p * 3 + 0], g = rgb[p * 3 + 1], b = rgb[p * 3 + 2];
    if (r == k_rgb[0] && g == k_rgb[1] && b == k_rgb[2])
      coloured++;
    else if (r == 0 && g == 0 && b == 0)
      blank++;
    else
      garbage++;
  }
  MESSAGE(what, ": points ", result.point_count, "  nodes ", result.node_count, "  coloured ", coloured, "  blank ", blank, "  GARBAGE ", garbage);

  // The only two legitimate outcomes: the colour the coloured input wrote, or zero for a point whose
  // input never had the attribute. Anything else is uninitialized memory written into the dataset.
  REQUIRE(garbage == 0);
  // ...and the test would be vacuous if the merge under test never actually happened.
  REQUIRE(coloured > 0);
  REQUIRE(blank > 0);

  dew_request_release(request);
  dew_dataset_close(dataset);
}

} // namespace

TEST_CASE("amend: merging inputs that disagree about rgb never yields uninitialized colour")
{
  REQUIRE(build_mixed_dataset(k_amend_path));

  // dew_lod_full walks down to the leaves, which are COLLAPSED units -- the merge tree_collapse does.
  require_colours_are_clean(k_amend_path, dew_lod_full, 0, "full resolution (collapsed leaves)");
  // Coarser frontiers stop on interior nodes, which are LOD units -- the merge tree_lod_generator
  // does. Several levels, because the pyramid is built bottom-up: a level is assembled from the one
  // below it, so a mistake could be confined to one of them.
  for (int32_t level = 1; level <= 3; level++)
    require_colours_are_clean(k_amend_path, dew_lod_level, level, "lod level " + std::to_string(level) + " (LOD nodes)");

  std::remove(k_amend_path);
}

// ---------------------------------------------------------------------------------------------
// 6. Multi-pass conversion: the path add_storage's replacement branch actually serves.

// A conversion whose done-morton watermark advances more than once runs several LOD passes, and a
// node LOD-ed in an earlier pass is regenerated in a later one UNDER ITS EXISTING ID -- i.e. through
// add_storage's replacement branch. That branch used to be forbidden by an assert while its body was
// written to handle exactly that case, so in a debug build this path could not run at all and in a
// release build it double-counted the reference.
//
// Small read chunks force the shape: each chunk advances the watermark, and each advance fires a
// pass (processor.cpp, handle_index_write_done -> generate_lod).
//
// The assertion is that the dataset is still CORRECT afterwards -- every source point present
// exactly once. A double-counted reference does not corrupt the points directly; it strands units
// so their blobs are never freed, which shows up as a dataset that keeps growing and, once the
// replacement branch is live, as an assert firing mid-conversion.

TEST_CASE("amend: a multi-pass conversion regenerates LOD nodes without losing points")
{
  constexpr const char *path = "attribute_amend_multipass.dew";
  REQUIRE(build_mixed_dataset_tuned(path, /*node_point_limit=*/400, /*read_chunk_bytes=*/16 * 1024));

  dew_error_t *error = nullptr;
  auto *dataset = dew_dataset_create(path, uint32_t(strlen(path)), nullptr, 0, nullptr, nullptr, &error);
  REQUIRE(dataset != nullptr);
  REQUIRE(dew_dataset_wait_ready(dataset, -1) == dew_dataset_ready);

  const char *attributes[] = {DEW_ATTRIBUTE_RGB};
  dew_region_request_t spec{};
  for (int i = 0; i < 3; i++)
  {
    spec.aabb_min[i] = -1.0;
    spec.aabb_max[i] = double(k_amend_grid) * 2.0 + 1.0;
  }
  spec.lod_mode = dew_lod_full;
  spec.clip_mode = dew_clip_node;
  spec.attribute_names = attributes;
  spec.attribute_count = 1;
  spec.position_format = dew_position_r64_absolute;

  auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
  REQUIRE(request != nullptr);
  REQUIRE(dew_request_wait(request, -1) == dew_request_completed);

  dew_request_result_t result{};
  REQUIRE(dew_request_get_result(request, &result) == 1);

  // Every source point, exactly once: a full-resolution query returns the converted count. LOD nodes
  // are subsampled COPIES, so a walk that emitted an extra level would overshoot -- which is how a
  // regenerated node that was left double-referenced would most likely show up.
  REQUIRE(result.point_count == 2 * k_amend_points);

  const dew_attribute_buffer_t *rgb_buffer = nullptr;
  for (uint32_t i = 0; i < result.buffer_count; i++)
    if (result.buffers[i].name && strcmp(result.buffers[i].name, DEW_ATTRIBUTE_RGB) == 0)
      rgb_buffer = &result.buffers[i];
  REQUIRE(rgb_buffer != nullptr);

  const auto *rgb = static_cast<const uint8_t *>(rgb_buffer->data);
  uint64_t coloured = 0, blank = 0, garbage = 0;
  for (uint64_t p = 0; p < result.point_count; p++)
  {
    const uint8_t r = rgb[p * 3 + 0], g = rgb[p * 3 + 1], b = rgb[p * 3 + 2];
    if (r == k_rgb[0] && g == k_rgb[1] && b == k_rgb[2])
      coloured++;
    else if (r == 0 && g == 0 && b == 0)
      blank++;
    else
      garbage++;
  }
  MESSAGE("multi-pass: points ", result.point_count, "  nodes ", result.node_count, "  coloured ", coloured, "  blank ", blank, "  GARBAGE ", garbage);
  REQUIRE(garbage == 0);
  REQUIRE(coloured == k_amend_points);
  REQUIRE(blank == k_amend_points);

  dew_request_release(request);
  dew_dataset_close(dataset);
  std::remove(path);
}

// ---------------------------------------------------------------------------------------------
// 7. Growing a dataset one input at a time.

// The other half of the iterative workflow, and the half that already works: adding POINTS to a
// dataset that has already been converted and closed. This is what a scan-by-scan ingest needs --
// convert one file, look at the result, convert the next into the same dataset -- and it is worth a
// test because the whole plan for adding ATTRIBUTES later assumes this part is solid.
//
// Two properties, and the second one is the one that makes the workflow safe to re-run:
//   * reopening with dew_open_file_semantics_open_existing and adding a new input grows the dataset;
//   * re-adding an input that already landed is SKIPPED rather than duplicated, so a job that is
//     interrupted and restarted converges instead of counting points twice.

namespace
{

uint64_t query_point_count(const char *path)
{
  dew_error_t *error = nullptr;
  auto *dataset = dew_dataset_create(path, uint32_t(strlen(path)), nullptr, 0, nullptr, nullptr, &error);
  REQUIRE(dataset != nullptr);
  REQUIRE(dew_dataset_wait_ready(dataset, -1) == dew_dataset_ready);

  dew_region_request_t spec{};
  for (int i = 0; i < 3; i++)
  {
    spec.aabb_min[i] = -1.0;
    spec.aabb_max[i] = double(k_amend_grid) * 2.0 + 1.0;
  }
  spec.lod_mode = dew_lod_full;
  spec.clip_mode = dew_clip_node;
  spec.position_format = dew_position_r64_absolute;

  auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
  REQUIRE(request != nullptr);
  REQUIRE(dew_request_wait(request, -1) == dew_request_completed);
  dew_request_result_t result{};
  REQUIRE(dew_request_get_result(request, &result) == 1);
  const uint64_t count = result.point_count;
  dew_request_release(request);
  dew_dataset_close(dataset);
  return count;
}

// Add one named input to an existing dataset (or create it), and drain.
bool add_one_input(const char *path, const char *input_name, dew_converter_open_file_semantics_t semantics)
{
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), semantics, &error);
  if (!converter)
  {
    if (error)
      dew_error_destroy(error);
    return false;
  }
  dew_converter_file_convert_callbacks_t callbacks{};
  callbacks.pre_init = amend_pre_init;
  callbacks.init = amend_init;
  callbacks.convert_data = amend_convert_data;
  callbacks.destroy_user_ptr = amend_destroy_user_ptr;
  dew_converter_set_file_converter_callbacks(converter, callbacks);
  // ONLY on a fresh dataset. The tree_initialization setters (node point limit, tree scale, read
  // chunk bytes) assert !_configuration_initialized, and a reopened dataset has already restored and
  // sealed its configuration -- so calling one here aborts the process in a debug build. The stored
  // configuration is authoritative on reopen, which is the only coherent semantic (a dataset cannot
  // change its node point limit after it has been built), but the API does not say so and the guard
  // is a bare assert rather than a refusal.
  if (semantics == dew_open_file_semantics_truncate)
    dew_converter_set_node_point_limit(converter, 700);

  dew_converter_str_buffer name{input_name, uint32_t(strlen(input_name))};
  dew_converter_add_data_file(converter, &name, 1);
  dew_converter_wait_idle(converter);
  const bool ok = dew_converter_status(converter) != dew_conversion_status_error;
  dew_converter_destroy(converter);
  return ok;
}

} // namespace

TEST_CASE("amend: reopening a finished dataset re-adds its inputs without duplicating them")
{
  // WHAT REOPENING ACTUALLY BUYS, pinned because it is easy to assume more.
  //
  // dew_open_file_semantics_open_existing is a RESUME, not an EXTEND. Re-adding an input that
  // already landed is correctly skipped -- register_file matches it by name and reports it done, so
  // a job that is interrupted can simply be re-run over its whole input list and will converge.
  //
  // Adding a NEW input to a dataset whose trees are already final is a different matter and does NOT
  // work today: the points reach tree_build.cpp, which asserts
  //     tree_state[...] == tree_state_t::building && "point insert into finalized tree"
  // and aborts. That is the SAME immutability that stops an attribute being added after the fact, so
  // an iterative ingest ("convert one scan, look, convert the next into the same dataset") needs the
  // same unlock as an amend does. Not asserted here, because a deliberate SIGABRT is not something a
  // test suite can catch.
  constexpr const char *path = "attribute_amend_incremental.dew";
  std::remove(path);

  REQUIRE(add_one_input(path, k_coloured_name, dew_open_file_semantics_truncate));
  const uint64_t after_first = query_point_count(path);
  MESSAGE("after the first conversion: ", after_first, " points");
  REQUIRE(after_first == k_amend_points);

  // Re-adding the SAME input to the finished dataset: skipped, not converted a second time.
  REQUIRE(add_one_input(path, k_coloured_name, dew_open_file_semantics_open_existing));
  const uint64_t after_repeat = query_point_count(path);
  MESSAGE("after re-adding the same input: ", after_repeat, " points");
  REQUIRE(after_repeat == k_amend_points);

  std::remove(path);
}

// ---------------------------------------------------------------------------------------------
// 8. How the inputs may be handed to ONE converter session.

// Finalization tracks the done-morton watermark, not the end of the run: a tree is sealed as soon as
// the watermark proves no more points can land in its range (tree_handler.cpp, gated on
// leaves_collapsed). So WHEN a file is added matters spatially, not just temporally -- an input
// whose points fall in a range the watermark has already swept past has nowhere to go.
//
// That is a real constraint on an ingest loop over thousands of scans, so it is measured here rather
// than reasoned about. Both inputs interleave (even/odd x), which is the worst case: every tree
// covers both, so a late arrival always targets ground the early one already claimed.

namespace
{

// Add each name in its own dew_converter_add_data_file call, optionally draining between them.
bool build_in_batches(const char *path, const std::vector<const char *> &names, bool drain_between)
{
  std::remove(path);
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_truncate, &error);
  if (!converter)
  {
    if (error)
      dew_error_destroy(error);
    return false;
  }
  dew_converter_file_convert_callbacks_t callbacks{};
  callbacks.pre_init = amend_pre_init;
  callbacks.init = amend_init;
  callbacks.convert_data = amend_convert_data;
  callbacks.destroy_user_ptr = amend_destroy_user_ptr;
  dew_converter_set_file_converter_callbacks(converter, callbacks);
  dew_converter_set_node_point_limit(converter, 700);

  for (const char *name : names)
  {
    dew_converter_str_buffer buf{name, uint32_t(strlen(name))};
    dew_converter_add_data_file(converter, &buf, 1);
    if (drain_between)
      dew_converter_wait_idle(converter);
  }
  dew_converter_wait_idle(converter);
  const bool ok = dew_converter_status(converter) != dew_conversion_status_error;
  dew_converter_destroy(converter);
  return ok;
}

} // namespace

TEST_CASE("amend: several add_data_file calls in one session are safe without draining")
{
  // The pattern an ingest loop over 3817 scans would naturally use: batch them, hand each batch over
  // as it is discovered, never drain until the end. The files stay in flight together, so the
  // watermark cannot run ahead of an input that has been registered but not yet read.
  constexpr const char *path = "attribute_amend_batches.dew";
  REQUIRE(build_in_batches(path, {k_coloured_name, k_plain_name}, /*drain_between=*/false));

  const uint64_t points = query_point_count(path);
  MESSAGE("two separate add_data_file calls, no drain between: ", points, " points");
  REQUIRE(points == 2 * k_amend_points);

  // The NEGATIVE half, measured but not asserted here because it aborts the process rather than
  // failing: the same two calls with a dew_converter_wait_idle between them dies on
  // tree_build.cpp's "point insert into finalized tree". Draining runs the terminal pass, which
  // seals every tree; the second input then has nowhere to go. So the rule for an ingest loop is
  // "add as many batches as you like, drain exactly once at the end".

  std::remove(path);
}

// ---------------------------------------------------------------------------------------------
// 9. Mutable datasets: ingest that spans sessions.

// A tree is normally sealed as soon as the done-morton watermark proves nothing more can land in
// it, and sealing is what makes it uploadable and then evictable -- which is also what makes it
// impossible to add to. Test 8 above measures the consequence: reopen a finished dataset, add a new
// input, and tree_build aborts on "point insert into finalized tree".
//
// dew_converter_set_mutable holds that gate open. Trees stay `building`, no band is emitted, and
// nothing is uploaded until dew_converter_finalize. Collapse and LOD still run and no reader
// consults tree_state, so a mutable dataset reads and renders like any other.

namespace
{

// One input into a dataset, in mutable mode, in its own converter session.
bool add_one_input_mutable(const char *path, const char *input_name, dew_converter_open_file_semantics_t semantics)
{
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), semantics, &error);
  if (!converter)
  {
    if (error)
      dew_error_destroy(error);
    return false;
  }
  dew_converter_set_mutable(converter, 1);
  dew_converter_file_convert_callbacks_t callbacks{};
  callbacks.pre_init = amend_pre_init;
  callbacks.init = amend_init;
  callbacks.convert_data = amend_convert_data;
  callbacks.destroy_user_ptr = amend_destroy_user_ptr;
  dew_converter_set_file_converter_callbacks(converter, callbacks);
  // Only on a fresh dataset -- the tree-initialization setters assert once the configuration is
  // sealed, and a reopened dataset has already restored one.
  if (semantics == dew_open_file_semantics_truncate)
    dew_converter_set_node_point_limit(converter, 700);

  dew_converter_str_buffer name{input_name, uint32_t(strlen(input_name))};
  dew_converter_add_data_file(converter, &name, 1);
  dew_converter_wait_idle(converter);
  const bool ok = dew_converter_status(converter) != dew_conversion_status_error;
  dew_converter_destroy(converter);
  return ok;
}

} // namespace

TEST_CASE("mutable: the mode survives closing and reopening the dataset")
{
  constexpr const char *path = "attribute_amend_mutable_flag.dew";
  std::remove(path);

  REQUIRE(add_one_input_mutable(path, k_coloured_name, dew_open_file_semantics_truncate));

  // The flag lives in tree_config_t, which rides in the registry blob -- so reopening restores it
  // without the caller having to say so again. (It may say so again; that is idempotent.)
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
  REQUIRE(converter != nullptr);
  const uint8_t restored = dew_converter_is_mutable(converter);
  MESSAGE("mutable flag after reopen: ", int(restored));
  CHECK(restored == 1);
  dew_converter_destroy(converter);

  std::remove(path);
}

TEST_CASE("mutable: a second session can add a new input to an existing dataset")
{
  // The sequence that aborts without mutable mode (test 8 records that abort). Three pieces have to
  // be in place, and each failed differently while they were not:
  //   * the deferred seal -- otherwise the second insert aborts on "point insert into finalized tree";
  //   * every tree resident on open -- otherwise the insert path segfaults on a lazily-loaded null;
  //   * the LOD watermarks reset on reopen -- otherwise no pass fires, so no checkpoint is written,
  //     and the second session's points reach the tree and are discarded (it returned 8000, not 16000).
  // THE POINT OF THE WHOLE MODE, and the exact sequence that aborts without it (test 8 records the
  // abort). Two inputs, two separate converter sessions, the dataset closed in between.
  constexpr const char *path = "attribute_amend_mutable_grow.dew";
  std::remove(path);

  REQUIRE(add_one_input_mutable(path, k_coloured_name, dew_open_file_semantics_truncate));
  const uint64_t after_first = query_point_count(path);
  MESSAGE("session 1: ", after_first, " points");
  REQUIRE(after_first == k_amend_points);

  // Reopened, and the trees are still building -- so this insert lands instead of aborting.
  REQUIRE(add_one_input_mutable(path, k_plain_name, dew_open_file_semantics_open_existing));
  const uint64_t after_second = query_point_count(path);
  MESSAGE("session 2: ", after_second, " points");
  REQUIRE(after_second == 2 * k_amend_points);

  // Re-adding an input that already landed is still skipped, so a restarted ingest converges.
  REQUIRE(add_one_input_mutable(path, k_coloured_name, dew_open_file_semantics_open_existing));
  REQUIRE(query_point_count(path) == 2 * k_amend_points);

  // And the second session reached the LOD PYRAMID, not merely full resolution -- which is the
  // whole reason the watermark reset exists. A coarse frontier has to contain points from both
  // sessions: the first input is coloured, the second is not, so seeing BOTH at a coarse level is
  // proof the re-LOD covered the newly added input rather than just the one already there.
  require_colours_are_clean(path, dew_lod_level, 3, "two mutable sessions, lod level 3");

  std::remove(path);
}

// ---------------------------------------------------------------------------------------------
// 10. Two sessions must produce the same dataset as one.

namespace
{

struct colour_census_t
{
  uint64_t points = 0;
  uint32_t nodes = 0;
  uint64_t coloured = 0;
  uint64_t blank = 0;
  uint64_t garbage = 0;
};

colour_census_t census_at(const char *path, dew_lod_mode_t lod_mode, int32_t lod)
{
  colour_census_t out;
  dew_error_t *error = nullptr;
  auto *dataset = dew_dataset_create(path, uint32_t(strlen(path)), nullptr, 0, nullptr, nullptr, &error);
  REQUIRE(dataset != nullptr);
  REQUIRE(dew_dataset_wait_ready(dataset, -1) == dew_dataset_ready);

  const char *attributes[] = {DEW_ATTRIBUTE_RGB};
  dew_region_request_t spec{};
  for (int i = 0; i < 3; i++)
  {
    spec.aabb_min[i] = -1.0;
    spec.aabb_max[i] = double(k_amend_grid) * 2.0 + 1.0;
  }
  spec.lod_mode = lod_mode;
  spec.lod = lod;
  spec.clip_mode = dew_clip_node;
  spec.attribute_names = attributes;
  spec.attribute_count = 1;
  spec.position_format = dew_position_r64_absolute;

  auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
  REQUIRE(request != nullptr);
  REQUIRE(dew_request_wait(request, -1) == dew_request_completed);
  dew_request_result_t result{};
  REQUIRE(dew_request_get_result(request, &result) == 1);

  out.points = result.point_count;
  out.nodes = result.node_count;
  const dew_attribute_buffer_t *rgb_buffer = nullptr;
  for (uint32_t i = 0; i < result.buffer_count; i++)
    if (result.buffers[i].name && strcmp(result.buffers[i].name, DEW_ATTRIBUTE_RGB) == 0)
      rgb_buffer = &result.buffers[i];
  REQUIRE(rgb_buffer != nullptr);
  const auto *rgb = static_cast<const uint8_t *>(rgb_buffer->data);
  for (uint64_t p = 0; p < result.point_count; p++)
  {
    const uint8_t r = rgb[p * 3 + 0], g = rgb[p * 3 + 1], b = rgb[p * 3 + 2];
    if (r == k_rgb[0] && g == k_rgb[1] && b == k_rgb[2])
      out.coloured++;
    else if (r == 0 && g == 0 && b == 0)
      out.blank++;
    else
      out.garbage++;
  }
  dew_request_release(request);
  dew_dataset_close(dataset);
  return out;
}

} // namespace

TEST_CASE("mutable: two sessions produce the same dataset as one")
{
  // The property that makes a mutable ingest trustworthy: HOW the input arrived must not change
  // WHAT is stored. The same two inputs, once in a single conversion and once split across two
  // sessions with the dataset closed in between, must agree -- at full resolution and at every
  // coarse frontier.
  //
  // This is what caught the collapse bug. sub_tree_insert_points only cleared leaves_collapsed on
  // the tree it was handed, not on the sub-trees the routing actually reached, and on a reopened
  // dataset those all come back true from tree_compute_leaves_collapsed. Collapse then skipped
  // them, leaving each touched leaf holding two subsets instead of one merged unit -- 80 leaf
  // subsets against one session's 40. Since find_indices_to_quantize runs per subset, that also
  // moved the LOD representative picks: level 3 came out 832/2624 coloured/blank instead of
  // 1896/1560, from the same 16000 points.
  constexpr const char *one_path = "attribute_amend_equiv_one.dew";
  constexpr const char *two_path = "attribute_amend_equiv_two.dew";
  std::remove(one_path);
  std::remove(two_path);

  REQUIRE(build_mixed_dataset_tuned(one_path, 700, 0));
  REQUIRE(add_one_input_mutable(two_path, k_coloured_name, dew_open_file_semantics_truncate));
  REQUIRE(add_one_input_mutable(two_path, k_plain_name, dew_open_file_semantics_open_existing));

  for (int32_t lod = 0; lod <= 4; lod++)
  {
    const auto a = census_at(one_path, lod == 0 ? dew_lod_full : dew_lod_level, lod);
    const auto b = census_at(two_path, lod == 0 ? dew_lod_full : dew_lod_level, lod);
    MESSAGE("lod ", lod, ": one=[pts ", a.points, " nodes ", a.nodes, " col ", a.coloured, " blank ", a.blank, "]  two=[pts ", b.points, " nodes ", b.nodes, " col ", b.coloured, " blank ", b.blank, "]");
    CHECK(a.points == b.points);
    CHECK(a.nodes == b.nodes);
    CHECK(a.coloured == b.coloured);
    CHECK(a.blank == b.blank);
    CHECK(a.garbage == 0);
    CHECK(b.garbage == 0);
  }

  std::remove(one_path);
  std::remove(two_path);
}

// ---------------------------------------------------------------------------------------------
// 11. Ending mutable mode.

namespace
{

// Reopen a mutable dataset purely to finalize it -- no new input.
bool finalize_dataset(const char *path)
{
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
  if (!converter)
  {
    if (error)
      dew_error_destroy(error);
    return false;
  }
  dew_converter_finalize(converter);
  const bool ok = dew_converter_status(converter) != dew_conversion_status_error;
  dew_converter_destroy(converter);
  return ok;
}

bool reopened_is_mutable(const char *path)
{
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
  REQUIRE(converter != nullptr);
  const bool m = dew_converter_is_mutable(converter) != 0;
  dew_converter_destroy(converter);
  return m;
}

} // namespace

TEST_CASE("mutable: finalize seals the dataset and the seal survives a reopen")
{
  constexpr const char *path = "attribute_amend_finalize.dew";
  std::remove(path);

  REQUIRE(add_one_input_mutable(path, k_coloured_name, dew_open_file_semantics_truncate));
  REQUIRE(add_one_input_mutable(path, k_plain_name, dew_open_file_semantics_open_existing));
  REQUIRE(reopened_is_mutable(path));

  REQUIRE(finalize_dataset(path));

  // The flag is cleared AND persisted -- a dataset that reported itself sealed but came back mutable
  // would silently keep deferring uploads forever.
  CHECK(!reopened_is_mutable(path));

  // And finalizing changed nothing about the data.
  MESSAGE("after finalize: ", query_point_count(path), " points");
  CHECK(query_point_count(path) == 2 * k_amend_points);

  std::remove(path);
}

TEST_CASE("mutable: a finalized dataset matches one converted in a single pass")
{
  // The end-to-end promise of the whole mode: build it however you like -- one session or several,
  // mutable throughout -- and after finalize it is indistinguishable from an ordinary conversion.
  constexpr const char *one_path = "attribute_amend_fin_one.dew";
  constexpr const char *many_path = "attribute_amend_fin_many.dew";
  std::remove(one_path);
  std::remove(many_path);

  REQUIRE(build_mixed_dataset_tuned(one_path, 700, 0));
  REQUIRE(add_one_input_mutable(many_path, k_coloured_name, dew_open_file_semantics_truncate));
  REQUIRE(add_one_input_mutable(many_path, k_plain_name, dew_open_file_semantics_open_existing));
  REQUIRE(finalize_dataset(many_path));

  for (int32_t lod = 0; lod <= 4; lod++)
  {
    const auto a = census_at(one_path, lod == 0 ? dew_lod_full : dew_lod_level, lod);
    const auto b = census_at(many_path, lod == 0 ? dew_lod_full : dew_lod_level, lod);
    MESSAGE("lod ", lod, ": single=[pts ", a.points, " nodes ", a.nodes, " col ", a.coloured, " blank ", a.blank, "]  finalized=[pts ", b.points, " nodes ", b.nodes, " col ", b.coloured,
            " blank ", b.blank, "]");
    CHECK(a.points == b.points);
    CHECK(a.nodes == b.nodes);
    CHECK(a.coloured == b.coloured);
    CHECK(a.blank == b.blank);
    CHECK(a.garbage == 0);
    CHECK(b.garbage == 0);
  }

  std::remove(one_path);
  std::remove(many_path);
}

// ---------------------------------------------------------------------------------------------
// 12. The write primitive an amend needs: a unit gaining a slot.

TEST_CASE("amend: append_storage keeps the existing blobs and adds one")
{
  // What add_storage CANNOT do. It replaces the whole location vector and pushes whatever was there
  // onto the discard list -- right for a regenerated LOD node, fatal for an amend, where the blobs
  // being discarded would be the unit's positions and every attribute it already carried.
  input_storage_map_t map;
  const input_data_id_t id{11, 0};
  const auto positions = make_location(0, 1000, 64);
  const auto intensity = make_location(0, 2000, 32);
  const auto rgb = make_location(0, 3000, 48);

  map.add_storage(id, attributes_id_t{1}, {positions, intensity});
  REQUIRE(map.take_discarded().empty());

  map.append_storage(id, attributes_id_t{2}, {rgb});

  // Everything that was there, in the same slots, plus the new one at the end. Order is the whole
  // contract: readers resolve a name against the config and index this vector with the result, so a
  // reordered vector mis-maps every attribute of the unit.
  const auto stored = map.info(id).second;
  REQUIRE(stored.size() == 3);
  CHECK(stored[0].offset == positions.offset);
  CHECK(stored[1].offset == intensity.offset);
  CHECK(stored[2].offset == rgb.offset);
  CHECK(map.attribute_id(id).data == 2);

  // And critically: NOTHING was discarded. add_storage in the same position would have thrown the
  // positions and intensity blobs onto the freed list, and the next checkpoint would have reused
  // that space while the unit still pointed at it.
  CHECK(map.take_discarded().empty());

  // Still one reference: appending a slot re-describes a unit, it does not add a reference.
  CHECK(map.ref_count(id) == 1);
  map.dereference(id);
  CHECK(!map.contains(id));
}

TEST_CASE("amend: an appended slot survives the round trip beside units that never gained one")
{
  // The half-amended state, on disk. A partially attributed dataset is legal -- readers resolve per
  // node and zero-fill what a node lacks -- so an amend must be interruptible and resumable, which
  // means the mixed widths have to persist.
  input_storage_map_t map;
  const input_data_id_t amended{1, 0};
  const input_data_id_t untouched{2, 0};

  map.add_storage(amended, attributes_id_t{5}, {make_location(0, 100, 10), make_location(0, 200, 20)});
  map.add_storage(untouched, attributes_id_t{5}, {make_location(0, 300, 30), make_location(0, 400, 40)});
  map.append_storage(amended, attributes_id_t{6}, {make_location(1, 500, 50)});

  std::vector<uint8_t> buffer(map.serialized_size());
  const auto written = map.serialize(buffer.data(), buffer.data() + buffer.size());
  REQUIRE(written.first);

  input_storage_map_t restored;
  REQUIRE(restored.deserialize(buffer.data(), buffer.data() + buffer.size()).first);

  REQUIRE(restored.info(amended).second.size() == 3);
  REQUIRE(restored.info(untouched).second.size() == 2);
  CHECK(restored.attribute_id(amended).data == 6);
  CHECK(restored.attribute_id(untouched).data == 5);
  CHECK(restored.location(amended, 2).offset == 500);
  MESSAGE("after round trip: amended has ", restored.info(amended).second.size(), " slots, untouched has ", restored.info(untouched).second.size());
}

// ---------------------------------------------------------------------------------------------
// 13. Destination mode: nothing ships while mutable, everything ships on finalize.

// Hermetically skipped unless DEW_TEST_S3 is set, same convention as cloud_io_live_test.cpp. Against
// a local minio:
//   docker run -d --name dew-minio -p 9000:9000 -e MINIO_ROOT_USER=minioadmin \
//     -e MINIO_ROOT_PASSWORD=minioadmin minio/minio server /data
//   docker run --rm --network host --entrypoint sh minio/mc -c \
//     "mc alias set local http://127.0.0.1:9000 minioadmin minioadmin && mc mb --ignore-existing local/pointstest"
//   DEW_TEST_S3=1 AWS_ACCESS_KEY_ID=minioadmin AWS_SECRET_ACCESS_KEY=minioadmin \
//     AWS_ENDPOINT_URL=http://127.0.0.1:9000 AWS_REGION=us-east-1 \
//     ./private_interface_unit_tests -tc="mutable: destination*"
//
// What this covers that no local test can: mutable suppresses emit_band_job, so a mutable dataset in
// destination mode must upload NOTHING -- a tree that has been banded and evicted from the cache is
// exactly the tree an amend can no longer read. finalize re-enables it, and only then does the
// bucket get a complete, readable dataset.

namespace
{

// One input into a destination-mode dataset, mutable unless told otherwise.
bool add_one_input_to_destination(const char *cache_path, const std::string &destination, const char *input_name, dew_converter_open_file_semantics_t semantics, bool make_mutable, bool finalize)
{
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create_with_destination(cache_path, strlen(cache_path), destination.c_str(), destination.size(), nullptr, 0, semantics, &error);
  if (!converter)
  {
    if (error)
      dew_error_destroy(error);
    return false;
  }
  if (make_mutable)
    dew_converter_set_mutable(converter, 1);
  dew_converter_file_convert_callbacks_t callbacks{};
  callbacks.pre_init = amend_pre_init;
  callbacks.init = amend_init;
  callbacks.convert_data = amend_convert_data;
  callbacks.destroy_user_ptr = amend_destroy_user_ptr;
  dew_converter_set_file_converter_callbacks(converter, callbacks);
  if (semantics == dew_open_file_semantics_truncate)
    dew_converter_set_node_point_limit(converter, 700);

  if (input_name)
  {
    dew_converter_str_buffer name{input_name, uint32_t(strlen(input_name))};
    dew_converter_add_data_file(converter, &name, 1);
  }
  if (finalize)
    dew_converter_finalize(converter);
  else
    dew_converter_wait_idle(converter);
  const bool ok = dew_converter_status(converter) != dew_conversion_status_error;
  dew_converter_destroy(converter);
  return ok;
}

} // namespace

TEST_CASE("mutable: destination mode ships nothing until finalize")
{
  if (!std::getenv("DEW_TEST_S3"))
    return; // hermetic skip when no object store is configured

  const std::string bucket = std::getenv("DEW_TEST_S3_BUCKET") ? std::getenv("DEW_TEST_S3_BUCKET") : "pointstest";
  // A FRESH prefix per  run. A bucket dataset is claimed by the uuid its first session writes into the
  // root manifest, and there is no API that unclaims it -- so reusing one prefix means the test
  // passes exactly once and then fails on every rerun with a uuid mismatch, which reads like a
  // regression and is not one. Each run therefore leaves one prefix behind; purge with
  //   mc rm -r --force local/<bucket>/mutable_flow_*
  const auto stamp = uint64_t(std::chrono::steady_clock::now().time_since_epoch().count()) ^ uint64_t(std::chrono::system_clock::now().time_since_epoch().count());
  const std::string destination = "s3://" + bucket + "/mutable_flow_" + std::to_string(stamp);
  MESSAGE("destination: ", destination);
  constexpr const char *cache = "attribute_amend_bucket_cache.dew";
  std::remove(cache);

  // Session 1, mutable: converts locally, uploads nothing.
  REQUIRE(add_one_input_to_destination(cache, destination, k_coloured_name, dew_open_file_semantics_truncate, true, false));
  // The cache is a complete, readable dataset even though the bucket has nothing.
  REQUIRE(query_point_count(cache) == k_amend_points);

  // Session 2, still mutable: grows it.
  REQUIRE(add_one_input_to_destination(cache, destination, k_plain_name, dew_open_file_semantics_open_existing, true, false));
  REQUIRE(query_point_count(cache) == 2 * k_amend_points);

  // Finalize: seal, then ship. No new input.
  REQUIRE(add_one_input_to_destination(cache, destination, nullptr, dew_open_file_semantics_open_existing, false, true));

  // The bucket now holds a dataset that opens and reads on its own -- which is only true if the
  // terminal band landed, i.e. if finalize really did re-enable band emission.
  MESSAGE("reading back from ", destination);
  dew_error_t *error = nullptr;
  auto *dataset = dew_dataset_create(destination.c_str(), uint32_t(destination.size()), nullptr, 0, nullptr, nullptr, &error);
  if (!dataset && error)
  {
    int code = 0;
    const char *str = nullptr;
    size_t len = 0;
    dew_error_get_info(error, &code, &str, &len);
    MESSAGE("dew_dataset_create failed (", code, "): ", std::string(str ? str : "", len));
  }
  REQUIRE(dataset != nullptr);
  const auto ready = dew_dataset_wait_ready(dataset, -1);
  if (ready != dew_dataset_ready)
  {
    dew_error_t *ready_error = nullptr;
    dew_dataset_get_error(dataset, &ready_error);
    int code = 0;
    const char *str = nullptr;
    size_t len = 0;
    if (ready_error)
      dew_error_get_info(ready_error, &code, &str, &len);
    MESSAGE("dataset not ready (", int(ready), ") code ", code, ": ", std::string(str ? str : "", len));
  }
  REQUIRE(ready == dew_dataset_ready);

  dew_region_request_t spec{};
  for (int i = 0; i < 3; i++)
  {
    spec.aabb_min[i] = -1.0;
    spec.aabb_max[i] = double(k_amend_grid) * 2.0 + 1.0;
  }
  spec.lod_mode = dew_lod_full;
  spec.clip_mode = dew_clip_node;
  spec.position_format = dew_position_r64_absolute;
  auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
  REQUIRE(request != nullptr);
  REQUIRE(dew_request_wait(request, -1) == dew_request_completed);
  dew_request_result_t result{};
  REQUIRE(dew_request_get_result(request, &result) == 1);
  MESSAGE("from the bucket: ", result.point_count, " points");
  CHECK(result.point_count == 2 * k_amend_points);
  dew_request_release(request);
  dew_dataset_close(dataset);

  std::remove(cache);
}

// ---------------------------------------------------------------------------------------------
// 14. add_attribute / add_data_for_attribute: giving a converted dataset an attribute it never had.

// The iterative workflow, end to end. Convert the geometry once carrying a join key, close, reopen,
// and hand the dataset a value per key. Nothing is reconverted and no unit's existing blobs move.
//
// The key attribute is an ordinary attribute (here a u32 global point index) declared by the reader
// at ingest. It is what makes the join possible at all: the converter reorders every point by morton
// code, splits it across nodes, collapses chunks and samples LOD levels, so "my Nth point" means
// nothing by the time the data is on disk.
//
// LOD nodes come along for free -- an LOD node's key values are the keys of the points sampled into
// it, so joining against the same table gives each one its true value. That only works if the key
// SURVIVES to the LOD levels, which needs lod_all_attributes: the default keep-list is
// rgb/intensity/classification and would drop the key at the first LOD level.

namespace
{

constexpr const char *k_key_name = "point_key";
constexpr const char *k_amended_attribute = "reflectance";
constexpr const char *k_keyed_input = "keyed";

struct keyed_emit_state_t
{
  uint32_t emitted = 0;
};

dew_converter_file_pre_init_info_t keyed_pre_init(const char *, size_t, dew_error_t **)
{
  dew_converter_file_pre_init_info_t info{};
  info.approximate_point_count = k_amend_points;
  info.found_point_count = 1;
  info.approximate_point_size_bytes = 16;
  info.scale[0] = info.scale[1] = info.scale[2] = 1.0;
  info.found_scale = 1;
  return info;
}

void keyed_init(const char *, size_t, dew_converter_header_t *header, dew_attributes_t *attributes, void **user_ptr, dew_error_t **)
{
  header->point_count = k_amend_points;
  for (int i = 0; i < 3; i++)
  {
    header->offset[i] = 0.0;
    header->scale[i] = 1.0;
    header->min[i] = 0.0;
    header->max[i] = double(k_amend_grid);
  }
  dew_attributes_add_attribute(attributes, DEW_ATTRIBUTE_XYZ, uint32_t(strlen(DEW_ATTRIBUTE_XYZ)), dew_type_i32, dew_components_3);
  dew_attributes_add_attribute(attributes, k_key_name, uint32_t(strlen(k_key_name)), dew_type_u32, dew_components_1);
  *user_ptr = new keyed_emit_state_t();
}

// The key IS the point's index in the grid, so a point at key k sits at a position this test can
// recompute -- which is what lets the check below tie a returned value back to a returned position
// without trusting the ordering of either.
void keyed_convert_data(void *user_ptr, const dew_converter_header_t *, const dew_attribute_t *, uint32_t, uint32_t max_points, dew_blob_t *buffers, uint32_t, uint32_t *points_read, uint8_t *done,
                        dew_error_t **)
{
  auto *state = static_cast<keyed_emit_state_t *>(user_ptr);
  const uint32_t remaining = k_amend_points - state->emitted;
  const uint32_t n = remaining < max_points ? remaining : max_points;
  auto *xyz = static_cast<int32_t *>(buffers[0].data);
  auto *keys = static_cast<uint32_t *>(buffers[1].data);
  for (uint32_t i = 0; i < n; i++)
  {
    const uint32_t p = state->emitted + i;
    xyz[i * 3 + 0] = int32_t(p % k_amend_grid);
    xyz[i * 3 + 1] = int32_t((p / k_amend_grid) % k_amend_grid);
    xyz[i * 3 + 2] = int32_t(p / (k_amend_grid * k_amend_grid));
    keys[i] = p;
  }
  state->emitted += n;
  *points_read = n;
  *done = state->emitted >= k_amend_points ? 1 : 0;
}

void keyed_destroy_user_ptr(void *user_ptr)
{
  delete static_cast<keyed_emit_state_t *>(user_ptr);
}

void set_keyed_callbacks(dew_converter_t *converter)
{
  dew_converter_file_convert_callbacks_t callbacks{};
  callbacks.pre_init = keyed_pre_init;
  callbacks.init = keyed_init;
  callbacks.convert_data = keyed_convert_data;
  callbacks.destroy_user_ptr = keyed_destroy_user_ptr;
  dew_converter_set_file_converter_callbacks(converter, callbacks);
}

// The value a given key must come back with. Distinct per key and never zero, so "the amend did not
// reach this point" (zeros) cannot pass for a real value.
float expected_value(uint32_t key)
{
  return 1.0f + float(key) * 0.5f;
}

// A mutable dataset of keyed points, node limit tuned low enough to force several nodes and LOD
// levels, with the key kept all the way up the pyramid.
bool build_keyed_dataset(const char *path)
{
  std::remove(path);
  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_truncate, &error);
  if (!converter)
  {
    if (error)
      dew_error_destroy(error);
    return false;
  }
  dew_converter_set_mutable(converter, 1);
  dew_converter_set_lod_all_attributes(converter, 1);
  dew_converter_set_node_point_limit(converter, 700);
  set_keyed_callbacks(converter);
  dew_converter_str_buffer name{k_keyed_input, uint32_t(strlen(k_keyed_input))};
  dew_converter_add_data_file(converter, &name, 1);
  dew_converter_wait_idle(converter);
  const bool ok = dew_converter_status(converter) != dew_conversion_status_error;
  dew_converter_destroy(converter);
  return ok;
}

} // namespace

TEST_CASE("amend: a converted dataset gains an attribute it was never converted with")
{
  constexpr const char *path = "attribute_amend_add.dew";
  REQUIRE(build_keyed_dataset(path));

  // Reopen and amend. Separate session on purpose: this is the workflow -- convert now, produce the
  // derived values later, join them on afterwards.
  {
    dew_error_t *error = nullptr;
    auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
    REQUIRE(converter != nullptr);
    dew_converter_set_mutable(converter, 1);
    set_keyed_callbacks(converter);

    REQUIRE(dew_converter_add_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), k_key_name, uint32_t(strlen(k_key_name)), dew_type_r32, dew_components_1) == 1);

    // Delivered in two chunks, because a real caller streams: the second call must extend the table
    // rather than replace it.
    std::vector<uint64_t> keys(k_amend_points);
    std::vector<float> values(k_amend_points);
    for (uint32_t k = 0; k < k_amend_points; k++)
    {
      keys[k] = k;
      values[k] = expected_value(k);
    }
    const uint32_t half = k_amend_points / 2;
    REQUIRE(dew_converter_add_data_for_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), keys.data(), values.data(), half) == 1);
    REQUIRE(dew_converter_add_data_for_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), keys.data() + half, values.data() + half, k_amend_points - half) == 1);

    dew_converter_commit_attributes(converter);
    REQUIRE(dew_converter_status(converter) != dew_conversion_status_error);
    dew_converter_finalize(converter);
    REQUIRE(dew_converter_status(converter) != dew_conversion_status_error);
    dew_converter_destroy(converter);
  }

  // Read it back through the ordinary query path -- not through converter internals. The attribute
  // has to be a first-class part of the dataset, indistinguishable from one that was converted in.
  dew_error_t *error = nullptr;
  auto *dataset = dew_dataset_create(path, uint32_t(strlen(path)), nullptr, 0, nullptr, nullptr, &error);
  REQUIRE(dataset != nullptr);
  REQUIRE(dew_dataset_wait_ready(dataset, -1) == dew_dataset_ready);

  const char *attributes[] = {k_key_name, k_amended_attribute};
  dew_region_request_t spec{};
  for (int i = 0; i < 3; i++)
  {
    spec.aabb_min[i] = -1.0;
    spec.aabb_max[i] = double(k_amend_grid) + 1.0;
  }
  spec.lod_mode = dew_lod_full;
  spec.clip_mode = dew_clip_node;
  spec.attribute_names = attributes;
  spec.attribute_count = 2;
  spec.position_format = dew_position_r64_absolute;

  auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
  REQUIRE(request != nullptr);
  REQUIRE(dew_request_wait(request, -1) == dew_request_completed);
  dew_request_result_t result{};
  REQUIRE(dew_request_get_result(request, &result) == 1);
  REQUIRE(result.point_count == k_amend_points);

  const dew_attribute_buffer_t *key_buffer = nullptr;
  const dew_attribute_buffer_t *value_buffer = nullptr;
  for (uint32_t i = 0; i < result.buffer_count; i++)
  {
    if (!result.buffers[i].name)
      continue;
    if (strcmp(result.buffers[i].name, k_key_name) == 0)
      key_buffer = &result.buffers[i];
    else if (strcmp(result.buffers[i].name, k_amended_attribute) == 0)
      value_buffer = &result.buffers[i];
  }
  // The attribute exists at all -- the query resolved it by name against the amended configs.
  REQUIRE(value_buffer != nullptr);
  REQUIRE(key_buffer != nullptr);

  MESSAGE("key buffer: ", key_buffer->size_bytes, " bytes type ", int(key_buffer->type), "; value buffer: ", value_buffer->size_bytes, " bytes type ", int(value_buffer->type), "; points ",
          result.point_count);
  REQUIRE(key_buffer->size_bytes == result.point_count * sizeof(uint32_t));
  REQUIRE(value_buffer->size_bytes == result.point_count * sizeof(float));
  const auto *keys = static_cast<const uint32_t *>(key_buffer->data);
  const auto *values = static_cast<const float *>(value_buffer->data);
  uint64_t correct = 0, zero = 0, wrong = 0;
  std::vector<uint8_t> seen(k_amend_points, 0);
  for (uint64_t p = 0; p < result.point_count; p++)
  {
    const uint32_t key = keys[p];
    REQUIRE(key < k_amend_points);
    seen[key] = 1;
    if (values[p] == expected_value(key))
      correct++;
    else if (values[p] == 0.0f)
      zero++;
    else
      wrong++;
  }
  MESSAGE("amended: ", correct, " correct, ", zero, " zero, ", wrong, " wrong, over ", result.node_count, " nodes");
  // Every point carries the value its own key was given -- not a neighbour's, and not a zero from a
  // buffer the join never filled.
  CHECK(wrong == 0);
  CHECK(zero == 0);
  CHECK(correct == k_amend_points);
  // And every key appeared, so the frontier really was the whole dataset.
  CHECK(std::count(seen.begin(), seen.end(), uint8_t(0)) == 0);

  dew_request_release(request);
  dew_dataset_close(dataset);
  std::remove(path);
}

TEST_CASE("amend: LOD nodes get the amended attribute, sampled point by sampled point")
{
  // The claim that makes this cheap: an LOD node holds a SUBSET of its descendants' points, and its
  // key buffer holds those points' keys. Joining it against the same table therefore needs no
  // provenance metadata, no re-LOD, and no second pass -- the key attribute IS the provenance.
  //
  // Checked at a COARSE lod_pixel_size so the walk stops above the leaves and the frontier is made
  // of LOD nodes. Every value returned there must still be its own point's.
  constexpr const char *path = "attribute_amend_add_lod.dew";
  REQUIRE(build_keyed_dataset(path));

  {
    dew_error_t *error = nullptr;
    auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
    REQUIRE(converter != nullptr);
    dew_converter_set_mutable(converter, 1);
    set_keyed_callbacks(converter);
    REQUIRE(dew_converter_add_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), k_key_name, uint32_t(strlen(k_key_name)), dew_type_r32, dew_components_1) == 1);
    std::vector<uint64_t> keys(k_amend_points);
    std::vector<float> values(k_amend_points);
    for (uint32_t k = 0; k < k_amend_points; k++)
    {
      keys[k] = k;
      values[k] = expected_value(k);
    }
    REQUIRE(dew_converter_add_data_for_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), keys.data(), values.data(), k_amend_points) == 1);
    dew_converter_commit_attributes(converter);
    dew_converter_finalize(converter);
    REQUIRE(dew_converter_status(converter) != dew_conversion_status_error);
    dew_converter_destroy(converter);
  }

  dew_error_t *error = nullptr;
  auto *dataset = dew_dataset_create(path, uint32_t(strlen(path)), nullptr, 0, nullptr, nullptr, &error);
  REQUIRE(dataset != nullptr);
  REQUIRE(dew_dataset_wait_ready(dataset, -1) == dew_dataset_ready);

  const char *attributes[] = {k_key_name, k_amended_attribute};
  dew_region_request_t spec{};
  for (int i = 0; i < 3; i++)
  {
    spec.aabb_min[i] = -1.0;
    spec.aabb_max[i] = double(k_amend_grid) + 1.0;
  }
  // An explicit level, not a point budget: the budget only bites once nodes have been EMITTED, and
  // on a tree this small the whole leaf frontier is reached before the running total moves at all.
  // Level 5 is tree 0's deepest level (its root cell spans lods 9..5), so the walk stops there and
  // hands back LOD nodes.
  spec.lod_mode = dew_lod_level;
  spec.lod = 5;
  spec.clip_mode = dew_clip_node;
  spec.attribute_names = attributes;
  spec.attribute_count = 2;
  spec.position_format = dew_position_r64_absolute;

  auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
  REQUIRE(request != nullptr);
  REQUIRE(dew_request_wait(request, -1) == dew_request_completed);
  dew_request_result_t result{};
  REQUIRE(dew_request_get_result(request, &result) == 1);
  // A coarse frontier: strictly fewer points than the dataset, which is what proves these are LOD
  // nodes and not the leaves.
  MESSAGE("coarse frontier: ", result.point_count, " of ", k_amend_points, " points over ", result.node_count, " nodes");
  REQUIRE(result.point_count > 0);
  REQUIRE(result.point_count < k_amend_points);

  const dew_attribute_buffer_t *key_buffer = nullptr;
  const dew_attribute_buffer_t *value_buffer = nullptr;
  for (uint32_t i = 0; i < result.buffer_count; i++)
  {
    if (!result.buffers[i].name)
      continue;
    if (strcmp(result.buffers[i].name, k_key_name) == 0)
      key_buffer = &result.buffers[i];
    else if (strcmp(result.buffers[i].name, k_amended_attribute) == 0)
      value_buffer = &result.buffers[i];
  }
  REQUIRE(value_buffer != nullptr);
  REQUIRE(key_buffer != nullptr);

  const auto *keys = static_cast<const uint32_t *>(key_buffer->data);
  const auto *values = static_cast<const float *>(value_buffer->data);
  uint64_t correct = 0, mismatched = 0;
  for (uint64_t p = 0; p < result.point_count; p++)
  {
    if (keys[p] < k_amend_points && values[p] == expected_value(keys[p]))
      correct++;
    else
      mismatched++;
  }
  MESSAGE("LOD frontier: ", correct, " correct, ", mismatched, " mismatched");
  CHECK(mismatched == 0);
  CHECK(correct == result.point_count);

  dew_request_release(request);
  dew_dataset_close(dataset);
  std::remove(path);
}

TEST_CASE("amend: a key the dataset does not carry leaves it untouched rather than corrupt")
{
  // The failure that must NOT be silent-but-wrong. Joining on an attribute no unit has cannot match
  // anything; the dataset has to come back exactly as it was rather than gain a buffer of zeros
  // masquerading as data, or worse, a config claiming a slot the location vector does not have.
  constexpr const char *path = "attribute_amend_add_nokey.dew";
  REQUIRE(build_keyed_dataset(path));

  {
    dew_error_t *error = nullptr;
    auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
    REQUIRE(converter != nullptr);
    dew_converter_set_mutable(converter, 1);
    set_keyed_callbacks(converter);
    REQUIRE(dew_converter_add_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), "not_an_attribute", 16, dew_type_r32, dew_components_1) == 1);
    std::vector<uint64_t> keys{0, 1, 2};
    std::vector<float> values{1.0f, 2.0f, 3.0f};
    REQUIRE(dew_converter_add_data_for_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), keys.data(), values.data(), 3) == 1);
    dew_converter_commit_attributes(converter);
    // Not an error: a key that no unit carries is the same case as a key that only SOME units carry,
    // which is legal and expected (LOD slimming drops it). It amends nothing.
    REQUIRE(dew_converter_status(converter) != dew_conversion_status_error);
    dew_converter_finalize(converter);
    dew_converter_destroy(converter);
  }

  // Still readable, still complete, and without the attribute.
  dew_error_t *error = nullptr;
  auto *dataset = dew_dataset_create(path, uint32_t(strlen(path)), nullptr, 0, nullptr, nullptr, &error);
  REQUIRE(dataset != nullptr);
  REQUIRE(dew_dataset_wait_ready(dataset, -1) == dew_dataset_ready);
  dew_region_request_t spec{};
  for (int i = 0; i < 3; i++)
  {
    spec.aabb_min[i] = -1.0;
    spec.aabb_max[i] = double(k_amend_grid) + 1.0;
  }
  spec.lod_mode = dew_lod_full;
  spec.clip_mode = dew_clip_node;
  spec.position_format = dew_position_r64_absolute;
  auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
  REQUIRE(request != nullptr);
  REQUIRE(dew_request_wait(request, -1) == dew_request_completed);
  dew_request_result_t result{};
  REQUIRE(dew_request_get_result(request, &result) == 1);
  CHECK(result.point_count == k_amend_points);
  dew_request_release(request);
  dew_dataset_close(dataset);
  std::remove(path);
}

TEST_CASE("amend: declaring an attribute on a dataset that is not mutable is refused")
{
  // A finalized dataset has trees that were banded and, in destination mode, evicted -- an amend has
  // to read every unit it touches, so it must be refused up front rather than half-applied.
  constexpr const char *path = "attribute_amend_add_immutable.dew";
  REQUIRE(build_keyed_dataset(path));

  dew_error_t *error = nullptr;
  auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
  REQUIRE(converter != nullptr);
  set_keyed_callbacks(converter);
  CHECK(dew_converter_is_mutable(converter) == 1); // the dataset was left mutable; do not set it here
  dew_converter_finalize(converter);
  CHECK(dew_converter_is_mutable(converter) == 0);

  CHECK(dew_converter_add_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), k_key_name, uint32_t(strlen(k_key_name)), dew_type_r32, dew_components_1) == 0);
  CHECK(dew_converter_status(converter) == dew_conversion_status_error);
  dew_converter_destroy(converter);
  std::remove(path);
}

TEST_CASE("amend: an amended attribute survives a later LOD pass")
{
  // THE DURABILITY QUESTION. An amend writes one blob per unit and appends it to that unit's
  // config. A later LOD pass REPLACES an LOD unit's storage wholesale (tree_lod_generator takes the
  // add_storage replacement branch) and rebuilds its config as the union of its children's, then
  // slims. So: does an attribute added by an amend still exist after the pyramid is regenerated?
  //
  // It matters because a mutable dataset is exactly the kind that gets more input later, and adding
  // input lowers the LOD floor and re-runs the pass over everything the new points touch.
  constexpr const char *path = "attribute_amend_relod.dew";
  REQUIRE(build_keyed_dataset(path));

  {
    dew_error_t *error = nullptr;
    auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
    REQUIRE(converter != nullptr);
    dew_converter_set_mutable(converter, 1);
    set_keyed_callbacks(converter);
    REQUIRE(dew_converter_add_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), k_key_name, uint32_t(strlen(k_key_name)), dew_type_r32, dew_components_1) == 1);
    std::vector<uint64_t> keys(k_amend_points);
    std::vector<float> values(k_amend_points);
    for (uint32_t k = 0; k < k_amend_points; k++)
    {
      keys[k] = k;
      values[k] = expected_value(k);
    }
    REQUIRE(dew_converter_add_data_for_attribute(converter, k_amended_attribute, uint32_t(strlen(k_amended_attribute)), keys.data(), values.data(), k_amend_points) == 1);
    dew_converter_commit_attributes(converter);
    REQUIRE(dew_converter_status(converter) != dew_conversion_status_error);
    dew_converter_destroy(converter);
  }

  const auto coarse_reflectance = [&](const char *stage) {
    dew_error_t *error = nullptr;
    auto *dataset = dew_dataset_create(path, uint32_t(strlen(path)), nullptr, 0, nullptr, nullptr, &error);
    REQUIRE(dataset != nullptr);
    REQUIRE(dew_dataset_wait_ready(dataset, -1) == dew_dataset_ready);
    const char *attributes[] = {k_key_name, k_amended_attribute};
    dew_region_request_t spec{};
    for (int i = 0; i < 3; i++)
    {
      spec.aabb_min[i] = -1.0;
      spec.aabb_max[i] = double(k_amend_grid) + 1.0;
    }
    spec.lod_mode = dew_lod_level;
    spec.lod = 5; // tree 0's deepest level -- a frontier of LOD nodes, not leaves
    spec.clip_mode = dew_clip_node;
    spec.attribute_names = attributes;
    spec.attribute_count = 2;
    spec.position_format = dew_position_r64_absolute;
    auto *request = dew_dataset_request_region(dataset, &spec, nullptr);
    REQUIRE(request != nullptr);
    REQUIRE(dew_request_wait(request, -1) == dew_request_completed);
    dew_request_result_t result{};
    REQUIRE(dew_request_get_result(request, &result) == 1);

    const dew_attribute_buffer_t *key_buffer = nullptr;
    const dew_attribute_buffer_t *value_buffer = nullptr;
    for (uint32_t i = 0; i < result.buffer_count; i++)
    {
      if (!result.buffers[i].name)
        continue;
      if (strcmp(result.buffers[i].name, k_key_name) == 0)
        key_buffer = &result.buffers[i];
      else if (strcmp(result.buffers[i].name, k_amended_attribute) == 0)
        value_buffer = &result.buffers[i];
    }
    uint64_t correct = 0, wrong = 0, zero = 0;
    if (key_buffer && value_buffer && value_buffer->size_bytes)
    {
      const auto *keys = static_cast<const uint32_t *>(key_buffer->data);
      const auto *values = static_cast<const float *>(value_buffer->data);
      for (uint64_t p = 0; p < result.point_count; p++)
      {
        if (keys[p] < k_amend_points && values[p] == expected_value(keys[p]))
          correct++;
        else if (values[p] == 0.0f)
          zero++;  // a point the amend never covered reads as zero; that is the contract, not damage
        else
          wrong++;
      }
    }
    MESSAGE(stage, ": ", result.point_count, " coarse points, reflectance ", value_buffer ? "present" : "ABSENT", ", ", correct, " correct, ", zero, " zero (never amended), ", wrong, " WRONG");
    const uint64_t points = result.point_count;
    dew_request_release(request);
    dew_dataset_close(dataset);
    return std::make_tuple(points, correct, wrong, value_buffer != nullptr && value_buffer->size_bytes != 0);
  };

  auto before = coarse_reflectance("after the amend");
  REQUIRE(std::get<3>(before));
  REQUIRE(std::get<0>(before) > 0);
  CHECK(std::get<2>(before) == 0);
  CHECK(std::get<1>(before) == std::get<0>(before));

  // Now GROW the dataset. A new input lowers the LOD floor, so the pass re-runs over everything the
  // new points touch -- which, since this input covers the same cell, is the whole pyramid.
  {
    dew_error_t *error = nullptr;
    auto *converter = dew_converter_create(path, strlen(path), dew_open_file_semantics_open_existing, &error);
    REQUIRE(converter != nullptr);
    dew_converter_set_mutable(converter, 1);
    set_keyed_callbacks(converter);
    constexpr const char *second = "keyed-second";
    dew_converter_str_buffer name{second, uint32_t(strlen(second))};
    dew_converter_add_data_file(converter, &name, 1);
    dew_converter_wait_idle(converter);
    REQUIRE(dew_converter_status(converter) != dew_conversion_status_error);
    dew_converter_finalize(converter);
    dew_converter_destroy(converter);
  }

  auto after = coarse_reflectance("after adding an input (re-LOD)");
  // The attribute must still be there, and still right. If a regenerated LOD node dropped it, the
  // buffer comes back absent or zero-filled -- the amend would be a one-shot on any dataset that
  // ever grows again, which is precisely the kind an amend runs on.
  // Still present, and no point may hold a value that is neither its own nor zero. Zeros ARE
  // expected: the second input's points were never amended, and a point with no table entry reads as
  // zero by contract. What would be damage is a point carrying some OTHER point's value, which is
  // what a regenerated LOD node produces if the amend's slot mapping does not survive.
  CHECK(std::get<3>(after));
  CHECK(std::get<2>(after) == 0);
  CHECK(std::get<1>(after) > 0);

  std::remove(path);
}
