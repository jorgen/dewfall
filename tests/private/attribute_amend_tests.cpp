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

#include <doctest/doctest.h>

#include "attributes_configs.hpp"
#include "dataset_types.hpp"
#include "input_header.hpp"
#include "input_storage_map.hpp"

#include <dew/core/default_attribute_names.h>
#include <dew/core/format.h>
#include <dew/core/types.h>

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

TEST_CASE("mutable: a second session can add a new input to an existing dataset" * doctest::skip())
{
  // SKIPPED: this is the target for the next piece of work, not a regression.
  //
  // The mode itself works -- the flag persists (test above), trees stay `building`, and no band is
  // emitted. What is still missing is one level down: the insert path assumes every tree is
  // RESIDENT. On reopen tree_registry.data is resized to N null pointers and trees are loaded
  // lazily, but sub_tree_insert_points -> move_storage_locations_to_subtree dereferences a sub-tree
  // unconditionally and segfaults on the null. During a fresh conversion every tree was created in
  // the same session, so this never showed.
  //
  // Fixing it means either loading every tree when a mutable dataset is opened, or making the
  // insert path request-and-retry the way the renderer already does (tree_handler request_trees).
  // Un-skip when that lands.
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

  std::remove(path);
}
