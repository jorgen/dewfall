/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2021  Jørgen Lind
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

#include <dew/core/types.h>

#include "error.hpp"
#include "morton.hpp"

#include <fmt/format.h>
#include <cassert>
#include <cstring>
#include <memory>
#include <vector>

namespace dew::core
{
struct input_data_id_t
{
  uint32_t data;
  uint32_t sub;
};

inline bool operator<(const input_data_id_t a, const input_data_id_t b)
{
  return a.data < b.data || (a.data == b.data && a.sub < b.sub);
}

inline bool operator==(const input_data_id_t a, const input_data_id_t b)
{
  return a.data == b.data && a.sub == b.sub;
}

inline bool operator!=(const input_data_id_t a, const input_data_id_t b)
{
  return !(a == b);
}

inline bool input_data_id_is_leaf(input_data_id_t input)
{
  return !(input.sub & decltype(input.sub)(1) << 31);
}

// Collapsed-leaf units (a final leaf's subsets merged into its own per-node unit) come from their
// own persisted counter seeded 1<<62: bit 31 of .sub stays clear (they ARE leaf data -- stats and
// perf bucket them as source), bit 30 set keeps them disjoint from reader chunk ids (whose sub
// counter the reader asserts below 1<<30).
inline bool input_data_id_is_collapsed_leaf(input_data_id_t input)
{
  return (input.sub & 0xC0000000u) == 0x40000000u;
}

struct file_error_t
{
  input_data_id_t input_id;
  dew_error_t error;
};

struct attributes_id_t
{
  uint32_t data;
};

inline bool operator==(const attributes_id_t a, const attributes_id_t b)
{
  return memcmp(&a, &b, sizeof(a)) == 0;
}

inline bool operator!=(const attributes_id_t a, const attributes_id_t b)
{
  return memcmp(&a, &b, sizeof(a)) != 0;
}

struct input_name_ref_t
{
  const char *name;
  uint32_t name_length;
};

} // namespace dew::core (temporarily close for public type)

struct dew_attributes_t
{
  std::vector<dew_attribute_t> attributes;
  std::vector<std::unique_ptr<char[]>> attribute_names;
};

namespace dew::core
{
struct attribute_buffers_t
{
  std::vector<dew_blob_t> buffers;
  std::vector<std::unique_ptr<uint8_t[]>> data;
};

struct storage_location_t
{
  storage_location_t()
    : file_id(0)
    , size(0)
    , offset(0)
  {
  }

  storage_location_t(uint32_t a_file_id, uint32_t a_size, uint64_t a_offset)
    : file_id(a_file_id)
    , size(a_size)
    , offset(a_offset)
  {
  }

  uint32_t file_id;
  uint32_t size;
  uint64_t offset;
};

inline bool operator==(const storage_location_t a, const storage_location_t b)
{
  return memcmp(&a, &b, sizeof(a)) == 0;
}

inline bool operator!=(const storage_location_t a, const storage_location_t b)
{
  return memcmp(&a, &b, sizeof(a)) != 0;
}

struct point_format_t
{
  point_format_t() = default;

  point_format_t(dew_type_t a_type, dew_components_t a_components)
    : type(a_type)
    , components(a_components)
  {
  }

  dew_type_t type;
  dew_components_t components;
};

struct storage_header_t
{
  input_data_id_t input_id;
  uint32_t point_count;
  point_format_t point_format;
  morton::morton192_t morton_min;
  morton::morton192_t morton_max;
  int lod_span;
};

inline void storage_header_initialize(storage_header_t &header)
{
  header.point_count = 0;

  morton::morton_init_min(header.morton_max);
  morton::morton_init_max(header.morton_min);
  header.lod_span = 255;
}

// Split a serialized points blob (a storage_header_t followed by the point bytes) into the header + a view of
// the point data. Storage-free -- it only reads the buffer -- so the decode path and a decode Web Worker can
// call it without pulling in the storage handler. (Moved here from storage_handler.hpp.)
inline bool deserialize_points(const dew_blob_t &data, storage_header_t &header, dew_blob_t &point_data, dew_error_t &error)
{
  if (data.size < sizeof(header))
  {
    error.code = 2;
    error.msg = "Invalid input size";
    return false;
  }
  auto input_bytes = static_cast<uint8_t *>(data.data);
  memcpy(&header, input_bytes, sizeof(header));
  point_data.size = data.size - sizeof(header);
  point_data.data = input_bytes + sizeof(header);
  return true;
}

// Deep-copy an attribute set, re-pointing each attribute's `name` at the target's own NUL-terminated
// copy (the source's names are owned by its attribute_names vector and outlive nothing). Lives here
// beside the type rather than with the write pipeline's buffer helpers: it is pure data manipulation
// on a core type, and the attribute registry needs it.
inline void attributes_copy(const dew_attributes_t &source, dew_attributes_t &target)
{
  assert(target.attributes.empty());
  assert(target.attribute_names.empty());
  target.attributes = source.attributes;
  for (int i = 0; i < int(source.attributes.size()); i++)
  {
    auto &target_attrib_name = target.attribute_names.emplace_back(new char[source.attributes[i].name_size + 1]);
    memcpy(target_attrib_name.get(), source.attribute_names[i].get(), source.attributes[i].name_size);
    target.attribute_names[i].get()[source.attributes[i].name_size] = 0;
    target.attributes[i].name = target.attribute_names[i].get();
  }
}

struct points_t
{
  storage_header_t header;
  attributes_id_t attributes_id;
  attribute_buffers_t buffers;
};

struct tree_config_t
{
  double scale = {};
  double offset[3] = {};
  bool store_original_order = false;
  // Max points per octree node: subdivision keeps every node at or below this, which is the primary
  // lever on per-node blob size. The default (200k) keeps the data-heavy attributes (xyz, gps_time)
  // under ~1MB compressed. Set via dew_converter_set_node_point_limit.
  uint32_t node_point_limit = 200000;
  // Read/sort chunk BYTE target: the reader sizes each input chunk to about this many bytes
  // (computed from the file's per-point width, clamped to [node_point_limit, k_max_chunk_points]).
  // Big chunks amortize read+sort; leaf collapse cuts them back to per-node units at finality.
  // Set via dew_converter_set_read_chunk_bytes. (Registry v3 serializes the grown struct.)
  uint64_t read_chunk_byte_target = 64ull << 20;
  // LOD nodes carry only the visual attributes (rgb/intensity/classification + position) unless this
  // opts back into duplicating everything. Set via dew_converter_set_lod_all_attributes. (v4 field.)
  uint8_t lod_all_attributes = 0;
  // EXPERIMENTAL, default OFF: coarsen each LOD node's cell until reps <= children/4. This compounds
  // bottom-up (each level keeps a quarter of the one below) and drives the top LOD levels to a single
  // point, which the renderer draws as an empty far field -- the render density system is calibrated to
  // the fixed lod-9 sampling rate, so coarsening beyond it needs the renderer taught per-node density
  // first. Left in place (gated) for that future redesign; do NOT enable for rendered datasets. (v4.)
  uint8_t lod_adaptive_sampling = 0;
  // MUTABLE dataset: never seal. A tree is normally flipped to tree_state_t::final as soon as the
  // done-morton watermark proves nothing more can land in it, and finality is what lets a tree be
  // uploaded to a band and then evicted from the local cache -- which is exactly what makes it
  // un-amendable. While this is set, trees stay `building` and no band is emitted, so the dataset
  // can keep taking input (and, later, gain attributes) across sessions. Collapse and LOD run as
  // normal, and no reader consults tree_state, so a mutable dataset still reads and renders.
  // Cleared by dew_converter_finalize, which then seals in the ordinary way.
  // One of the reserved bytes, so sizeof stays 56 and TRG4 blobs are unchanged -- an older file
  // reads zero here, which is today's always-seal behaviour.
  uint8_t mutable_dataset = 0;
  uint8_t reserved_[5] = {};
};
// Chunk point-count clamp: 8M points default cap (a decompressed morton blob is count x up to 24B --
// keep worst-case read spikes bounded); 16M is the hard ceiling (u32 subset offsets stay far clear).
inline constexpr uint32_t k_default_max_chunk_points = 8u << 20;
inline constexpr uint32_t k_hard_max_chunk_points = 16u << 20;

struct offset_t
{
  explicit offset_t(uint64_t a_data)
    : data(a_data)
  {
  }

  offset_t()
    : data(0)
  {
  }

  uint64_t data;
};

struct offset_in_subset_t
{
  explicit offset_in_subset_t(uint32_t a_data)
    : data(a_data)
  {
  }

  offset_in_subset_t()
    : data(0)
  {
  }

  uint32_t data;
};

struct index_t
{
  explicit index_t(uint32_t a_data)
    : data(a_data)
  {
  }

  uint32_t data;
};

struct point_count_t
{
  explicit point_count_t()
    : data(0)
  {
  }

  explicit point_count_t(uint32_t a_data)
    : data(a_data)
  {
  }

  uint32_t data = 0;
};

struct points_subset_t
{
  points_subset_t() = default;

  points_subset_t(input_data_id_t id, offset_in_subset_t a_offset, point_count_t a_count)
    : input_id(id)
    , offset(a_offset)
    , count(a_count)
  {
  }

  input_data_id_t input_id;
  offset_in_subset_t offset;
  point_count_t count;
};
} // namespace dew::core


template <>
struct fmt::formatter<dew::core::storage_location_t>
{
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx)
  {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(dew::core::storage_location_t const &location, FormatContext &ctx)
  {
    return fmt::format_to(ctx.out(), "storage_location_t(file_id={}, size={}, offset={})", location.file_id, location.size, location.offset);
  }
};
