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
#include "input_header.hpp"

#include <assert.h>

namespace dew::converter
{
using namespace dew::core;

// ZERO-INITIALIZED, not merely allocated. A destination attribute set is the UNION of its sources'
// attributes, and a source that lacks one contributes NOTHING to that buffer -- the LOD generator
// skips it (tree_lod_generator.cpp, source_index < 0). Those points' bytes are never written, so
// with `new uint8_t[n]` (default-initialization, i.e. none for a scalar) they would carry whatever
// was on the heap straight into the dataset. Zero is the same value dew_access substitutes for an
// absent attribute, so the two paths now agree.
void attribute_buffers_initialize(const std::vector<point_format_t> &attributes_def, attribute_buffers_t &buffers, uint32_t point_count)
{
  buffers.data.reserve(attributes_def.size());
  buffers.buffers.reserve(attributes_def.size());
  for (auto &attribute : attributes_def)
  {
    uint32_t buffer_size = size_for_format(attribute.type) * uint32_t(attribute.components) * point_count;
    buffers.data.emplace_back(new uint8_t[buffer_size]());
    buffers.buffers.emplace_back(buffers.data.back().get(), buffer_size);
  }
}

void attribute_buffers_initialize(const std::vector<point_format_t> &attributes_def, attribute_buffers_t &buffers, uint32_t point_count, std::unique_ptr<uint8_t[]> &&morton_attribute_buffer)
{
  buffers.data.reserve(attributes_def.size());
  buffers.buffers.reserve(attributes_def.size());
  bool first = true;
  for (auto &attribute : attributes_def)
  {
    uint32_t buffer_size = size_for_format(attribute.type) * uint64_t(attribute.components) * point_count;
    if (first)
    {
      buffers.data.emplace_back(std::move(morton_attribute_buffer));
      first = false;
    }
    else
    {
      buffers.data.emplace_back(new uint8_t[buffer_size]()); // zeroed: see the overload above
    }
    buffers.buffers.emplace_back(buffers.data.back().get(), buffer_size);
  }
}

void attribute_buffers_adjust_buffers_to_size(const std::vector<point_format_t> &attributes_def, attribute_buffers_t &buffers, uint32_t point_count)
{
  assert(attributes_def.size() == buffers.buffers.size());

  for (int i = 0; i < int(attributes_def.size()); i++)
  {
    auto &buffer = buffers.buffers[i];
    buffer.size = size_for_format(attributes_def[i].type) * uint32_t(attributes_def[i].components) * point_count;
  }
}
} // namespace dew::converter
