/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2023  Jørgen Lind
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
#include "blob_manager.hpp"

#include <algorithm>
#include <assert.h>

free_blob_manager_t::free_blob_manager_t()
  : _next_offset{0}
{
}

free_blob_manager_t::offset_t free_blob_manager_t::register_blob(blob_size_t size)
{
  blob_size_t spillover_from_last_page = {0};
  page_t spillover_page = 0;
  for (auto page_it = _free_sections_by_page.begin(); page_it != _free_sections_by_page.end(); ++page_it)
  {
    auto &[page_number, sections] = *page_it;
    uint64_t page_offset = uint64_t(page_number) * FREE_BLOB_MANAGER_PAGE_SIZE;
    if (spillover_page + 1 != page_number || (sections.size() && sections.front().offset.data != page_offset))
    {
      spillover_from_last_page = {0};
    }
    for (uint32_t section_index = 0; section_index < sections.size(); section_index++)
    {
      auto &section = sections[section_index];
      auto current_section_size = blob_size_t{section.size.data + spillover_from_last_page.data};
      if (current_section_size.data >= size.data)
      {
        // Return the free section's own start, not the page start. A mid-page free
        // section (live data preceding it in the page) has section.offset > page_offset;
        // returning page_offset would hand back a live blob's location and overwrite it.
        // When spillover > 0 the front-section invariant guarantees section.offset == page_offset.
        offset_t return_offset = {section.offset.data - spillover_from_last_page.data};

        if (section.size.data == size.data - spillover_from_last_page.data)
        {
          sections.erase(sections.begin() + section_index);
        }
        else
        {
          section.offset.data += size.data - spillover_from_last_page.data;
          section.size.data -= size.data - spillover_from_last_page.data;
        }

        if (sections.empty())
        {
          _free_sections_by_page.erase(page_number);
        }

        if (spillover_from_last_page.data > 0)
        {
          // CONSUME THE TAIL WE JUST HANDED OUT. return_offset starts spillover_from_last_page bytes
          // BEFORE this section, inside the trailing free sections of the preceding pages -- so those
          // sections are now occupied and must leave the free list.
          //
          // This used to iterate `size / FREE_BLOB_MANAGER_PAGE_SIZE` times, which is ZERO for any
          // blob smaller than a page. The tail stayed free, the next request was handed the same
          // offset, and two blobs ended up on top of each other. It is silent: nothing fails until
          // something reads the blob that was overwritten.
          //
          // Walk back by PAGE NUMBER rather than by iterator: the current page's entry may already
          // have been erased just above, and erasing from an unordered_dense map invalidates
          // iterators.
          uint64_t remaining = spillover_from_last_page.data;
          page_t previous = page_number;
          while (remaining > 0 && previous > 0)
          {
            previous--;
            auto previous_it = _free_sections_by_page.find(previous);
            if (previous_it == _free_sections_by_page.end() || previous_it->second.empty())
              break;
            const uint64_t contributed = previous_it->second.back().size.data;
            if (contributed > remaining)
              break; // not a whole trailing section: it did not contribute to this spillover
            remaining -= contributed;
            if (previous_it->second.size() == 1)
              _free_sections_by_page.erase(previous_it);
            else
              previous_it->second.pop_back();
          }
          assert(remaining == 0 && "spillover consumed free space it could not account for");
        }
        return return_offset;
      }
    }

    if (sections.size() && sections.back().offset.data + sections.back().size.data == page_offset + free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE)
    {
      spillover_from_last_page.data += sections.back().size.data;
      spillover_page = page_number;
    }
    else
    {
      spillover_from_last_page = {0};
    }
  }

  offset_t new_offset = _next_offset;
  _next_offset.data += size.data;
  return new_offset;
}

[[nodiscard]] bool free_blob_manager_t::unregister_blob(offset_t total_offset, blob_size_t total_size)
{
  page_t start_page = total_offset.page();
  page_t end_page = page_t((total_offset.data + total_size.data - 1) / FREE_BLOB_MANAGER_PAGE_SIZE);

  if (total_offset.data + total_size.data > _next_offset.data)
  {
    return false;
  }

  for (page_t page = start_page; page <= end_page; ++page)
  {
    auto page_it = _free_sections_by_page.find(page);
    if (page_it == _free_sections_by_page.end())
    {
      page_it = _free_sections_by_page.emplace(page, std::vector<section_t>()).first;
    }
    offset_t page_start = {uint64_t(page) * FREE_BLOB_MANAGER_PAGE_SIZE};
    offset_t page_end = {page_start.data + FREE_BLOB_MANAGER_PAGE_SIZE};
    offset_t section_end = {std::min(total_offset.data + total_size.data, page_end.data)};
    offset_t offset_for_page = {std::max(total_offset.data, page_start.data)};
    blob_size_t size_for_page = {uint32_t(section_end.data - std::max(page_start.data, total_offset.data))};

    auto &sections = page_it->second;
    auto it = std::lower_bound(sections.begin(), sections.end(), offset_for_page, [](const section_t &section, const offset_t &val) { return section.offset.data < val.data; });

    if (it != sections.begin() && (it - 1)->offset.data + (it - 1)->size.data > offset_for_page.data)
    {
      return false;
    }

    if (it != sections.end() && it->offset.data < offset_for_page.data + size_for_page.data)
    {
      return false;
    }

    if (it != sections.begin() && (it - 1)->offset.data + (it - 1)->size.data == offset_for_page.data)
    {
      --it;
      it->size.data += size_for_page.data;
      auto next_it = it + 1;
      if (next_it != sections.end() && it->offset.data + it->size.data == next_it->offset.data)
      {
        it->size.data += next_it->size.data;
        sections.erase(next_it);
      }
    }
    else
    {
      // Insert at the sorted position (lower_bound `it`), not at the end: the per-page
      // sections vector must stay sorted by offset for the lower_bound overlap checks
      // above to work. Coalesce with the following section when adjacent.
      auto ins = sections.insert(it, section_t{offset_for_page, size_for_page});
      auto next_it = ins + 1;
      if (next_it != sections.end() && ins->offset.data + ins->size.data == next_it->offset.data)
      {
        ins->size.data += next_it->size.data;
        sections.erase(next_it);
      }
    }

    if (sections.empty())
    {
      _free_sections_by_page.erase(page_it);
    }
  }

  if (total_offset.data + total_size.data == _next_offset.data)
  {
    while (!_free_sections_by_page.empty())
    {
      auto last_page_it = --_free_sections_by_page.end();
      auto &last_section = last_page_it->second.back();
      if (last_section.offset.data + last_section.size.data != _next_offset.data)
      {
        break;
      }

      _next_offset.data -= last_section.size.data;
      if (last_page_it->second.size() == 1)
      {
        _free_sections_by_page.erase(last_page_it);
      }
      else
      {
        last_page_it->second.pop_back();
        break;
      }
    }
  }

  return true;
}

size_t free_blob_manager_t::get_free_sections_count() const
{
  size_t count = 0;
  bool last_page_ended_with_free_section = false;
  for (auto &[page_number, sections] : _free_sections_by_page)
  {
    if (last_page_ended_with_free_section && sections.size() && sections.front().offset.data == uint64_t(page_number) * FREE_BLOB_MANAGER_PAGE_SIZE)
    {
      count += sections.size() - 1;
    }
    else
    {
      count += sections.size();
    }
    last_page_ended_with_free_section = sections.size() && sections.back().offset.data + sections.back().size.data == uint64_t(page_number + 1) * FREE_BLOB_MANAGER_PAGE_SIZE;
  }
  return count;
}

size_t free_blob_manager_t::get_pages_count() const
{
  return _free_sections_by_page.size();
}

free_blob_manager_t::section_t free_blob_manager_t::get_free_section(page_t page, size_t n) const
{
  return _free_sections_by_page.at(page)[n];
}

free_blob_manager_t::offset_t free_blob_manager_t::get_file_size() const
{
  return {_next_offset.data};
}

uint32_t free_blob_manager_t::calculate_serialized_size() const
{
  uint32_t size = 0;

  auto page_count = uint32_t(_free_sections_by_page.size());
  size = sizeof(_next_offset) + sizeof(page_count);
  for (auto &[page_number, sections] : _free_sections_by_page)
  {
    auto free_sections_in_page_count = uint32_t(sections.size());
    size += sizeof(page_number) + sizeof(free_sections_in_page_count);
    size += uint32_t(sections.size() * sizeof(sections.front()));
  }
  return size;
}

void free_blob_manager_t::serialize_payload(uint8_t *data) const
{
  auto page_count = uint32_t(_free_sections_by_page.size());
  memcpy(data, &_next_offset, sizeof(_next_offset));
  data += sizeof(_next_offset);
  memcpy(data, &page_count, sizeof(page_count));
  data += sizeof(page_count);
  for (auto &[page_number, sections] : _free_sections_by_page)
  {
    memcpy(data, &page_number, sizeof(page_number));
    data += sizeof(page_number);
    auto free_sections_in_page_count = uint32_t(sections.size());
    memcpy(data, &free_sections_in_page_count, sizeof(free_sections_in_page_count));
    data += sizeof(free_sections_in_page_count);
    memcpy(data, sections.data(), sections.size() * sizeof(sections.front()));
    data += sections.size() * sizeof(sections.front());
  }
}

serialized_free_blob_manager_t free_blob_manager_t::serialize()
{
  serialized_free_blob_manager_t serialized = {};

  for (int i = 0; i < 5; i++)
  {
    auto calculted_size = calculate_serialized_size() + 32 * i;
    auto with_blob_offset = register_blob({calculted_size});
    auto calculated_size_with_blob_manager = calculate_serialized_size();
    if (calculated_size_with_blob_manager <= calculted_size)
    {
      serialized.size = calculted_size;
      serialized.offset = with_blob_offset.data;
      break;
    }
    auto did_unregister = unregister_blob(with_blob_offset, {calculted_size});
    (void)did_unregister;
    assert(did_unregister);
  }
  assert(serialized.size > 0);
  assert(serialized.offset > 0);

  serialized.data = std::make_shared<uint8_t[]>(serialized.size);
  serialize_payload(serialized.data.get());
  return serialized;
}

serialized_free_blob_manager_t free_blob_manager_t::serialize_preallocated(offset_t offset, blob_size_t size)
{
  assert(calculate_serialized_size() <= size.data);
  serialized_free_blob_manager_t serialized = {};
  serialized.offset = offset.data;
  serialized.size = size.data;
  serialized.data = std::make_shared<uint8_t[]>(size.data);
  serialize_payload(serialized.data.get());
  return serialized;
}

bool free_blob_manager_t::claim_blob(offset_t offset, blob_size_t size)
{
  if (size.data == 0)
    return false;
  const uint64_t begin = offset.data;
  const uint64_t end = offset.data + size.data;
  if (begin >= _next_offset.data)
  {
    // Tail claim: everything from the old tail up to `begin` becomes a free gap, the claimed
    // range extends the allocated tail. (In the checkpoint-replay use the gap is always empty.)
    const uint64_t gap_start = _next_offset.data;
    _next_offset.data = end;
    for (uint64_t gap_pos = gap_start; gap_pos < begin;)
    {
      const uint64_t page_end = (gap_pos / FREE_BLOB_MANAGER_PAGE_SIZE + 1) * FREE_BLOB_MANAGER_PAGE_SIZE;
      const uint64_t piece_end = begin < page_end ? begin : page_end;
      if (!unregister_blob({gap_pos}, {uint32_t(piece_end - gap_pos)}))
        return false;
      gap_pos = piece_end;
    }
    return true;
  }
  if (end > _next_offset.data)
    return false; // straddles the tail: not a shape register_blob ever produces
  // Interior claim: carve [begin, end) out of the free sections, page by page (sections never
  // span pages, mirroring unregister_blob's insertion).
  for (uint64_t pos = begin; pos < end;)
  {
    const page_t page = page_t(pos / FREE_BLOB_MANAGER_PAGE_SIZE);
    const uint64_t page_end = uint64_t(page + 1) * FREE_BLOB_MANAGER_PAGE_SIZE;
    const uint64_t piece_end = end < page_end ? end : page_end;
    auto page_it = _free_sections_by_page.find(page);
    if (page_it == _free_sections_by_page.end())
      return false;
    auto &sections = page_it->second;
    bool carved = false;
    for (uint32_t i = 0; i < uint32_t(sections.size()); i++)
    {
      auto &section = sections[i];
      const uint64_t s_begin = section.offset.data;
      const uint64_t s_end = s_begin + section.size.data;
      if (pos < s_begin || piece_end > s_end)
        continue;
      if (pos == s_begin && piece_end == s_end)
      {
        sections.erase(sections.begin() + i);
        if (sections.empty())
          _free_sections_by_page.erase(page_it);
      }
      else if (pos == s_begin)
      {
        section.offset.data = piece_end;
        section.size.data = uint32_t(s_end - piece_end);
      }
      else if (piece_end == s_end)
      {
        section.size.data = uint32_t(pos - s_begin);
      }
      else
      {
        section.size.data = uint32_t(pos - s_begin);
        section_t tail_piece{{piece_end}, {uint32_t(s_end - piece_end)}};
        sections.insert(sections.begin() + i + 1, tail_piece);
      }
      carved = true;
      break;
    }
    if (!carved)
      return false;
    pos = piece_end;
  }
  return true;
}

dew_error_t free_blob_manager_t::deserialize(const std::unique_ptr<uint8_t[]> &data, uint32_t size)
{
  if (size < sizeof(_next_offset) + sizeof(uint32_t))
  {
    return {1, "Data size too small for deserialization"};
  }

  uint8_t *data_ptr = data.get();
  uint32_t remaining_size = size;

  memcpy(&_next_offset, data_ptr, sizeof(_next_offset));
  data_ptr += sizeof(_next_offset);
  remaining_size -= sizeof(_next_offset);

  uint32_t page_count;
  memcpy(&page_count, data_ptr, sizeof(page_count));
  data_ptr += sizeof(page_count);
  remaining_size -= sizeof(page_count);

  for (uint32_t i = 0; i < page_count; i++)
  {
    if (remaining_size < sizeof(page_t) + sizeof(uint32_t))
    {
      return {1, "Insufficient data for page count"};
    }

    page_t page_number;
    memcpy(&page_number, data_ptr, sizeof(page_number));
    data_ptr += sizeof(page_number);
    remaining_size -= sizeof(page_number);

    uint32_t free_sections_in_page_count;
    memcpy(&free_sections_in_page_count, data_ptr, sizeof(free_sections_in_page_count));
    data_ptr += sizeof(free_sections_in_page_count);
    remaining_size -= sizeof(free_sections_in_page_count);

    if (remaining_size < free_sections_in_page_count * sizeof(section_t))
    {
      return {1, "Insufficient data for sections"};
    }

    std::vector<section_t> sections(free_sections_in_page_count);
    memcpy(sections.data(), data_ptr, free_sections_in_page_count * sizeof(sections.front()));
    data_ptr += free_sections_in_page_count * sizeof(sections.front());
    remaining_size -= free_sections_in_page_count * sizeof(sections.front());

    _free_sections_by_page.emplace(page_number, std::move(sections));
  }

  return {};
}
