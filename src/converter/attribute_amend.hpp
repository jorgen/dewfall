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

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <attributes_configs.hpp>
#include <dataset_types.hpp>
#include <dew/core/error.h>

namespace dew::converter
{
class tree_handler_t;
class storage_handler_t;

// How much RAM the commit may use for one pass. Overridable with DEW_AMEND_BUDGET_BYTES, which the
// tests use to force many passes over a small dataset -- the K>1 path is otherwise unreachable
// without a dataset too big to put in a test.
uint64_t amend_memory_budget();

// An append-only file of fixed-width records, buffered. The amend's incoming values go here instead
// of into a hash map: a hash map made the commit's memory O(dataset), which measured at 52 bytes per
// point -- about 15 GB for a 294-million-point dataset.
class spill_file_t
{
public:
  spill_file_t() = default;
  ~spill_file_t();
  spill_file_t(const spill_file_t &) = delete;
  spill_file_t &operator=(const spill_file_t &) = delete;
  // Movable so a pending_attribute_t can live in a vector. The moved-from object must forget its
  // path as well as its handle: the destructor DELETES the file, and a moved-from copy that kept the
  // path would delete the spill out from under the object that now owns it.
  spill_file_t(spill_file_t &&other) noexcept;
  spill_file_t &operator=(spill_file_t &&other) noexcept;

  [[nodiscard]] bool open_write(const std::string &path, uint32_t record_size, std::string &error);
  [[nodiscard]] bool append(const uint8_t *record, std::string &error);
  [[nodiscard]] bool flush(std::string &error);
  void close();
  void remove_file();
  // Give up ownership without deleting. A spill_file_t deletes its file on destruction, which is
  // right for the value spill and wrong for anything whose file outlives the object that wrote it --
  // a sort run, the sorted output, or a file another spill_file_t is about to read. Forgetting this
  // deleted the value spill after the FIRST pass, so every pass after it found nothing.
  void release();

  [[nodiscard]] bool open_read(const std::string &path, uint32_t record_size, std::string &error);
  // Next record, or false at end of file. The returned pointer is into the read buffer and is valid
  // until the following call.
  [[nodiscard]] bool next(const uint8_t *&record, std::string &error);

  [[nodiscard]] const std::string &path() const
  {
    return _path;
  }
  [[nodiscard]] uint64_t records() const
  {
    return _records;
  }

private:
  std::string _path;
  FILE *_file = nullptr;
  uint32_t _record_size = 0;
  uint64_t _records = 0;
  std::vector<uint8_t> _buffer;
  size_t _buffer_used = 0;  // write: bytes staged. read: bytes valid.
  size_t _read_cursor = 0;
  bool _writing = false;
  bool _owns = false; // only a writer owns its file; a reader never deletes what it was pointed at
};

// One declared attribute, and the spilled values waiting to be joined onto the dataset.
//
// Deliberately keyed rather than ordered. The converter reorders every point by morton code, splits
// it across nodes, collapses chunks and samples it into LOD levels -- so a caller holding "the values
// for my Nth point" has no way to say where that point ended up. It names an attribute the dataset
// already carries instead, and supplies (key, value) pairs.
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
  uint32_t record_size = 0; // sizeof(uint64 key) + element_size

  spill_file_t spill;
  uint64_t last_key = 0;
  bool have_last = false;
  // Whether every key so far was >= its predecessor. The commit's merge needs the values sorted by
  // key; a caller that already emits them in order (a scan-ordered importer does, since its key is
  // (scan << k) | index) gets the sort for free.
  bool ascending = true;
  std::vector<uint8_t> record_scratch;
};

class attribute_amend_t
{
public:
  // Where spill files are written. Set by the processor to the dataset's own directory when that is
  // a local path, because that is where the caller already provisioned space; falls back to the
  // system temp directory.
  void set_spill_directory(std::string directory);

  // Declare a new attribute and the existing one to join it on. False if the name is taken by
  // another pending declaration, or the type is not a real point type.
  bool declare(const std::string &name, const std::string &key_attribute, dew_type_t type, dew_components_t components, std::string &error_message);
  // Buffer `count` values against `count` keys. Appends to the spill and returns; nothing is applied
  // to the dataset until commit. False if `name` was never declared or the spill could not be
  // written.
  bool add_data(const std::string &name, const uint64_t *keys, const void *values, uint64_t count);

  [[nodiscard]] bool empty() const
  {
    return _pending.empty();
  }
  void clear();

  // Apply every declaration to every unit of the dataset. Blocks. Requires every tree resident
  // (mutable mode loads them all at open) and must NOT run on the tree loop or the storage loop --
  // it blocks on reads and writes serviced by both.
  //
  // Per unit: read its key blob, build one new buffer, write it, append it to the unit's storage.
  // Nothing the unit already holds is read, recompressed or moved. Memory is bounded by
  // amend_memory_budget() regardless of how big the dataset or the value set is.
  [[nodiscard]] dew_error_t commit(tree_handler_t &tree_handler, storage_handler_t &storage_handler, core::attributes_configs_t &attributes_configs);

private:
  std::vector<pending_attribute_t> _pending;
  std::string _spill_directory;
  uint32_t _spill_serial = 0;
};

} // namespace dew::converter

#endif // DEW_ATTRIBUTE_AMEND_HPP
