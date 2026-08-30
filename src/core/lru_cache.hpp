/************************************************************************
** dewfall - point cloud management software.
** Copyright (C) 2024  Jorgen Lind
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

#include <ankerl/unordered_dense.h>

#include <cstdint>
#include <list>
#include <mutex>
#include <optional>

namespace dew::core
{

template <typename Key, typename Value, typename Hash, typename KeyEqual = std::equal_to<Key>>
class lru_cache_t
{
public:
  explicit lru_cache_t(uint64_t max_bytes)
    : _max_bytes(max_bytes)
  {
  }

  // `promote` false leaves the entry where it is instead of moving it to the front. For a read that
  // consumes a blob WHOLE there is no reason to keep it hot: whoever wanted it has it. A read that
  // takes only a SUBSET is different -- the other subsets belong to other nodes, which will be back
  // for the same blob -- so those promote as usual.
  std::optional<Value> get(const Key &key, bool promote = true)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _map.find(key);
    if (it == _map.end())
    {
      _miss_count++;
      return std::nullopt;
    }
    _hit_count++;
    if (promote)
      _lru_list.splice(_lru_list.begin(), _lru_list, it->second);
    return it->second->value;
  }

  // `hot` false inserts at the BACK -- first out rather than last. The miss path is what evicts, so
  // this is the half that matters for a blob read once and never again: without it every such read
  // pushes a genuinely reused blob one step closer to eviction.
  void put(const Key &key, Value value, uint64_t size, bool hot = true)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _map.find(key);
    if (it != _map.end())
    {
      _current_bytes -= it->second->size;
      it->second->value = std::move(value);
      it->second->size = size;
      _current_bytes += size;
      if (hot)
        _lru_list.splice(_lru_list.begin(), _lru_list, it->second);
    }
    else
    {
      if (hot)
        _lru_list.push_front({key, std::move(value), size});
      else
        _lru_list.push_back({key, std::move(value), size});
      _map[key] = hot ? _lru_list.begin() : std::prev(_lru_list.end());
      _current_bytes += size;
    }
    evict();
  }

  void erase(const Key &key)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _map.find(key);
    if (it != _map.end())
    {
      _current_bytes -= it->second->size;
      _lru_list.erase(it->second);
      _map.erase(it);
    }
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _map.clear();
    _lru_list.clear();
    _current_bytes = 0;
  }

  void set_max_bytes(uint64_t max_bytes)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _max_bytes = max_bytes;
    evict();
  }

  uint64_t current_bytes() const
  {
    std::lock_guard<std::mutex> lock(_mutex);
    return _current_bytes;
  }

  uint64_t hit_count() const
  {
    std::lock_guard<std::mutex> lock(_mutex);
    return _hit_count;
  }

  uint64_t miss_count() const
  {
    std::lock_guard<std::mutex> lock(_mutex);
    return _miss_count;
  }

private:
  struct entry_t
  {
    Key key;
    Value value;
    uint64_t size;
  };

  void evict()
  {
    while (_current_bytes > _max_bytes && !_lru_list.empty())
    {
      auto &back = _lru_list.back();
      _current_bytes -= back.size;
      _map.erase(back.key);
      _lru_list.pop_back();
    }
  }

  uint64_t _max_bytes;
  uint64_t _current_bytes = 0;
  uint64_t _hit_count = 0;
  uint64_t _miss_count = 0;
  std::list<entry_t> _lru_list;
  ankerl::unordered_dense::map<Key, typename std::list<entry_t>::iterator, Hash, KeyEqual> _map;
  mutable std::mutex _mutex;
};

} // namespace dew::core
