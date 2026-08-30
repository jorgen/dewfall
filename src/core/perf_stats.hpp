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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>

namespace dew::core
{

struct io_counter_t
{
  std::atomic<uint64_t> total_bytes{0};
  std::atomic<uint64_t> total_time_us{0};
  std::atomic<uint32_t> operation_count{0};
  std::atomic<uint64_t> max_bytes_per_sec{0};
  std::atomic<uint64_t> min_bytes_per_sec{std::numeric_limits<uint64_t>::max()};

  void record(uint64_t bytes, uint64_t duration_us)
  {
    total_bytes.fetch_add(bytes, std::memory_order_relaxed);
    total_time_us.fetch_add(duration_us, std::memory_order_relaxed);
    operation_count.fetch_add(1, std::memory_order_relaxed);

    if (duration_us > 0)
    {
      uint64_t bps = bytes * 1000000ULL / duration_us;
      // Update max via compare-exchange loop
      uint64_t current_max = max_bytes_per_sec.load(std::memory_order_relaxed);
      while (bps > current_max)
      {
        if (max_bytes_per_sec.compare_exchange_weak(current_max, bps, std::memory_order_relaxed))
          break;
      }
      // Update min via compare-exchange loop
      uint64_t current_min = min_bytes_per_sec.load(std::memory_order_relaxed);
      while (bps < current_min)
      {
        if (min_bytes_per_sec.compare_exchange_weak(current_min, bps, std::memory_order_relaxed))
          break;
      }
    }
  }

  double avg_mbps() const
  {
    uint64_t t = total_time_us.load(std::memory_order_relaxed);
    if (t == 0)
      return 0.0;
    return double(total_bytes.load(std::memory_order_relaxed)) / double(t); // bytes/us = MB/s
  }

  double peak_mbps() const
  {
    return double(max_bytes_per_sec.load(std::memory_order_relaxed)) / 1e6;
  }

  double low_mbps() const
  {
    uint64_t v = min_bytes_per_sec.load(std::memory_order_relaxed);
    if (v == std::numeric_limits<uint64_t>::max())
      return 0.0;
    return double(v) / 1e6;
  }
};

struct perf_stats_t
{
  using clock_t = std::chrono::steady_clock;
  using time_point_t = clock_t::time_point;

  time_point_t conversion_start;
  time_point_t conversion_end;

  io_counter_t source_read;
  io_counter_t sort;
  io_counter_t source_write;
  io_counter_t lod_read;
  io_counter_t lod_write;

  std::atomic<uint64_t> tree_build_time_us{0};
  std::atomic<uint64_t> lod_generation_time_us{0};
  time_point_t lod_start;
  // Leaf collapse, timed apart from LOD. The two ran under one clock until it turned out that the
  // "LOD" duration had collapse folded into it; they are different algorithms with different costs
  // and only one of them was instrumented.
  std::atomic<uint64_t> collapse_time_us{0};
  time_point_t collapse_start;
  std::atomic<bool> lod_phase{false};

  std::atomic<uint64_t> cache_hits{0};
  std::atomic<uint64_t> cache_misses{0};

  double total_time_seconds() const
  {
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(conversion_end - conversion_start).count();
    return double(dur) / 1e6;
  }

  double total_bytes_written_mb() const
  {
    uint64_t total = source_write.total_bytes.load(std::memory_order_relaxed) + lod_write.total_bytes.load(std::memory_order_relaxed);
    return double(total) / 1e6;
  }

  // Binary serialization: version(1) + 5*io_counter(40 each) + tree_build_us(8) + lod_gen_us(8) + total_time_us(8) + cache_hits(8) + cache_misses(8)
  static constexpr uint32_t serialized_size = 1 + 5 * 40 + 8 + 8 + 8 + 8 + 8;

  std::unique_ptr<uint8_t[]> serialize(uint32_t &out_size) const
  {
    out_size = serialized_size;
    auto buf = std::make_unique<uint8_t[]>(out_size);
    uint8_t *p = buf.get();

    *p++ = 1; // version

    auto write_counter = [&p](const io_counter_t &c)
    {
      uint64_t v;
      v = c.total_bytes.load(std::memory_order_relaxed);
      memcpy(p, &v, 8); p += 8;
      v = c.total_time_us.load(std::memory_order_relaxed);
      memcpy(p, &v, 8); p += 8;
      uint32_t op = c.operation_count.load(std::memory_order_relaxed);
      memcpy(p, &op, 4); p += 4;
      // pad to align
      uint32_t pad = 0;
      memcpy(p, &pad, 4); p += 4;
      v = c.max_bytes_per_sec.load(std::memory_order_relaxed);
      memcpy(p, &v, 8); p += 8;
      v = c.min_bytes_per_sec.load(std::memory_order_relaxed);
      memcpy(p, &v, 8); p += 8;
    };

    write_counter(source_read);
    write_counter(sort);
    write_counter(source_write);
    write_counter(lod_read);
    write_counter(lod_write);

    uint64_t v;
    v = tree_build_time_us.load(std::memory_order_relaxed);
    memcpy(p, &v, 8); p += 8;
    v = lod_generation_time_us.load(std::memory_order_relaxed);
    memcpy(p, &v, 8); p += 8;

    auto total_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(conversion_end - conversion_start).count());
    memcpy(p, &total_us, 8); p += 8;

    v = cache_hits.load(std::memory_order_relaxed);
    memcpy(p, &v, 8); p += 8;
    v = cache_misses.load(std::memory_order_relaxed);
    memcpy(p, &v, 8); p += 8;

    return buf;
  }

  struct deserialized_perf_stats_t
  {
    uint64_t total_time_us;
    double total_time_seconds;

    struct counter_data_t
    {
      uint64_t total_bytes;
      uint64_t total_time_us;
      uint32_t operation_count;
      uint64_t max_bytes_per_sec;
      uint64_t min_bytes_per_sec;

      double avg_mbps() const
      {
        if (total_time_us == 0)
          return 0.0;
        return double(total_bytes) / double(total_time_us);
      }
      double peak_mbps() const { return double(max_bytes_per_sec) / 1e6; }
      double low_mbps() const
      {
        if (min_bytes_per_sec == std::numeric_limits<uint64_t>::max())
          return 0.0;
        return double(min_bytes_per_sec) / 1e6;
      }
    };

    counter_data_t source_read;
    counter_data_t sort;
    counter_data_t source_write;
    counter_data_t lod_read;
    counter_data_t lod_write;
    uint64_t tree_build_us;
    uint64_t lod_generation_us;
    uint64_t cache_hits;
    uint64_t cache_misses;
    bool valid;
  };

  static deserialized_perf_stats_t deserialize(const uint8_t *data, uint32_t size)
  {
    deserialized_perf_stats_t result{};
    result.valid = false;

    static constexpr uint32_t min_size = 1 + 5 * 40 + 8 + 8 + 8; // v1 without cache fields
    if (!data || size < min_size)
      return result;

    const uint8_t *p = data;
    uint8_t version = *p++;
    if (version != 1)
      return result;

    auto read_counter = [&p]() -> deserialized_perf_stats_t::counter_data_t
    {
      deserialized_perf_stats_t::counter_data_t c{};
      memcpy(&c.total_bytes, p, 8); p += 8;
      memcpy(&c.total_time_us, p, 8); p += 8;
      memcpy(&c.operation_count, p, 4); p += 4;
      p += 4; // skip pad
      memcpy(&c.max_bytes_per_sec, p, 8); p += 8;
      memcpy(&c.min_bytes_per_sec, p, 8); p += 8;
      return c;
    };

    result.source_read = read_counter();
    result.sort = read_counter();
    result.source_write = read_counter();
    result.lod_read = read_counter();
    result.lod_write = read_counter();

    memcpy(&result.tree_build_us, p, 8); p += 8;
    memcpy(&result.lod_generation_us, p, 8); p += 8;
    memcpy(&result.total_time_us, p, 8); p += 8;

    if (size >= serialized_size)
    {
      memcpy(&result.cache_hits, p, 8); p += 8;
      memcpy(&result.cache_misses, p, 8); p += 8;
    }

    result.total_time_seconds = double(result.total_time_us) / 1e6;
    result.valid = true;
    return result;
  }
};

} // namespace dew::core
