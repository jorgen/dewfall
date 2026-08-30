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
#include "converter.hpp"

#include <cstdio>
#include <dew/converter/converter.h>

#include "compressor.hpp"
#include "perf_stats.hpp"
#include "processor.hpp"

#include <vio/objstore/create_object_store.h>

#include <memory>
#include <string>
#include <string_view>
#include <filesystem>
#include <vector>

using namespace dew::converter;
using namespace dew::core;

struct dew_converter_t *dew_converter_create(const char *url, uint64_t url_size, enum dew_converter_open_file_semantics_t semantics, dew_error_t **error)
{
  auto *converter = new dew_converter_t(url, url_size, semantics);
  if (converter->error.code != 0)
  {
    if (error)
    {
      *error = new dew_error_t();
      (*error)->code = converter->error.code;
      (*error)->msg = converter->error.msg;
    }
    delete converter;
    return nullptr;
  }
  return converter;
}

struct dew_converter_t *dew_converter_create_with_connection(const char *url, uint64_t url_size, const char *connection, uint64_t connection_size, enum dew_converter_open_file_semantics_t semantics, dew_error_t **error)
{
  // Cloud destinations route through an IMPLICIT local cache + incremental DEW2 upload: one
  // conversion pipeline (local IO for LOD reads, resumable, band uploads) instead of the retired
  // direct object-per-blob write mode. The cache lives in $DEW_CACHE_DIR (or the OS cache dir)
  // named by a hash of the destination URL, capped via $DEW_CACHE_MAX_BYTES (0/unset = uncapped).
  // Local URLs (file:// / bare paths / dir:// / mem://) keep the classic single-store behavior.
  {
    std::string url_string(url, url_size);
    auto scheme_end = url_string.find("://");
    if (scheme_end != std::string::npos)
    {
      auto scheme = url_string.substr(0, scheme_end);
      if (scheme == "s3" || scheme == "az" || scheme == "azure" || scheme == "http" || scheme == "https")
      {
        std::string cache_dir;
        if (const char *env_dir = std::getenv("DEW_CACHE_DIR"))
          cache_dir = env_dir;
        else if (const char *home = std::getenv("HOME"))
#ifdef __APPLE__
          cache_dir = std::string(home) + "/Library/Caches/points";
#else
          cache_dir = std::string(home) + "/.cache/points";
#endif
        else
          cache_dir = ".points-cache";
        std::error_code ec;
        std::filesystem::create_directories(cache_dir, ec);
        // Stable cache identity per destination URL (fnv1a); resuming the same destination reuses
        // the same cache file, and the embedded uuid guards against mixups.
        uint64_t hash = 1469598103934665603ull;
        for (char c : url_string)
          hash = (hash ^ uint8_t(c)) * 1099511628211ull;
        char name[32];
        snprintf(name, sizeof(name), "%016llx.dew", static_cast<unsigned long long>(hash));
        std::string cache_path = cache_dir + "/" + name;
        auto *converter = dew_converter_create_with_destination(cache_path.c_str(), cache_path.size(), url, url_size, connection, connection_size, semantics, error);
        if (converter)
        {
          if (const char *env_cap = std::getenv("DEW_CACHE_MAX_BYTES"))
            dew_converter_set_cache_max_bytes(converter, strtoull(env_cap, nullptr, 10));
        }
        return converter;
      }
    }
  }

  // Install the connection string (credentials/endpoint/region) for the output URL's provider before the
  // storage backend is created inside dew_converter_create. A no-op for local (file/dir/mem) URLs.
  bool applied = false;
  if (connection && connection_size > 0)
  {
    auto result = vio::objstore::apply_connection_override(std::string(url, url_size), std::string_view(connection, connection_size));
    if (!result)
    {
      if (error)
      {
        *error = new dew_error_t();
        (*error)->code = result.error().code != 0 ? result.error().code : -1;
        (*error)->msg = result.error().msg;
      }
      return nullptr;
    }
    applied = true;
  }
  auto *converter = dew_converter_create(url, url_size, semantics, error);
  // The override was consumed when the backend was created (its bucket/prefix are baked in), so clear it
  // now; leaving it set would leak into a later create with a different URL in the same process.
  if (applied)
  {
    vio::objstore::clear_s3_config_override();
    vio::objstore::clear_azure_config_override();
  }
  return converter;
}

struct dew_converter_t *dew_converter_create_with_destination(const char *cache_path, uint64_t cache_path_size, const char *destination_url, uint64_t destination_url_size, const char *connection, uint64_t connection_size,
                                                                    enum dew_converter_open_file_semantics_t semantics, dew_error_t **error)
{
  if (!destination_url || destination_url_size == 0)
    return dew_converter_create(cache_path, cache_path_size, semantics, error);
  std::string cache(cache_path, cache_path_size);
  // The cache must be a local packed file: reject remote schemes early with a clear error.
  auto scheme_end = cache.find("://");
  if (scheme_end != std::string::npos && cache.compare(0, scheme_end, "file") != 0)
  {
    if (error)
    {
      *error = new dew_error_t();
      (*error)->code = 1;
      (*error)->msg = "cache_path must be a local file path (the destination is where the dataset uploads)";
    }
    return nullptr;
  }
  dew::converter::destination_config_t destination;
  destination.url.assign(destination_url, destination_url_size);
  if (connection && connection_size)
    destination.connection.assign(connection, connection_size);
  auto *converter = new dew_converter_t(cache_path, cache_path_size, semantics, destination);
  if (converter->error.code != 0)
  {
    if (error)
    {
      *error = new dew_error_t();
      (*error)->code = converter->error.code;
      (*error)->msg = converter->error.msg;
    }
    delete converter;
    return nullptr;
  }
  return converter;
}

void dew_converter_set_cache_max_bytes(dew_converter_t *converter, uint64_t max_bytes)
{
  converter->processor.set_cache_max_bytes(max_bytes);
}

void dew_converter_set_read_cache_bytes(dew_converter_t *converter, uint64_t max_bytes)
{
  converter->processor.storage_handler().set_read_cache_size(max_bytes);
}

void dew_converter_set_decompressed_cache_bytes(dew_converter_t *converter, uint64_t max_bytes)
{
  converter->processor.storage_handler().set_decompressed_cache_size(max_bytes);
}

// Write a machine-readable record of the run beside the dataset.
//
// Separate from dew info, which describes the DATASET. This describes the CONVERSION -- where the
// time went and how much redundant work happened -- so two runs can be diffed. It is JSON because
// the point is to compare runs mechanically, not to read one prettily.
//
// The counters that make it worth having are the ones an aggregate hides. `cache_hits` read 99.5% on
// a conversion that was spending most of its CPU re-inflating blobs, because a decoded-cache hit and
// a re-inflating hit were counted the same. read.decoded_hits vs read.recompressed_hits separates
// them, and decompress.cpu_seconds against wall time says how much of the machine that cost.
uint8_t dew_converter_write_stats(dew_converter_t *converter, const char *path, uint32_t path_size, dew_error_t **error)
{
  if (!converter || !path || !path_size)
    return 0;
  const std::string filename(path, path_size);
  auto &ps = converter->processor.perf_stats();

  const double wall = ps.total_time_seconds();
  const auto us = [](const std::atomic<uint64_t> &v) { return double(v.load(std::memory_order_relaxed)) / 1e6; };
  const auto n = [](const std::atomic<uint64_t> &v) { return v.load(std::memory_order_relaxed); };

  const uint64_t decoded = n(ps.read_decoded_hits);
  const uint64_t recompressed = n(ps.read_recompressed_hits);
  const uint64_t misses = n(ps.read_misses);
  const uint64_t reads = decoded + recompressed + misses;

  std::string out;
  out += "{\n";
  out += fmt::format("  \"wall_seconds\": {:.3f},\n", wall);
  out += "  \"phases\": {\n";
  out += fmt::format("    \"tree_build_seconds\": {:.3f},\n", us(ps.tree_build_time_us));
  out += fmt::format("    \"collapse_seconds\": {:.3f},\n", us(ps.collapse_time_us));
  out += fmt::format("    \"lod_seconds\": {:.3f}\n", us(ps.lod_generation_time_us));
  out += "  },\n";
  out += "  \"collapse\": {\n";
  out += fmt::format("    \"jobs\": {},\n", n(ps.collapse_jobs));
  out += fmt::format("    \"merge_entries\": {},\n", n(ps.collapse_merge_entries));
  out += fmt::format("    \"cpu_seconds\": {:.3f},\n", us(ps.collapse_worker_us));
  out += fmt::format("    \"read_cpu_seconds\": {:.3f},\n", us(ps.collapse_read_us));
  out += fmt::format("    \"merge_cpu_seconds\": {:.3f},\n", us(ps.collapse_merge_us));
  out += fmt::format("    \"attribute_cpu_seconds\": {:.3f}\n", us(ps.collapse_attribute_us));
  out += "  },\n";
  // LOD counters are reset each pass, so these describe the LAST pass rather than the run.
  out += "  \"lod_last_pass\": {\n";
  out += fmt::format("    \"workers\": {},\n", n(ps.lod_workers));
  out += fmt::format("    \"cpu_seconds\": {:.3f},\n", us(ps.lod_worker_us));
  out += fmt::format("    \"read_cpu_seconds\": {:.3f},\n", us(ps.lod_read_us));
  out += fmt::format("    \"sample_cpu_seconds\": {:.3f},\n", us(ps.lod_sample_us));
  out += fmt::format("    \"attribute_cpu_seconds\": {:.3f}\n", us(ps.lod_attribute_us));
  out += "  },\n";
  out += "  \"read\": {\n";
  out += fmt::format("    \"total\": {},\n", reads);
  out += fmt::format("    \"decoded_hits\": {},\n", decoded);
  out += fmt::format("    \"recompressed_hits\": {},\n", recompressed);
  out += fmt::format("    \"misses\": {},\n", misses);
  out += fmt::format("    \"decoded_hit_fraction\": {:.4f}\n", reads ? double(decoded) / double(reads) : 0.0);
  out += "  },\n";
  out += "  \"decompress\": {\n";
  out += fmt::format("    \"cpu_seconds\": {:.3f},\n", us(ps.decompress_us));
  out += fmt::format("    \"input_bytes\": {},\n", n(ps.decompress_input_bytes));
  // How much of the machine went into decompression. Above 1.0 means more than one core's worth;
  // it was near 10 on a 16-thread run before the decoded cache was sized.
  out += fmt::format("    \"cores\": {:.2f}\n", wall > 0.0 ? us(ps.decompress_us) / wall : 0.0);
  out += "  },\n";
  out += "  \"io\": {\n";
  const auto counter = [&](const char *name, const io_counter_t &c, bool last) {
    return fmt::format("    \"{}\": {{ \"bytes\": {}, \"seconds\": {:.3f}, \"mb_per_second\": {:.1f} }}{}\n", name, c.total_bytes.load(std::memory_order_relaxed),
                       double(c.total_time_us.load(std::memory_order_relaxed)) / 1e6, c.avg_mbps(), last ? "" : ",");
  };
  out += counter("source_read", ps.source_read, false);
  out += counter("sort", ps.sort, false);
  out += counter("source_write", ps.source_write, false);
  out += counter("lod_read", ps.lod_read, false);
  out += counter("lod_write", ps.lod_write, true);
  out += "  }\n";
  out += "}\n";

  FILE *f = fopen(filename.c_str(), "wb");
  if (!f)
  {
    if (error)
    {
      *error = dew_error_create();
      const auto msg = fmt::format("cannot write conversion stats to {}", filename);
      dew_error_set_info(*error, 1, msg.c_str(), msg.size());
    }
    return 0;
  }
  const bool ok = fwrite(out.data(), 1, out.size(), f) == out.size();
  fclose(f);
  return ok ? 1 : 0;
}

void dew_converter_set_upload_callbacks(dew_converter_t *converter, dew_converter_upload_callbacks_t callbacks, void *user_ptr)
{
  converter->processor.set_upload_callbacks(callbacks, user_ptr);
}

bool dew_converter_get_upload_state(dew_converter_t *converter, dew_converter_upload_state_t *state)
{
  memset(state, 0, sizeof(*state));
  auto upload = converter->processor.upload_stats();
  state->bytes_uploaded = upload.bytes_uploaded;
  state->bands_committed = upload.bands_committed;
  state->objects_written = upload.objects_written;
  state->upload_parked = upload.parked ? 1 : 0;
  state->destination_complete = upload.complete ? 1 : 0;
  dew::converter::storage_handler_t::cache_tier_stats_t cache = {};
  const bool have_cache = converter->processor.storage_handler().get_cache_tier_stats(cache);
  state->cache_resident_bytes = cache.resident_bytes;
  state->cache_max_bytes = cache.cap_bytes;
  state->cache_spilled_bytes = cache.spilled_bytes;
  return have_cache;
}

void dew_converter_wait_local_complete(dew_converter_t *converter)
{
  converter->processor.wait_local_complete();
}

void dew_converter_destroy(dew_converter_t *destroy)
{
  delete destroy;
}

void dew_converter_set_file_converter_callbacks(dew_converter_t *converter, dew_converter_file_convert_callbacks_t callbacks)
{
  converter->processor.set_converter_callbacks(callbacks);
}

void dew_converter_set_runtime_callbacks(dew_converter_t *converter, dew_converter_runtime_callbacks_t callbacks, void *user_ptr)
{
  converter->processor.set_runtime_callbacks(callbacks, user_ptr);
}

void dew_converter_set_compression(dew_converter_t *converter, enum dew_converter_compression_t compression)
{
  converter->processor.storage_handler().set_compressor(static_cast<compression_method_t>(compression));
}

void dew_converter_set_store_original_order(dew_converter_t *converter, bool store)
{
  // Peek: reading tree_config() would SEAL the configuration, silently pinning the default scale
  // before the first input file's native scale could be adopted.
  auto config = converter->processor.tree_config_peek();
  config.store_original_order = store;
  converter->processor.set_pre_init_tree_config(config);
}

void dew_converter_set_tree_scale(dew_converter_t *converter, double scale)
{
  if (scale > 0.0)
    converter->processor.set_tree_scale_override(scale);
}

void dew_converter_set_lod_all_attributes(dew_converter_t *converter, uint8_t all)
{
  auto config = converter->processor.tree_config_peek();
  config.lod_all_attributes = all ? 1 : 0;
  converter->processor.set_pre_init_tree_config(config);
}

void dew_converter_set_mutable(dew_converter_t *converter, uint8_t is_mutable)
{
  converter->processor.set_mutable(is_mutable != 0);
}

uint8_t dew_converter_is_mutable(dew_converter_t *converter)
{
  return converter->processor.is_mutable() ? 1 : 0;
}

void dew_converter_finalize(dew_converter_t *converter)
{
  converter->processor.finalize();
}

uint8_t dew_converter_add_attribute(dew_converter_t *converter, const char *name, uint32_t name_size, const char *key_attribute, uint32_t key_attribute_size, dew_type_t type, dew_components_t components)
{
  return converter->processor.declare_attribute(std::string(name, name_size), std::string(key_attribute, key_attribute_size), type, components) ? 1 : 0;
}

uint8_t dew_converter_add_data_for_attribute(dew_converter_t *converter, const char *name, uint32_t name_size, const uint64_t *keys, const void *values, uint64_t count)
{
  return converter->processor.add_attribute_data(std::string(name, name_size), keys, values, count) ? 1 : 0;
}

void dew_converter_commit_attributes(dew_converter_t *converter)
{
  converter->processor.commit_attributes();
}

void dew_converter_set_compression_level(dew_converter_t *converter, int level)
{
  converter->processor.storage_handler().set_compression_level(level);
}

void dew_converter_set_node_point_limit(dew_converter_t *converter, uint32_t points)
{
  converter->processor.set_pre_init_node_point_limit(points);
}

void dew_converter_set_read_chunk_bytes(dew_converter_t *converter, uint64_t bytes)
{
  converter->processor.set_pre_init_read_chunk_bytes(bytes);
}

void dew_converter_add_data_file(dew_converter_t *converter, dew_converter_str_buffer *buffers, uint32_t buffer_count)
{
  std::vector<std::pair<std::unique_ptr<char[]>, uint32_t>> input_files;
  input_files.reserve(buffer_count);
  for (uint32_t i = 0; i < buffer_count; i++)
  {
    input_files.emplace_back();
    auto &input_data_source = input_files.back();
    input_data_source.first.reset(new char[buffers[i].size + 1]);
    memcpy(input_data_source.first.get(), buffers[i].data, buffers[i].size);
    input_data_source.first.get()[buffers[i].size] = 0;
    input_data_source.second = buffers[i].size;
  }
  converter->processor.add_files(std::move(input_files));
}

void dew_converter_wait_idle(dew_converter_t *converter)
{
  converter->processor.wait_idle();
}

dew_converter_conversion_status_t dew_converter_status(dew_converter_t *converter)
{
  if (converter->processor.has_errors())
    return dew_conversion_status_error;
  if (!converter->processor.is_idle() || converter->processor.upload_active())
    return dew_conversion_status_in_progress;
  return dew_conversion_status_completed;
}

static void fill_converter_stats(const compression_stats_t &src, dew_converter_stats_t *dst)
{
  memset(dst, 0, sizeof(*dst));
  dst->input_file_count = src.input_file_count;
  dst->total_buffer_count = src.total_buffer_count;
  dst->lod_buffer_count = src.lod_buffer_count;
  dst->compression_method = static_cast<uint32_t>(src.method);
  dst->input_file_size_bytes = src.input_file_size_bytes;
  dst->attribute_count = static_cast<uint32_t>(std::min(src.per_attribute.size(), size_t(32)));
  for (uint32_t i = 0; i < dst->attribute_count; i++)
  {
    auto &s = src.per_attribute[i];
    auto &d = dst->attributes[i];
    auto name_len = std::min(s.name.size(), size_t(63));
    memcpy(d.name, s.name.data(), name_len);
    d.name[name_len] = '\0';
    d.type = s.format.type;
    d.components = s.format.components;
    d.buffer_count = s.buffer_count;
    d.uncompressed_bytes = s.uncompressed_bytes;
    d.compressed_bytes = s.compressed_bytes;
    d.min_value = s.min_value;    d.max_value = s.max_value;
    memcpy(d.path_counts, s.path_counts, sizeof(d.path_counts));
    d.lod_buffer_count = s.lod_buffer_count;
    d.lod_uncompressed_bytes = s.lod_uncompressed_bytes;
    d.lod_compressed_bytes = s.lod_compressed_bytes;
  }
}

bool dew_converter_get_compression_stats(struct dew_converter_t *converter, struct dew_converter_stats_t *stats)
{
  if (!converter)
    return false;
  auto &src = converter->processor.storage_handler().get_compression_stats();
  fill_converter_stats(src, stats);
  return true;
}

static void fill_live_io_stats(dew_converter_io_stats_t &dst, const io_counter_t &src)
{
  dst.total_bytes = src.total_bytes.load(std::memory_order_relaxed);
  dst.total_time_us = src.total_time_us.load(std::memory_order_relaxed);
  dst.operation_count = src.operation_count.load(std::memory_order_relaxed);
  dst.avg_mbps = src.avg_mbps();
  dst.peak_mbps = src.peak_mbps();
  dst.low_mbps = src.low_mbps();
}

static void fill_io_stats(dew_converter_io_stats_t &dst, const perf_stats_t::deserialized_perf_stats_t::counter_data_t &src)
{
  dst.total_bytes = src.total_bytes;
  dst.total_time_us = src.total_time_us;
  dst.operation_count = src.operation_count;
  dst.avg_mbps = src.avg_mbps();
  dst.peak_mbps = src.peak_mbps();
  dst.low_mbps = src.low_mbps();
}

bool dew_converter_get_perf_stats(dew_converter_t *converter, dew_converter_perf_stats_t *perf_stats)
{
  if (!converter)
    return false;
  auto &parsed = converter->processor.storage_handler().get_deserialized_perf_stats();
  if (!parsed.valid)
  {
    memset(perf_stats, 0, sizeof(*perf_stats));
    return false;
  }

  perf_stats->total_time_seconds = parsed.total_time_seconds;
  uint64_t written = parsed.source_write.total_bytes + parsed.lod_write.total_bytes;
  perf_stats->total_bytes_written_mb = double(written) / 1e6;
  perf_stats->overall_mbps = parsed.total_time_seconds > 0 ? perf_stats->total_bytes_written_mb / parsed.total_time_seconds : 0;
  fill_io_stats(perf_stats->source_read, parsed.source_read);
  fill_io_stats(perf_stats->sort, parsed.sort);
  fill_io_stats(perf_stats->source_write, parsed.source_write);
  fill_io_stats(perf_stats->lod_read, parsed.lod_read);
  fill_io_stats(perf_stats->lod_write, parsed.lod_write);
  perf_stats->tree_build_seconds = double(parsed.tree_build_us) / 1e6;
  perf_stats->lod_generation_seconds = double(parsed.lod_generation_us) / 1e6;
  perf_stats->cache_hits = parsed.cache_hits;
  perf_stats->cache_misses = parsed.cache_misses;
  return true;
}

bool dew_converter_get_live_perf_stats(struct dew_converter_t *converter, struct dew_converter_perf_stats_t *perf_stats)
{
  if (!converter)
    return false;
  auto &ps = converter->processor.perf_stats();
  auto now = perf_stats_t::clock_t::now();
  double elapsed = double(std::chrono::duration_cast<std::chrono::microseconds>(now - ps.conversion_start).count()) / 1e6;

  perf_stats->total_time_seconds = elapsed;
  uint64_t written = ps.source_write.total_bytes.load(std::memory_order_relaxed) + ps.lod_write.total_bytes.load(std::memory_order_relaxed);
  perf_stats->total_bytes_written_mb = double(written) / 1e6;
  perf_stats->overall_mbps = elapsed > 0 ? perf_stats->total_bytes_written_mb / elapsed : 0;

  fill_live_io_stats(perf_stats->source_read, ps.source_read);
  fill_live_io_stats(perf_stats->sort, ps.sort);
  fill_live_io_stats(perf_stats->source_write, ps.source_write);
  fill_live_io_stats(perf_stats->lod_read, ps.lod_read);
  fill_live_io_stats(perf_stats->lod_write, ps.lod_write);

  perf_stats->tree_build_seconds = double(ps.tree_build_time_us.load(std::memory_order_relaxed)) / 1e6;
  perf_stats->lod_generation_seconds = double(ps.lod_generation_time_us.load(std::memory_order_relaxed)) / 1e6;
  perf_stats->cache_hits = ps.cache_hits.load(std::memory_order_relaxed);
  perf_stats->cache_misses = ps.cache_misses.load(std::memory_order_relaxed);
  return true;
}

