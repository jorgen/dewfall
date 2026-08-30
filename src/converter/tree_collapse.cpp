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
#include "tree_collapse.hpp"

#include <chrono>

#include <fmt/format.h>

#include <atomic>

#include "attributes_configs.hpp"
#include "input_header.hpp"
#include "morton_tree_coordinate_transform.hpp"
#include "storage_handler.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace dew::converter
{
using namespace dew::core;

tree_collapse_runner_t::tree_collapse_runner_t(vio::event_loop_t &event_loop, vio::thread_pool_t &thread_pool, tree_registry_t &tree_registry, storage_handler_t &storage, attributes_configs_t &attributes_configs)
  : _event_loop(event_loop)
  , _thread_pool(thread_pool)
  , _tree_registry(tree_registry)
  , _storage(storage)
  , _attributes_configs(attributes_configs)
  , _worker_done(_event_loop, [this]() { handle_worker_done(); })
{
}

static input_data_id_t next_collapsed_id(tree_registry_t &tree_registry)
{
  input_data_id_t ret; // NOLINT(*-pro-type-member-init)
  static_assert(sizeof(ret) == sizeof(tree_registry.current_collapsed_node_id), "id/counter layout mismatch");
  memcpy(&ret, &tree_registry.current_collapsed_node_id, sizeof(ret));
  tree_registry.current_collapsed_node_id++;
  assert(input_data_id_is_collapsed_leaf(ret));
  return ret;
}

// A leaf already in collapsed shape needs no work: one subset spanning a whole unit that is this
// leaf's own (a collapsed unit, or a reader chunk covering exactly this leaf -- point counts from
// the registry's chunk table; a chunk without an entry is from a pre-v3 cache and gets rewritten).
static bool leaf_is_collapsed_shape(const tree_registry_t &tree_registry, const points_collection_t &collection)
{
  if (collection.data.size() != 1 || collection.data[0].offset.data != 0 || uint64_t(collection.data[0].count.data) != collection.point_count)
    return false;
  auto id = collection.data[0].input_id;
  if (input_data_id_is_collapsed_leaf(id))
    return true;
  auto refs = tree_registry.chunk_tree_refs.find(id);
  return refs != tree_registry.chunk_tree_refs.end() && refs->second.point_count == collection.data[0].count.data;
}

void tree_collapse_runner_t::collapse_for_pass(const morton::morton192_t &target, std::function<void()> on_done)
{
  assert(_jobs.empty() && "one collapse pass at a time");
  morton::morton192_t terminal;
  memset(&terminal, 0xFF, sizeof(terminal));
  const bool terminal_pass = !(target < terminal);

  for (auto &tree : _tree_registry.data)
  {
    if (!tree)
      continue; // not loaded; a later (or the terminal) pass picks it up, same as finality
    if (_tree_registry.tree_state[tree->id.data] != uint8_t(tree_state_t::building))
      continue;
    if (tree->leaves_collapsed)
      continue;
    if (!terminal_pass && !(tree->morton_max < target))
      continue; // still reachable by future input

    const size_t jobs_before = _jobs.size();
    for (int level = 0; level < 5; level++)
    {
      for (uint32_t skip = 0; skip < uint32_t(tree->nodes[level].size()); skip++)
      {
        auto &collection = tree->data[level][skip];
        if (tree->nodes[level][skip] != 0 || collection.point_count == 0)
          continue;
        if (leaf_is_collapsed_shape(_tree_registry, collection))
          continue;
        collapse_job_t job;
        job.tree_id = tree->id;
        job.level = level;
        job.node_index = skip;
        job.new_id = next_collapsed_id(_tree_registry);
        job.collection = collection;
        for (auto &subset : collection.data)
        {
          if (job.sources.contains(subset.input_id))
            continue;
          auto info = tree->storage_map.info(subset.input_id);
          job.sources[subset.input_id] = {info.first, std::move(info.second)};
        }
        _jobs.push_back(std::move(job));
      }
    }
    if (_jobs.size() == jobs_before)
    {
      // Every leaf already has collapsed shape (LOD-only tree, or exact-chunk leaves): flag it so
      // finality can proceed without a rewrite.
      tree->leaves_collapsed = true;
    }
  }

  if (std::getenv("DEW_DEBUG_CHAIN"))
    fprintf(stderr, "[collapse] pass target=%llx jobs=%zu\n", (unsigned long long)target.data[0], _jobs.size());
  if (_jobs.empty())
  {
    on_done();
    return;
  }

  // Chunk locality: neighbouring leaves slice the same chunks; keeping them adjacent maximizes
  // RAM-LRU hits on the decompressed source blobs.
  std::sort(_jobs.begin(), _jobs.end(), [](const collapse_job_t &a, const collapse_job_t &b) { return a.collection.min < b.collection.min; });

  _on_done = std::move(on_done);
  _completed.store(0, std::memory_order_relaxed);
  for (auto &job : _jobs)
  {
    auto *job_ptr = &job;
    _thread_pool.enqueue([this, job_ptr] { merge_worker(*job_ptr); });
  }
}

namespace
{
struct merge_entry_t
{
  morton::morton192_t absolute;
  uint32_t subset_index;
  uint32_t source_index; // absolute index within the source unit
};

template <typename S_M>
void append_absolute(const read_only_points_t &points, uint32_t subset_index, const points_subset_t &subset, std::vector<merge_entry_t> &out)
{
  const auto slice = morton_buffer_for_subset(points.data, points.header.point_format.type, subset.offset, subset.count);
  const auto *source = reinterpret_cast<const S_M *>(slice.data);
  for (uint32_t i = 0; i < subset.count.data; i++)
  {
    merge_entry_t entry;
    morton::morton_upcast(source[i], points.header.morton_min, entry.absolute);
    entry.subset_index = subset_index;
    entry.source_index = subset.offset.data + i;
    out.push_back(entry);
  }
}

template <typename D_M>
std::unique_ptr<uint8_t[]> make_destination_morton(const std::vector<merge_entry_t> &entries, uint32_t &size)
{
  size = uint32_t(entries.size() * sizeof(D_M));
  auto buffer = std::make_unique<uint8_t[]>(size);
  auto *out = reinterpret_cast<D_M *>(buffer.get());
  for (size_t i = 0; i < entries.size(); i++)
    morton::morton_downcast(entries[i].absolute, out[i]);
  return buffer;
}
} // namespace

namespace
{
// Leaf collapse, measured. It was entirely uninstrumented while its cost was being reported as part
// of LOD generation -- the pass clock started before collapse and only stopped after LOD, so every
// "LOD is 93% of the conversion" statement actually covered both. Set DEW_DEBUG_LOD to print this
// alongside the LOD probe; the counters are cumulative over the run.
struct collapse_probe_t
{
  std::atomic<uint64_t> jobs{0};
  std::atomic<uint64_t> read_us{0};    // reading every source subset's blob: blocking, on a pool thread
  std::atomic<uint64_t> merge_us{0};   // the k-way merge (a stable_sort over all subsets' entries)
  std::atomic<uint64_t> attrib_us{0};  // scattering source attributes into the merged unit
  std::atomic<uint64_t> total_us{0};
  std::atomic<uint64_t> entries{0};    // total merge entries, i.e. (chunk x leaf) subset incidences
};
collapse_probe_t g_collapse_probe;
const bool g_collapse_probe_on = std::getenv("DEW_DEBUG_LOD") != nullptr;

struct collapse_scoped_us_t
{
  std::atomic<uint64_t> &sink;
  std::chrono::steady_clock::time_point start;
  explicit collapse_scoped_us_t(std::atomic<uint64_t> &s)
    : sink(s)
    , start(std::chrono::steady_clock::now())
  {
  }
  ~collapse_scoped_us_t()
  {
    sink.fetch_add(uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count()), std::memory_order_relaxed);
  }
};
} // namespace

void tree_collapse_runner_t::merge_worker(collapse_job_t &job)
{
  const auto collapse_worker_start = std::chrono::steady_clock::now();
  auto finish = [this]() {
    _completed.fetch_add(1, std::memory_order_acq_rel);
    _worker_done.post_event();
  };

  // 1. Read every source unit's points blob (decompressed, RAM-LRU cached) and up-cast each
  //    subset's morton values to absolute 192-bit codes. Stored unit values are plain truncations
  //    of the absolute code (see morton_downcast/upcast), so this is lossless.
  std::vector<merge_entry_t> entries;
  entries.reserve(job.collection.point_count);
  std::vector<std::unique_ptr<read_only_points_t>> reads; // keep buffers alive over the loop
  for (uint32_t subset_index = 0; subset_index < uint32_t(job.collection.data.size()); subset_index++)
  {
    auto &subset = job.collection.data[subset_index];
    std::unique_ptr<read_only_points_t> points;
    {
      collapse_scoped_us_t timer(g_collapse_probe.read_us);
      points = std::make_unique<read_only_points_t>(_storage, job.sources.at(subset.input_id).locations[0]);
    }
    if (points->error.code != 0)
    {
      if (std::getenv("DEW_DEBUG_CHAIN"))
        fprintf(stderr, "[collapse] read failed id=%u.%u: %s\n", subset.input_id.data, subset.input_id.sub, points->error.msg.c_str());
      // The storage error pipe already flagged the conversion; the leaf keeps its subsets and the
      // tree stays building (retried by a later pass).
      job.failed = true;
      finish();
      return;
    }
    switch (points->header.point_format.type)
    {
    case dew_type_m32:
      append_absolute<morton::morton32_t>(*points, subset_index, subset, entries);
      break;
    case dew_type_m64:
      append_absolute<morton::morton64_t>(*points, subset_index, subset, entries);
      break;
    case dew_type_m128:
      append_absolute<morton::morton128_t>(*points, subset_index, subset, entries);
      break;
    case dew_type_m192:
      append_absolute<morton::morton192_t>(*points, subset_index, subset, entries);
      break;
    default:
      assert(false && "points blob must hold a morton type");
      job.failed = true;
      finish();
      return;
    }
    reads.push_back(std::move(points));
  }
  assert(entries.size() == job.collection.point_count);

  // 2. Merge: subsets are internally sorted but interleave across chunks. Stable sort keeps the
  //    subset order as the tie-break for equal codes (duplicate points).
  {
    collapse_scoped_us_t timer(g_collapse_probe.merge_us);
    std::stable_sort(entries.begin(), entries.end(), [](const merge_entry_t &a, const merge_entry_t &b) { return a.absolute < b.absolute; });
  }
  g_collapse_probe.entries.fetch_add(entries.size(), std::memory_order_relaxed);

  // 3. Destination format: the sorter's exact rule -- lod span of [min, max] picks the narrowest
  //    morton type whose truncation is lossless for every point in the unit.
  job.generated_min = entries.front().absolute;
  job.generated_max = entries.back().absolute;
  const int lod_span = morton::morton_lod(job.generated_min, job.generated_max);
  const dew_type_t destination_type = morton_type_from_lod(lod_span);
  uint32_t morton_buffer_size = 0;
  std::unique_ptr<uint8_t[]> morton_buffer;
  switch (destination_type)
  {
  case dew_type_m32:
    morton_buffer = make_destination_morton<morton::morton32_t>(entries, morton_buffer_size);
    break;
  case dew_type_m64:
    morton_buffer = make_destination_morton<morton::morton64_t>(entries, morton_buffer_size);
    break;
  case dew_type_m128:
    morton_buffer = make_destination_morton<morton::morton128_t>(entries, morton_buffer_size);
    break;
  default:
    morton_buffer = make_destination_morton<morton::morton192_t>(entries, morton_buffer_size);
    break;
  }

  // 4. Attributes: identity permutation in merge order; quantize_attributres scatters every source
  //    attribute blob into the destination union (original_order kept -- values travel with their
  //    points, chunk attribution is lost by design).
  std::vector<attributes_id_t> source_attribute_ids;
  source_attribute_ids.reserve(job.sources.size());
  for (auto &[unit_id, info] : job.sources)
    source_attribute_ids.push_back(info.attributes_id);
  std::sort(source_attribute_ids.begin(), source_attribute_ids.end(), [](attributes_id_t a, attributes_id_t b) { return a.data < b.data; });
  source_attribute_ids.erase(std::unique(source_attribute_ids.begin(), source_attribute_ids.end(), [](attributes_id_t a, attributes_id_t b) { return a.data == b.data; }), source_attribute_ids.end());
  auto mapping = _attributes_configs.get_lod_attribute_mapping(lod_span, source_attribute_ids.data(), source_attribute_ids.data() + source_attribute_ids.size(), /*keep_original_order=*/true);

  std::vector<std::pair<input_data_id_t, uint32_t>> indecies;
  indecies.reserve(entries.size());
  for (auto &entry : entries)
    indecies.emplace_back(job.collection.data[entry.subset_index].input_id, entry.source_index);

  attribute_buffers_t buffers;
  attribute_buffers_initialize(mapping.destination, buffers, uint32_t(indecies.size()), std::move(morton_buffer));
  {
    collapse_scoped_us_t timer(g_collapse_probe.attrib_us);
    quantize_attributres(_storage, job.sources, indecies, mapping, buffers);
  }
  attribute_buffers_adjust_buffers_to_size(mapping.destination, buffers, uint32_t(indecies.size()));

  // 5. Write the unit (compression + residency accounting in the storage handler).
  storage_header_t header;
  storage_header_initialize(header);
  header.input_id = job.new_id;
  header.point_count = uint32_t(indecies.size());
  header.morton_min = job.generated_min;
  header.morton_max = job.generated_max;
  header.lod_span = lod_span;
  header.point_format = {destination_type, dew_components_1};
  job.generated_attributes_id = mapping.destination_id;
  _storage.write(header, mapping.destination_id, std::move(buffers), [&job, finish](const storage_header_t &, attributes_id_t, std::vector<storage_location_t> locations, const dew_error_t &error) {
    if (error.code != 0)
    {
      if (std::getenv("DEW_DEBUG_CHAIN"))
        fprintf(stderr, "[collapse] write failed: %s\n", error.msg.c_str());
      job.failed = true;
    }
    else
      job.generated_locations = std::move(locations);
    finish();
  });
  g_collapse_probe.jobs.fetch_add(1, std::memory_order_relaxed);
  g_collapse_probe.total_us.fetch_add(uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - collapse_worker_start).count()), std::memory_order_relaxed);
  if (g_collapse_probe_on && (g_collapse_probe.jobs.load(std::memory_order_relaxed) % 500) == 0)
    fmt::print(stderr, "\n[collapse] {} jobs, {:.1f}s in-worker | read {:.1f}s  merge {:.1f}s  attributes {:.1f}s | {} merge entries\n", g_collapse_probe.jobs.load(std::memory_order_relaxed),
               double(g_collapse_probe.total_us.load(std::memory_order_relaxed)) / 1e6, double(g_collapse_probe.read_us.load(std::memory_order_relaxed)) / 1e6,
               double(g_collapse_probe.merge_us.load(std::memory_order_relaxed)) / 1e6, double(g_collapse_probe.attrib_us.load(std::memory_order_relaxed)) / 1e6,
               g_collapse_probe.entries.load(std::memory_order_relaxed));
}

void tree_collapse_runner_t::handle_worker_done()
{
  // Void-pipe posts may coalesce, so completion is judged by the counter, not by wake count.
  // Stale wakes after apply see an empty job list and return.
  if (_jobs.empty())
    return;
  if (_completed.load(std::memory_order_acquire) < _jobs.size())
    return;
  apply_results();
}

void tree_collapse_runner_t::apply_results()
{
  for (auto &job : _jobs)
  {
    auto *tree = _tree_registry.get(job.tree_id);
    assert(tree && "collapsed trees stay loaded for the duration of the pass");
    if (std::getenv("DEW_DEBUG_CHAIN"))
      fprintf(stderr, "[collapse] apply tree=%u level=%d idx=%u failed=%d locations=%zu\n", job.tree_id.data, job.level, job.node_index, int(job.failed), job.generated_locations.size());
    if (job.failed || job.generated_locations.empty())
      continue; // leaf keeps its subsets; the tree stays building and is retried next pass

    tree->is_dirty = true;
    auto &collection = tree->data[job.level][job.node_index];
    assert(collection.point_count == job.collection.point_count && "final-able leaves are immutable during the pass");

    // Release the old subset references. When a reader chunk's last TREE drops it, its blobs ride
    // the checkpoint freed list (restore_discarded -> take_discarded -> post-commit unregister /
    // punch / spill deref).
    for (auto &subset : collection.data)
    {
      const auto id = subset.input_id;
      if (!tree->storage_map.contains(id))
        continue; // second subset of a unit already fully dereferenced below
      const bool had = tree->storage_map.contains(id);
      auto attrib_locations = tree->storage_map.dereference(id);
      const bool erased = had && !tree->storage_map.contains(id);
      if (!erased || !input_data_id_is_leaf(id) || input_data_id_is_collapsed_leaf(id))
        continue;
      auto refs = _tree_registry.chunk_tree_refs.find(id);
      if (refs == _tree_registry.chunk_tree_refs.end())
        continue; // pre-v3 cache: chunk lifetime unknown, never freed
      assert(refs->second.tree_count > 0);
      if (--refs->second.tree_count == 0)
      {
        // The chunk's write was counted in the compression stats; it is dead now -- uncount it so a
        // fresh conversion doesn't report its source data twice (chunk + collapsed unit).
        _storage.note_source_blobs_freed(attrib_locations.first, attrib_locations.second, refs->second.point_count);
        tree->storage_map.restore_discarded(std::move(attrib_locations.second));
        _tree_registry.chunk_tree_refs.erase(refs);
      }
    }

    tree->storage_map.add_storage(job.new_id, job.generated_attributes_id, std::move(job.generated_locations));
    collection.data.clear();
    collection.data.emplace_back(job.new_id, offset_in_subset_t(0), point_count_t(uint32_t(job.collection.point_count)));
    collection.min = job.generated_min;
    collection.max = job.generated_max;
    collection.min_lod = morton::morton_lod(collection.min, collection.max);
    job.applied = true;
  }

  // A tree is collapsed when every leaf now has collapsed shape (failed jobs leave it building).
  ankerl::unordered_dense::map<uint32_t, bool, ankerl::unordered_dense::hash<uint32_t>> touched;
  for (auto &job : _jobs)
    touched[job.tree_id.data] = touched[job.tree_id.data] || !job.applied;
  for (auto &[tree_id, any_failed] : touched)
  {
    if (std::getenv("DEW_DEBUG_CHAIN"))
      fprintf(stderr, "[collapse] tree=%u failed=%d\n", tree_id, int(any_failed));
    if (!any_failed)
      _tree_registry.get(tree_id_t(tree_id))->leaves_collapsed = true;
  }

  _jobs.clear();
  auto on_done = std::move(_on_done);
  _on_done = {};
  on_done();
}

} // namespace dew::converter
