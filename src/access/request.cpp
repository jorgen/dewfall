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

#include "dataset_impl.hpp"

#include "compressor.hpp"
#include "format_util.hpp"

#include <algorithm>
#include <cstring>
#include <future>

namespace dew::access
{

void request_impl_t::cancel()
{
  auto expected = dew_request_pending;
  if (status.compare_exchange_strong(expected, dew_request_canceled))
    wait_cond.notify_all();
}

void request_impl_t::finish(dew_request_status_t terminal)
{
  auto expected = dew_request_pending;
  // A cancel that got in first wins: it is already terminal and must stay that way.
  status.compare_exchange_strong(expected, terminal);
  {
    std::unique_lock<std::mutex> lock(wait_mutex);
  }
  wait_cond.notify_all();
}

namespace
{

position_format_t to_internal(dew_position_format_t f)
{
  switch (f)
  {
  case dew_position_r32_relative:
    return position_format_t::r32_relative;
  case dew_position_i32_grid:
    return position_format_t::i32_grid;
  case dew_position_r64_absolute:
  default:
    return position_format_t::r64_absolute;
  }
}

} // namespace

// Execute a region request end to end: walk to a converged node set, then for each node read the
// position blob plus each requested attribute, decode, optionally clip, and append to the
// concatenated output buffers.
//
// Runs as a coroutine on the dataset's own loop, so the caller's thread is never blocked. Reads are
// still issued one at a time -- overlapping them is the next step, and this one only has to prove
// the suspend/resume path against results that must not change.
vio::task_t<bool> run_region_request(dataset_impl_t &dataset, const region_job_t &spec, request_impl_t &request)
{
  region_query_t query;
  for (int i = 0; i < 3; i++)
  {
    query.box.min[i] = spec.box_min[i];
    query.box.max[i] = spec.box_max[i];
  }
  query.whole_dataset = (spec.box_min[0] >= spec.box_max[0] && spec.box_min[1] >= spec.box_max[1] && spec.box_min[2] >= spec.box_max[2]);
  switch (spec.lod_mode)
  {
  case dew_lod_level:
    query.lod_mode = lod_mode_t::level;
    query.lod = spec.lod;
    break;
  case dew_lod_point_budget:
    query.lod_mode = lod_mode_t::point_budget;
    query.max_points = spec.max_points;
    break;
  case dew_lod_full:
  default:
    query.lod_mode = lod_mode_t::full;
    break;
  }

  const auto &names = spec.attribute_names;
  const uint32_t attribute_count = uint32_t(names.size());

  region_result_t walked;
  if (!co_await dataset.co_walk_to_convergence(query, walked))
  {
    request.error = dataset.error.code ? dataset.error : dew_error_t{1, "region walk failed"};
    co_return false;
  }

  const auto position_format = to_internal(spec.position_format);
  const uint32_t position_stride_bytes = position_stride(position_format);

  // Buffer 0 is always the positions; requested attributes follow in the order given.
  request.buffers.clear();
  request.buffers.resize(size_t(attribute_count) + 1);
  auto &positions = request.buffers[0];
  positions.name = "xyz";
  positions.stride = position_stride_bytes;
  switch (position_format)
  {
  case position_format_t::r64_absolute:
    positions.type = dew_type_r64;
    break;
  case position_format_t::r32_relative:
    positions.type = dew_type_r32;
    break;
  case position_format_t::i32_grid:
    positions.type = dew_type_i32;
    break;
  }
  positions.components = dew_components_3;

  // Resolve each attribute's format ACROSS ALL selected nodes before appending anything.
  //
  // Nodes do not all carry the same attribute set -- slimmed LOD nodes drop the non-visual ones --
  // so a node that lacks an attribute has to contribute zeros to keep every buffer index-aligned
  // with the positions. Discovering the stride lazily from the first node that happens to have the
  // attribute breaks that: any earlier node contributes nothing at all, and the attribute array ends
  // up shorter than xyz and silently misaligned against it.
  for (uint32_t a = 0; a < attribute_count; a++)
  {
    auto &out = request.buffers[a + 1];
    out.name = names[a];
    for (const auto &node : walked.nodes)
    {
      const auto index = dataset.attributes.get_attribute_index(node.attributes_id, names[a]);
      if (index.index < 0)
        continue;
      out.type = index.format.type;
      out.components = index.format.components;
      out.stride = uint32_t(size_for_format(index.format.type, index.format.components));
      break;
    }
  }

  // What one node contributes, decoded off the dataset loop and appended in walk order afterwards.
  // Staging is not an optimisation: decoding straight into the shared concatenated buffers from
  // several pool threads would make the output order depend on thread scheduling.
  struct node_stage_t
  {
    bool valid = false;
    uint32_t kept = 0;
    double origin[3] = {0, 0, 0};
    std::vector<uint8_t> positions;
    std::vector<std::vector<uint8_t>> attributes;
    const region_node_t *node = nullptr;
    dew_error_t error;
  };

  // One node's reads, issued but not yet awaited.
  struct pending_node_t
  {
    const region_node_t *node = nullptr;
    std::shared_ptr<read_request_t> position;
    std::vector<std::shared_ptr<read_request_t>> attributes; // null where the node lacks it
  };

  auto &loop = dataset.loop_thread.event_loop();
  // max_reads_in_flight is a TARGET, not a hard cap: a node's position blob and its attribute blobs
  // are issued as a unit, so the floor is one node's worth (1 + attribute_count) even when the
  // budget is smaller. Splitting a node across batches would buy nothing -- it cannot be decoded
  // until all of its blobs have landed anyway.
  const uint32_t reads_per_node = 1 + attribute_count;
  const uint32_t batch_nodes = std::max<uint32_t>(1, dataset.max_reads_in_flight / std::max<uint32_t>(1, reads_per_node));

  for (size_t begin = 0; begin < walked.nodes.size(); begin += batch_nodes)
  {
    if (request.status.load(std::memory_order_acquire) == dew_request_canceled)
      co_return false;
    const size_t end = std::min(begin + batch_nodes, walked.nodes.size());

    // ---- issue: every read in the batch goes out before any of them is awaited, which is what
    // turns per-blob latency into one batch's worth instead of the sum. read() only queues.
    std::vector<pending_node_t> pending;
    pending.reserve(end - begin);
    for (size_t i = begin; i < end; i++)
    {
      const auto &node = walked.nodes[i];
      if (node.point_count.data == 0)
        continue;
      const tree_t *tree = dataset.registry().get(node.tree_id);
      if (!tree)
        continue;

      pending_node_t entry;
      entry.node = &node;
      // Slot 0 of a storage unit is a storage_header_t followed by the morton codes.
      const auto position_location = tree->storage_map.location(node.input_id, 0);
      if (position_location.size == 0)
        continue; // absent slot; offset == 0 is a VALID location, so never test that
      entry.position = dataset.reader->read(position_location, read_options_t{false, true, {}});

      entry.attributes.resize(attribute_count);
      for (uint32_t a = 0; a < attribute_count; a++)
      {
        if (request.buffers[a + 1].stride == 0)
          continue; // no node has it at all
        const auto index = dataset.attributes.get_attribute_index(node.attributes_id, names[a]);
        if (index.index < 0)
          continue; // this node lacks it: contributes zeros
        const auto location = tree->storage_map.location(node.input_id, index.index);
        if (location.size == 0)
          continue;
        entry.attributes[a] = dataset.reader->read(location, read_options_t{false, true, {}});
      }
      pending.push_back(std::move(entry));
    }

    // ---- await: they were all issued together, so the later ones are usually already done.
    for (auto &entry : pending)
    {
      co_await entry.position->await_on(loop);
      for (auto &attribute : entry.attributes)
      {
        if (attribute)
          co_await attribute->await_on(loop);
      }
    }

    // ---- decode: pure CPU, so hop it to the pool. Under wasm the pool has no workers and runs the
    // job inline, which must be equally correct.
    std::vector<node_stage_t> stages(pending.size());
    std::vector<std::future<void>> jobs;
    jobs.reserve(pending.size());
    for (size_t i = 0; i < pending.size(); i++)
    {
      auto *entry = &pending[i];
      auto *stage = &stages[i];
      jobs.push_back(dataset.pool.enqueue([entry, stage, &dataset, &request, &spec, position_format, position_stride_bytes, attribute_count, &query]() {
        stage->node = entry->node;
        if (entry->position->error.code != 0)
        {
          stage->error = entry->position->error;
          return;
        }
        storage_header_t header;
        dew_blob_t point_data;
        dew_error_t split_error;
        if (!deserialize_points(entry->position->buffer_info, header, point_data, split_error))
        {
          stage->error = split_error;
          return;
        }
        const uint32_t offset = entry->node->offset_in_subset.data;
        const uint32_t count = entry->node->point_count.data;
        if (uint64_t(offset) + count > header.point_count)
          return; // subset does not fit the stored unit; skip rather than read out of bounds

        const uint32_t src_stride = uint32_t(size_for_format(header.point_format.type, header.point_format.components));
        const auto *src = static_cast<const uint8_t *>(point_data.data) + uint64_t(offset) * src_stride;

        stage->positions.resize(size_t(count) * position_stride_bytes);
        if (!decode_positions(src, count * src_stride, count, header.point_format, header.morton_min, header.lod_span, dataset.registry().tree_config, position_format, stage->positions.data(),
                              uint64_t(count) * position_stride_bytes, stage->origin))
        {
          // Named, because "failed to decode node positions" with no context is unactionable on a
          // dataset with tens of thousands of nodes. The only way this fires is an unrecognised
          // point_format in the deserialised header -- i.e. slot 0 did not read back as a positions
          // blob -- so say which node and what was actually found there.
          stage->error = {1, fmt::format("failed to decode positions for node tree {} level {} index {}: point_format type {} components {}, {} points, subset offset {} count {}",
                                         stage->node->tree_id.data, stage->node->level, stage->node->index, int(header.point_format.type), int(header.point_format.components), header.point_count, offset,
                                         count)};
          return;
        }

        stage->attributes.resize(attribute_count);
        std::vector<attribute_span_t> spans(attribute_count, attribute_span_t{nullptr, 0});
        for (uint32_t a = 0; a < attribute_count; a++)
        {
          const uint32_t stride = request.buffers[a + 1].stride;
          if (stride == 0)
            continue;
          // Zero-filled by default, so a node lacking the attribute still contributes its full share
          // and every buffer stays aligned with the positions.
          stage->attributes[a].assign(size_t(count) * stride, uint8_t(0));
          auto &source = entry->attributes[a];
          if (source && source->error.code == 0 && uint64_t(offset + count) * stride <= source->buffer_info.size)
            memcpy(stage->attributes[a].data(), static_cast<const uint8_t *>(source->buffer_info.data) + uint64_t(offset) * stride, size_t(count) * stride);
          spans[a] = attribute_span_t{stage->attributes[a].data(), stride};
        }

        uint32_t kept = count;
        if (spec.clip_mode == dew_clip_point && !query.whole_dataset && !entry->node->fully_inside)
        {
          kept = clip_to_box(stage->positions.data(), position_format, stage->origin, dataset.registry().tree_config.scale, count, spec.box_min, spec.box_max, spans.data(), attribute_count);
          stage->positions.resize(size_t(kept) * position_stride_bytes);
          for (uint32_t a = 0; a < attribute_count; a++)
          {
            const uint32_t stride = request.buffers[a + 1].stride;
            if (stride)
              stage->attributes[a].resize(size_t(kept) * stride);
          }
        }
        stage->kept = kept;
        stage->valid = true;
      }));
    }
    for (auto &job : jobs)
      job.get();

    // ---- append in WALK ORDER, on this loop. Output is therefore identical no matter how many
    // decode threads ran, which is what makes the result reproducible.
    for (auto &stage : stages)
    {
      if (stage.error.code != 0)
      {
        request.error = stage.error;
        co_return false;
      }
      if (!stage.valid || stage.kept == 0)
        continue;

      positions.data.insert(positions.data.end(), stage.positions.begin(), stage.positions.end());
      for (uint32_t a = 0; a < attribute_count; a++)
      {
        auto &out = request.buffers[a + 1];
        if (out.stride)
          out.data.insert(out.data.end(), stage.attributes[a].begin(), stage.attributes[a].end());
      }

      dew_result_node_t result_node{};
      result_node.tree_id = stage.node->tree_id.data;
      result_node.level = stage.node->level;
      result_node.index = stage.node->index;
      result_node.lod = stage.node->lod;
      result_node.first_point = request.point_count;
      result_node.point_count = stage.kept;
      for (int i = 0; i < 3; i++)
        result_node.position_offset[i] = stage.origin[i];
      result_node.is_leaf = stage.node->is_leaf ? 1 : 0;
      result_node.is_lod = stage.node->is_lod ? 1 : 0;
      request.nodes.push_back(result_node);
      request.point_count += stage.kept;
    }
  }

  co_return true;
}

// Spawn the request on the dataset's own loop and return at once.
//
// The job and the request shared_ptr are passed BY VALUE into the coroutine, never captured by a
// coroutine lambda: a lambda's captures live in its closure, which is destroyed after the first
// suspension, so anything captured would dangle on resume. object_backend.cpp and dew_wasm_api.cpp
// both carry the same note.
void dataset_impl_t::spawn_region_request(region_job_t job, std::shared_ptr<dew_request_t> request)
{
  auto *dataset = this;
  loop_thread.event_loop().run_in_loop([dataset, job = std::move(job), request]() mutable {
    [](dataset_impl_t *ds, region_job_t j, std::shared_ptr<dew_request_t> r) -> vio::detached_task_t {
      const bool ok = co_await run_region_request(*ds, j, *r);
      r->finish(ok ? dew_request_completed : dew_request_failed);
      // Queue for delivery and raise the wake. The callback itself runs later, on the host thread.
      ds->publish(r);
    }(dataset, std::move(job), request);
  });
}

} // namespace dew::access
