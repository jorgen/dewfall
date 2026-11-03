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

// The public C surface. Every entry point is a thin shell over dataset_impl_t / request_impl_t so
// that the ownership and threading rules documented in query.h are enforced in exactly one place.

#include "dataset_impl.hpp"

#include <chrono>
#include <cstring>

using namespace dew::access;

namespace
{
void fill_error(dew_error_t **out, const dew_error_t &src)
{
  if (!out)
    return;
  *out = new dew_error_t(src);
}
} // namespace

struct dew_dataset_t *dew_dataset_create(const char *url, uint32_t url_len, const char *connection, uint32_t connection_len, const struct dew_dataset_options_t *options, struct dew_pump_t *pump, struct dew_error_t **error)
{
  if (!url || url_len == 0)
  {
    fill_error(error, {1, "no dataset url given"});
    return nullptr;
  }
  dew_dataset_options_t defaults{};
  auto *dataset = new dew_dataset_t(std::string(url, url_len), connection && connection_len ? std::string(connection, connection_len) : std::string(), options ? *options : defaults, pump);
  return dataset;
}

void dew_dataset_close(struct dew_dataset_t *dataset)
{
  delete dataset;
}

enum dew_dataset_state_t dew_dataset_state(struct dew_dataset_t *dataset)
{
  return dataset ? dataset->state.load(std::memory_order_acquire) : dew_dataset_error;
}

void dew_dataset_get_error(struct dew_dataset_t *dataset, struct dew_error_t **error)
{
  if (dataset)
    fill_error(error, dataset->error);
}

uint32_t dew_dataset_poll(struct dew_dataset_t *dataset)
{
  if (!dataset)
    return 0;
  // Only meaningful for a private pump. Draining one subsystem of a SHARED pump is precisely the
  // partial drain that loses wakeups, so refuse rather than half-work.
  if (!dataset->owns_pump)
    return 0;
  return dew_pump_poll(dataset->pump);
}

uint32_t dew_dataset_pending_count(struct dew_dataset_t *dataset)
{
  if (!dataset)
    return 0;
  return dew_pump_pending_count(dataset->pump);
}

enum dew_dataset_state_t dew_dataset_wait_ready(struct dew_dataset_t *dataset, int32_t timeout_ms)
{
  if (!dataset)
    return dew_dataset_error;
  // announced_state, not state: it only moves once set_state has fired the pump (see set_state).
  auto settled = [dataset] { return dataset->announced_state != dew_dataset_opening; };
  std::unique_lock<std::mutex> lock(dataset->state_mutex);
  if (timeout_ms < 0)
    dataset->state_cond.wait(lock, settled);
  else
    dataset->state_cond.wait_for(lock, std::chrono::milliseconds(timeout_ms), settled);
  // Still `opening` means the wait timed out, which is distinct from having failed to open.
  return dataset->announced_state;
}

uint8_t dew_access_can_block(void)
{
#ifdef __EMSCRIPTEN__
  return 0;
#else
  return 1;
#endif
}

void dew_dataset_get_info(struct dew_dataset_t *dataset, struct dew_dataset_info_t *info)
{
  if (!dataset || !info)
    return;
  dataset->info(*info);
}

uint32_t dew_dataset_attribute_count(struct dew_dataset_t *dataset)
{
  return dataset ? dataset->attributes.attrib_name_registry_count() : 0;
}

uint32_t dew_dataset_get_attribute_name(struct dew_dataset_t *dataset, uint32_t index, char *name, uint32_t name_buffer_size)
{
  if (!dataset)
    return 0;
  return dataset->attributes.attrib_name_registry_get(index, name, name_buffer_size);
}

struct dew_request_t *dew_dataset_request_region(struct dew_dataset_t *dataset, const struct dew_region_request_t *spec, struct dew_error_t **error)
{
  if (!dataset || !spec)
  {
    fill_error(error, {1, "null dataset or request"});
    return nullptr;
  }
  if (dataset->state.load(std::memory_order_acquire) != dew_dataset_ready)
  {
    fill_error(error, dataset->error.code ? dataset->error : dew_error_t{1, "dataset is not ready"});
    return nullptr;
  }

  auto request = std::make_shared<dew_request_t>();
  request->dataset = dataset;
  request->done = spec->done;
  request->done_user_ptr = spec->done_user_ptr;

  // Copy everything out of the caller's struct before returning: the request outlives this call, and
  // attribute_names points at memory the caller may free the moment we return.
  region_job_t job;
  for (int i = 0; i < 3; i++)
  {
    job.box_min[i] = spec->aabb_min[i];
    job.box_max[i] = spec->aabb_max[i];
  }
  job.lod_mode = spec->lod_mode;
  job.lod = spec->lod;
  job.max_points = spec->max_points;
  job.position_format = spec->position_format;
  job.clip_mode = spec->clip_mode;
  job.attribute_names.reserve(spec->attribute_count);
  for (uint32_t a = 0; a < spec->attribute_count; a++)
    job.attribute_names.emplace_back(spec->attribute_names && spec->attribute_names[a] ? spec->attribute_names[a] : "");

  dataset->requests.push_back(request);
  // Runs on the dataset's loop; the caller's thread is not blocked and the request is genuinely
  // pending when this returns.
  dataset->spawn_region_request(std::move(job), request);
  return request.get();
}

enum dew_request_status_t dew_request_status(struct dew_request_t *request)
{
  return request ? request->status.load(std::memory_order_acquire) : dew_request_failed;
}

enum dew_request_status_t dew_request_wait(struct dew_request_t *request, int32_t timeout_ms)
{
  if (!request)
    return dew_request_failed;

  auto terminal = [request] { return request->status.load(std::memory_order_acquire) != dew_request_pending; };
  {
    std::unique_lock<std::mutex> lock(request->wait_mutex);
    if (timeout_ms < 0)
      request->wait_cond.wait(lock, terminal);
    else
      request->wait_cond.wait_for(lock, std::chrono::milliseconds(timeout_ms), terminal);
  }

  const auto status = request->status.load(std::memory_order_acquire);
  // Still pending means the wait timed out -- unambiguously, unlike OpenVDS's bool.
  if (status == dew_request_pending)
    return status;

  // Deliver here rather than making the caller poll as well; claim_callback keeps it exactly once
  // even though the pump drain is also holding this request.
  if (request->claim_callback() && request->done)
    request->done(request, status, request->done_user_ptr);
  return status;
}

void dew_request_cancel(struct dew_request_t *request)
{
  if (request)
    request->cancel();
}

void dew_request_get_error(struct dew_request_t *request, struct dew_error_t **error)
{
  if (request)
    fill_error(error, request->error);
}

float dew_request_completion_factor(struct dew_request_t *request)
{
  if (!request)
    return 0.0f;
  return request->status.load(std::memory_order_acquire) == dew_request_pending ? 0.0f : 1.0f;
}

void dew_request_release(struct dew_request_t *request)
{
  if (!request)
    return;
  request->cancel();
  auto *dataset = request->dataset;
  if (!dataset)
    return;
  // Drop BOTH references the dataset holds. Anything still running keeps its own shared_ptr, so
  // releasing an unfinished request is safe -- it just means the caller is done with the handle.
  //
  // The dispatch queue matters as much as the request list: a release means no callback is wanted,
  // and a queued entry that nobody ever polls for would pin the request until the dataset died.
  {
    std::unique_lock<std::mutex> lock(dataset->dispatch_mutex);
    for (auto it = dataset->awaiting_dispatch.begin(); it != dataset->awaiting_dispatch.end(); ++it)
    {
      if (it->get() == request)
      {
        dataset->awaiting_dispatch.erase(it);
        break;
      }
    }
  }
  for (auto it = dataset->requests.begin(); it != dataset->requests.end(); ++it)
  {
    if (it->get() == request)
    {
      dataset->requests.erase(it);
      return;
    }
  }
}

uint8_t dew_request_get_result(struct dew_request_t *request, struct dew_request_result_t *out)
{
  if (!request || !out)
    return 0;
  if (request->status.load(std::memory_order_acquire) != dew_request_completed)
    return 0;

  request->buffer_views.clear();
  request->buffer_views.reserve(request->buffers.size());
  for (auto &buffer : request->buffers)
  {
    dew_attribute_buffer_t view{};
    view.name = buffer.name.c_str();
    view.name_size = uint32_t(buffer.name.size());
    view.type = buffer.type;
    view.components = buffer.components;
    view.data = buffer.data.data();
    view.size_bytes = buffer.data.size();
    request->buffer_views.push_back(view);
  }

  out->point_count = request->point_count;
  out->buffers = request->buffer_views.data();
  out->buffer_count = uint32_t(request->buffer_views.size());
  out->nodes = request->nodes.data();
  out->node_count = uint32_t(request->nodes.size());
  return 1;
}

uint64_t dew_request_attribute_size(struct dew_request_t *request, uint32_t attribute_index)
{
  if (!request || attribute_index >= request->buffers.size())
    return 0;
  return request->buffers[attribute_index].data.size();
}

uint64_t dew_request_copy_attribute(struct dew_request_t *request, uint32_t attribute_index, uint8_t *dst, uint64_t dst_bytes, struct dew_error_t **error)
{
  if (!request || attribute_index >= request->buffers.size())
  {
    fill_error(error, {1, "no such attribute in this result"});
    return 0;
  }
  const auto &buffer = request->buffers[attribute_index].data;
  if (dst_bytes < buffer.size())
  {
    fill_error(error, {1, "destination buffer is too small"});
    return 0;
  }
  if (!buffer.empty())
    memcpy(dst, buffer.data(), buffer.size());
  return buffer.size();
}
