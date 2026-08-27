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
#ifndef DEW_ACCESS_QUERY_H
#define DEW_ACCESS_QUERY_H

/* Request-based, asynchronous access to a converted .dew dataset.
 *
 * Open a dataset, ask for the points in a world-space box (or for specific octree nodes), and get
 * back contiguous per-attribute buffers. Modelled on OpenVDS's VolumeDataAccessManager, with its
 * sharper edges filed off:
 *
 *   - request status is IDEMPOTENT; a request lives until dew_request_release, rather than being
 *     consumed by the first terminal observation;
 *   - waiting returns the status, so a timeout is distinguishable from a cancellation;
 *   - there is a completion callback, not just polling;
 *   - failure is distinct from cancellation.
 *
 * THREADING. A dataset and its requests are not thread-safe; call them from the thread that owns
 * them. The exceptions are dew_request_status and dew_request_cancel, which are safe from any
 * thread. Completion callbacks run on the thread that calls dew_dataset_poll (or dew_request_wait,
 * for that one request) -- never on an internal thread, so they cannot race the caller.
 *
 * MEMORY. The library never writes into caller-supplied memory asynchronously.
 * dew_request_copy_attribute writes only for the duration of the call. Result buffers obtained from
 * dew_request_get_result are owned by the request and stay valid until dew_request_release.
 */

#include <stdint.h>

#include <dew/access/export.h>
#include <dew/core/error.h>
#include <dew/core/pump.h>
#include <dew/core/format.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Awaiting these two.
 *
 * The whole protocol is one line each: keep looking until poll(handle) != pending. dew_dataset_t is
 * `opening` until the index read lands; dew_request_t is `pending` until the query finishes. The pump
 * guarantees a wake when either changes, so a poll after that wake is a pure read -- no completion
 * callback has to outlive the handle, and nothing has to be retracted when a caller loses interest.
 *
 * tools/bindgen turns these into await wrappers: dew/await/dew_await.hpp for C++ coroutines and
 * dew.aio for Python. Adding an awaitable handle is this annotation and nothing else. */
/* Both handles are OWNED -- the wrappers must destroy them -- but neither destructor is named
 * *_destroy, which is the only thing the generators infer from. So say it: without this a generated
 * wrapper treats them as borrowed views and quietly leaks every dataset and every request. */
//= awaitable: poll=dew_dataset_state pending=dew_dataset_opening
//= destroy: dew_dataset_close
struct dew_dataset_t;
//= awaitable: poll=dew_request_status pending=dew_request_pending release=dew_request_release
//= destroy: dew_request_release
struct dew_request_t;

enum dew_dataset_state_t
{
  dew_dataset_opening,
  dew_dataset_ready,
  dew_dataset_error
};

enum dew_request_status_t
{
  dew_request_pending,
  dew_request_completed,
  dew_request_canceled,
  dew_request_failed
};

/* How far to descend the octree.
 *
 * IMPORTANT: LOD nodes hold SUBSAMPLED COPIES of their descendants, not a partition of them. Every
 * mode returns exactly one frontier -- never a node and its ancestor -- because unioning levels
 * would double-count by 2-3x while still looking plausible. */
enum dew_lod_mode_t
{
  dew_lod_full,        /* descend to the leaves: full resolution, source points only */
  dew_lod_level,       /* stop at `lod` (larger = coarser) and return that frontier */
  dew_lod_point_budget /* descend while the running total stays under `max_points` */
};

enum dew_clip_mode_t
{
  dew_clip_node, /* whole nodes that overlap the box; fast, returns points outside it */
  dew_clip_point /* exactly the points inside the box; needs a per-point pass after decode */
};

enum dew_position_format_t
{
  dew_position_r64_absolute, /* double[3] world units -- lossless */
  dew_position_r32_relative, /* float[3] relative to each node's origin */
  dew_position_i32_grid      /* int32[3] in octree grid steps from each node's origin */
};

/* Fires from dew_dataset_poll / dew_request_wait on the CALLING thread. */
//= py.skip
typedef void (*dew_request_done_callback_t)(struct dew_request_t *request, enum dew_request_status_t status, void *user_ptr);

struct dew_dataset_options_t
{
  uint64_t memory_budget_bytes; /* 0 = derived default */
  uint32_t decode_threads;      /* 0 = decode inline on the polling thread */
  /* Target number of blob reads outstanding at once -- what turns a query over a high-latency store
   * from N round trips into roughly N/this. A target rather than a cap: one node's blobs are always
   * issued together, so the floor is 1 + the number of attributes requested. 0 = derived. */
  uint32_t max_reads_in_flight;
};

/* Returns immediately with the dataset in `opening`; `error` is only set for arguments that cannot
 * be used at all. A missing or corrupt dataset surfaces as state == dew_dataset_error, so that the
 * failure path is the same on native and under wasm, where opening cannot block.
 *
 * `pump` is where completions are delivered. Pass one to drive several subsystems from a single
 * wake, or NULL to have the dataset own a private pump -- in which case dew_dataset_poll drives it
 * and you need not know the pump exists. */
//= nullable: options, pump
DEW_ACCESS_EXPORT struct dew_dataset_t *dew_dataset_create(const char *url, uint32_t url_len, const char *connection, uint32_t connection_len, const struct dew_dataset_options_t *options, struct dew_pump_t *pump,
                                                           struct dew_error_t **error);
/* Close the dataset and release everything it holds. BLOCKS: it cancels outstanding requests, JOINS
 * the thread pool and stops the reader's loop, then destroys a private pump.
 *
 * Those are the same threads a wake fires on -- fire() runs the callback synchronously on whichever
 * thread published the completion -- so a language binding must NOT hold a runtime lock that the
 * wake callback needs. Under Python that lock is the GIL, and holding it here deadlocks: the join
 * waits for a worker that is blocked acquiring the GIL inside the wake. */
//= blocking
//= py.drain_on_destroy: dew_dataset_close
DEW_ACCESS_EXPORT void dew_dataset_close(struct dew_dataset_t *dataset);

DEW_ACCESS_EXPORT enum dew_dataset_state_t dew_dataset_state(struct dew_dataset_t *dataset);
DEW_ACCESS_EXPORT void dew_dataset_get_error(struct dew_dataset_t *dataset, struct dew_error_t **error);

/* Dispatch this dataset's completions on the calling thread; returns how many were dispatched.
 *
 * Only valid on a dataset with a PRIVATE pump (created with pump == NULL). On a dataset sharing a
 * pump it returns 0 -- draining one subsystem of a shared pump is exactly the partial drain that
 * loses wakeups, so call dew_pump_poll instead. */
//= py.skip
DEW_ACCESS_EXPORT uint32_t dew_dataset_poll(struct dew_dataset_t *dataset);
//= py.skip
DEW_ACCESS_EXPORT uint32_t dew_dataset_pending_count(struct dew_dataset_t *dataset);
//= blocking
DEW_ACCESS_EXPORT enum dew_dataset_state_t dew_dataset_wait_ready(struct dew_dataset_t *dataset, int32_t timeout_ms);
/* 1 when this build can block in a wait (native). Lets portable callers branch.
 * py.skip: Python always has the blocking path, since query_box runs the request to completion. */
//= py.skip
DEW_ACCESS_EXPORT uint8_t dew_access_can_block(void);

struct dew_dataset_info_t
{
  /* The root octree CELL -- a power-of-two cube that contains every point, but which can be
   * considerably larger than the data. For the actual extent of the points, run a coarse query
   * (dew_lod_point_budget) and take the bounds of the result. */
  double aabb_min[3];
  double aabb_max[3];
  double scale;
  double offset[3];
  uint32_t attribute_count;
  int32_t root_lod;
  int32_t deepest_lod;
  uint32_t node_point_limit;
  /* No total point count is stored in the format, so this is only filled in once a walk has visited
   * the whole tree. Check point_count_exact before trusting it. */
  uint64_t point_count;
  uint8_t point_count_exact;
};
DEW_ACCESS_EXPORT void dew_dataset_get_info(struct dew_dataset_t *dataset, struct dew_dataset_info_t *info);

DEW_ACCESS_EXPORT uint32_t dew_dataset_attribute_count(struct dew_dataset_t *dataset);
//= out_string: name[name_buffer_size]
DEW_ACCESS_EXPORT uint32_t dew_dataset_get_attribute_name(struct dew_dataset_t *dataset, uint32_t index, char *name, uint32_t name_buffer_size);

//= py.skip
struct dew_region_request_t
{
  double aabb_min[3];
  double aabb_max[3];
  enum dew_lod_mode_t lod_mode;
  int32_t lod;
  uint64_t max_points;
  //= arrays: attribute_names[attribute_count]
  const char *const *attribute_names; /* NUL-terminated; positions are implicit and never named */
  uint32_t attribute_count;
  enum dew_position_format_t position_format;
  enum dew_clip_mode_t clip_mode;
  dew_request_done_callback_t done;
  void *done_user_ptr;
};

/* Returns a new request; release it with dew_request_release.
 *
 * bind: skip because the generators cannot express "returns a handle of a DIFFERENT class than the
 * one this method hangs off". Python gets Dataset.query_box() instead, which runs the whole request
 * and hands back NumPy arrays -- see bindings/python/custom/query.h. */
//= bind: skip
DEW_ACCESS_EXPORT struct dew_request_t *dew_dataset_request_region(struct dew_dataset_t *dataset, const struct dew_region_request_t *request, struct dew_error_t **error);

//= py.skip
DEW_ACCESS_EXPORT enum dew_request_status_t dew_request_status(struct dew_request_t *request);
//= blocking
//= py.skip
DEW_ACCESS_EXPORT enum dew_request_status_t dew_request_wait(struct dew_request_t *request, int32_t timeout_ms);
//= py.skip
DEW_ACCESS_EXPORT void dew_request_cancel(struct dew_request_t *request);
//= py.skip
DEW_ACCESS_EXPORT void dew_request_get_error(struct dew_request_t *request, struct dew_error_t **error);
//= py.skip
DEW_ACCESS_EXPORT void dew_request_release(struct dew_request_t *request);
//= py.skip
DEW_ACCESS_EXPORT float dew_request_completion_factor(struct dew_request_t *request);

/* One attribute's contiguous buffer, concatenated across every node in the result. Buffer 0 is
 * always the positions and is never named in attribute_names; requested attributes start at 1. */
//= py.skip
struct dew_attribute_buffer_t
{
  const char *name;
  uint32_t name_size;
  enum dew_type_t type;
  enum dew_components_t components;
  const void *data;
  uint64_t size_bytes;
};

/* Where one node's points landed in the concatenated buffers, plus the origin its positions are
 * relative to (per node, because a single result-wide origin would lose precision in r32/i32). */
//= py.skip
struct dew_result_node_t
{
  uint32_t tree_id;
  uint16_t level;
  uint16_t index;
  int32_t lod;
  uint64_t first_point;
  uint64_t point_count;
  double position_offset[3];
  uint8_t is_leaf;
  uint8_t is_lod;
};

//= py.skip
struct dew_request_result_t
{
  uint64_t point_count;
  //= arrays: buffers[buffer_count]
  const struct dew_attribute_buffer_t *buffers;
  uint32_t buffer_count;
  //= arrays: nodes[node_count]
  const struct dew_result_node_t *nodes;
  uint32_t node_count;
};

//= py.skip
DEW_ACCESS_EXPORT uint8_t dew_request_get_result(struct dew_request_t *request, struct dew_request_result_t *out);
//= py.skip
DEW_ACCESS_EXPORT uint64_t dew_request_attribute_size(struct dew_request_t *request, uint32_t attribute_index);
//= arrays: dst[dst_bytes]
//= py.skip
DEW_ACCESS_EXPORT uint64_t dew_request_copy_attribute(struct dew_request_t *request, uint32_t attribute_index, uint8_t *dst, uint64_t dst_bytes, struct dew_error_t **error);

#ifdef __cplusplus
}
#endif
#endif /* DEW_ACCESS_QUERY_H */
