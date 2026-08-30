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
#ifndef DEW_CONVERTER_H
#define DEW_CONVERTER_H

#include <stdbool.h>
#include <stdint.h>

#include <dew/core/error.h>
#include <dew/core/format.h>
#include <dew/converter/export.h>
#include <dew/core/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct dew_converter_header_t
{
  uint64_t point_count;
  double offset[3];
  double scale[3];
  double min[3];
  double max[3];
};

/* dew_attributes_t, dew_attribute_t and dew_blob_t -- and the dew_attributes_add_attribute builder
 * that fills the first -- live in <dew/core/types.h> (included above). They describe data, not
 * conversion, and the dataset read path is expressed in terms of them. */

struct dew_converter_file_pre_init_info_t
{
  /* The file's minimum corner in world coordinates, and whether it is known (found_aabb_min).
   *
   * UNLIKE found_point_count AND found_scale, WHICH ARE HINTS, THIS ONE IS LOAD-BEARING. The
   * converter dispatches inputs in rising min-morton order and derives the done-morton watermark
   * from it; everything strictly below that watermark is final, which is what lets LOD passes
   * conclude, subtrees finalize, leaf collapse retire its chunks and checkpoints commit.
   *
   * Omitting it is SAFE but not cheap: a file with no min gets input_order 0, the watermark stays at
   * 0 until every input has finished, and all of the collapse and LOD work defers to a single
   * terminal pass. On a 300-million-point conversion that was the difference between finishing
   * incrementally and running for over an hour without producing a readable dataset.
   *
   * It cannot be recovered later. dew_converter_header_t's min from init() would be a valid bound,
   * but by then dispatch order is already fixed, and the watermark's prefix logic is only meaningful
   * if inputs were dispatched in rising min-morton order in the first place.
   *
   * It must be a TRUE lower bound on every point of the file, on every axis -- morton is monotone
   * per coordinate, so the min corner's code is <= that of any point in the box. Too low merely
   * slows the watermark; too high silently drops points below it. When the exact min is unknown, an
   * estimate padded generously is the right answer. */
  double aabb_min[3];
  uint64_t approximate_point_count;
  uint64_t input_file_size_bytes;
  uint8_t found_aabb_min;
  uint8_t approximate_point_size_bytes;
  uint8_t found_point_count;
  /* The source file's native coordinate precision (e.g. the LAS header scale factors). When provided
   * (found_scale) and no explicit tree scale was set, the converter adopts the finest scale across the
   * first batch of files as the octree scale instead of a fixed default -- storing more precision than
   * the source carries only inflates the dataset. */
  double scale[3];
  uint8_t found_scale;
};

typedef struct dew_converter_file_pre_init_info_t (*dew_converter_file_pre_init_callback_t)(const char *filename, size_t filename_size, struct dew_error_t **error);

typedef void (*dew_converter_file_init_callback_t)(const char *filename, size_t filename_size, struct dew_converter_header_t *header, struct dew_attributes_t *attributes, void **user_ptr, struct dew_error_t **error);

typedef void (*dew_converter_file_convert_data_callback_t)(void *user_ptr, const struct dew_converter_header_t *header, const struct dew_attribute_t *attributes, uint32_t attributes_size, uint32_t max_points_to_convert, struct dew_blob_t *buffers,
                                                       uint32_t buffers_size, uint32_t *points_read, uint8_t *done, struct dew_error_t **error);

typedef void (*dew_converter_file_destroy_user_ptr_t)(void *user_ptr);

struct dew_converter_file_convert_callbacks_t
{
  dew_converter_file_pre_init_callback_t pre_init;
  dew_converter_file_init_callback_t init;
  dew_converter_file_convert_data_callback_t convert_data;
  dew_converter_file_destroy_user_ptr_t destroy_user_ptr;
};

typedef void (*dew_converter_progress_callback_t)(void *user_ptr, float progress);

typedef void (*dew_converter_warning_callback_t)(void *user_ptr, const char *message);

typedef void (*dew_converter_error_callback_t)(void *user_ptr, const struct dew_error_t *error);

typedef void (*dew_converter_done_callback_t)(void *user_ptr);

struct dew_converter_runtime_callbacks_t
{
  dew_converter_progress_callback_t progress;
  dew_converter_warning_callback_t warning;
  dew_converter_error_callback_t error;
  dew_converter_done_callback_t done;
};

/* Destination-upload callbacks. A SEPARATE struct (not an extension of runtime_callbacks_t, which
 * is passed by value -- growing it would silently break the ABI of every existing caller). All
 * callbacks fire on internal threads; keep them cheap and thread-safe. */
typedef void (*dew_converter_upload_progress_callback_t)(void *user_ptr, uint64_t bytes_uploaded, uint32_t bands_committed);
/* A band of finalized subtrees committed at the destination; everything strictly below done_morton
 * (a 192-bit morton code, 3x64 little-endian) is durably readable there. */
typedef void (*dew_converter_band_committed_callback_t)(void *user_ptr, uint32_t band_id, const uint64_t done_morton[3]);
typedef void (*dew_converter_upload_error_callback_t)(void *user_ptr, const struct dew_error_t *error, uint8_t parked);
typedef void (*dew_converter_upload_done_callback_t)(void *user_ptr);

struct dew_converter_upload_callbacks_t
{
  dew_converter_upload_progress_callback_t progress;
  dew_converter_band_committed_callback_t band_committed;
  dew_converter_upload_error_callback_t error;
  dew_converter_upload_done_callback_t done;
};

struct dew_converter_upload_state_t
{
  uint64_t bytes_uploaded;
  uint32_t bands_committed;
  uint32_t objects_written;
  uint8_t upload_parked;      /* retries exhausted; resume by reopening later */
  uint8_t destination_complete;
  uint64_t cache_resident_bytes;
  uint64_t cache_max_bytes;
  uint64_t cache_spilled_bytes;
};

//= py.skip
struct dew_converter_buffer_callbacks_t
{
  int tmp;
};

//= py.skip
struct dew_converter_str_buffer
{
  const char *data;
  uint32_t size;
};

enum dew_converter_conversion_status_t
{
  dew_conversion_status_error,
  dew_conversion_status_in_progress,
  dew_conversion_status_completed
};

enum dew_converter_open_file_semantics_t
{
  dew_open_file_semantics_open_existing,
  dew_open_file_semantics_truncate,
  dew_open_file_semantics_read_only
};

enum dew_converter_compression_t
{
  dew_converter_compression_none = 0,
  dew_converter_compression_zstd = 2,
  dew_converter_compression_huff0 = 3
};

struct dew_converter_attribute_stats_t
{
  char name[64];
  enum dew_type_t type;
  enum dew_components_t components;
  uint64_t buffer_count;
  uint64_t uncompressed_bytes;
  uint64_t compressed_bytes;
  double min_value;  double max_value;
  uint64_t path_counts[4];
  uint64_t lod_buffer_count;
  uint64_t lod_uncompressed_bytes;
  uint64_t lod_compressed_bytes;
};

struct dew_converter_stats_t
{
  uint32_t input_file_count;
  uint32_t total_buffer_count;
  uint32_t lod_buffer_count;
  uint32_t compression_method;
  uint64_t input_file_size_bytes;
  uint32_t attribute_count;
  //= arrays: attributes[attribute_count]
  struct dew_converter_attribute_stats_t attributes[32];
};

struct dew_converter_io_stats_t
{
  uint64_t total_bytes;
  uint64_t total_time_us;
  uint32_t operation_count;
  double avg_mbps;
  double peak_mbps;
  double low_mbps;
};

struct dew_converter_perf_stats_t
{
  double total_time_seconds;
  double total_bytes_written_mb;
  double overall_mbps;
  struct dew_converter_io_stats_t source_read;
  struct dew_converter_io_stats_t sort;
  struct dew_converter_io_stats_t source_write;
  struct dew_converter_io_stats_t lod_read;
  struct dew_converter_io_stats_t lod_write;
  double tree_build_seconds;
  double lod_generation_seconds;
  uint64_t cache_hits;
  uint64_t cache_misses;
};

struct dew_converter_t;
DEW_CONVERTER_EXPORT struct dew_converter_t *dew_converter_create(const char *cache_filename, uint64_t cache_filename_size, enum dew_converter_open_file_semantics_t open_file_semantics, struct dew_error_t **error);

// As dew_converter_create, but first applies `connection` (a vendor connection string -- credentials /
// endpoint / region; see connection_cli.h + vio connection_string.h) for the output URL's provider, so a
// dataset can be written directly to a cloud store. `connection` may be null/empty for local outputs or
// when credentials come from the environment. `open_file_semantics` is typically truncate.
DEW_CONVERTER_EXPORT struct dew_converter_t *dew_converter_create_with_connection(const char *url, uint64_t url_size, const char *connection, uint64_t connection_size, enum dew_converter_open_file_semantics_t open_file_semantics, struct dew_error_t **error);

// Destination mode: convert into a LOCAL cache file (`cache_path`, a bare path or file:// URL) while
// finalized subtrees upload incrementally to `destination_url` (s3:// az:// dir://; DEW2 layout).
// The cache stays a valid, renderable DEW at every checkpoint; blobs already uploaded are evicted
// from it under the cache cap (see dew_converter_set_cache_max_bytes), and when the disk runs out
// mid-conversion, unfinished data spills to the destination's spill/ area and is fetched back on
// demand. A null/empty destination degrades to dew_converter_create. `connection` carries the
// destination's credentials (null/empty = environment). Reopening the same cache+destination resumes:
// committed bands are never re-uploaded (a destination belonging to a different dataset generation is
// refused via the embedded uuid).
DEW_CONVERTER_EXPORT struct dew_converter_t *dew_converter_create_with_destination(const char *cache_path, uint64_t cache_path_size, const char *destination_url, uint64_t destination_url_size, const char *connection, uint64_t connection_size,
                                                                                            enum dew_converter_open_file_semantics_t open_file_semantics, struct dew_error_t **error);

// Cache-file resident-bytes cap for destination mode. 0 = unlimited (the default). Callable any
// time; lowering it triggers eviction/spill passes.
DEW_CONVERTER_EXPORT void dew_converter_set_cache_max_bytes(struct dew_converter_t *converter, uint64_t max_bytes);

// In-RAM decompressed read-cache budget (default 256MB). Also the seam a render/query consumer tunes.
DEW_CONVERTER_EXPORT void dew_converter_set_read_cache_bytes(struct dew_converter_t *converter, uint64_t max_bytes);

/* Size the DECOMPRESSED blob cache (default 256 MB).
 *
 * Distinct from the read cache above, and on a large conversion it matters more. The read cache
 * holds COMPRESSED bytes: a hit there still costs a full zstd decompress, every time. This one holds
 * the decoded bytes, so a hit costs a lookup.
 *
 * Leaf collapse re-reads a chunk unit once per leaf it spans, so the same blob is decoded over and
 * over. Measured on a 70M-point conversion with a 4 GB read cache and this left at its default:
 * 1.2M reads, 48% of them "cache hits" that decompressed again, 2798 CPU-seconds inflating 115 GB --
 * about 60% of all CPU the conversion used. Decoded data is several times larger than compressed, so
 * size this generously or it holds almost nothing. */
DEW_CONVERTER_EXPORT void dew_converter_set_decompressed_cache_bytes(struct dew_converter_t *converter, uint64_t max_bytes);

/* Write a JSON record of THIS RUN to `path`: phase timings, collapse and LOD breakdowns, the
 * three-way read split, decompression volume, and the io counters. Returns 1 on success.
 *
 * Distinct from dew info, which describes the finished DATASET. This describes the conversion, so
 * two runs can be diffed -- which is the only way some of these problems are visible at all. The
 * aggregate hit rate read 99.5% on a run spending most of its CPU re-inflating blobs, because a
 * decoded-cache hit and a re-inflating hit counted the same; read.decoded_hits against
 * read.recompressed_hits, and decompress.cores against wall, are what separate them.
 *
 * Call before dew_converter_destroy. */
//= py.skip
DEW_CONVERTER_EXPORT uint8_t dew_converter_write_stats(struct dew_converter_t *converter, const char *path, uint32_t path_size, struct dew_error_t **error);

DEW_CONVERTER_EXPORT void dew_converter_set_upload_callbacks(struct dew_converter_t *converter, struct dew_converter_upload_callbacks_t callbacks, void *user_ptr);

// Snapshot of upload/cache-tier state (approximate counters; safe any time). Returns false when the
// converter has no destination configured.
DEW_CONVERTER_EXPORT bool dew_converter_get_upload_state(struct dew_converter_t *converter, struct dew_converter_upload_state_t *state);

// Wait until conversion is complete and the cache file is a fully checkpointed, valid DEW --
// uploads may still be in flight (contrast dew_converter_wait_idle, which also drains them).
DEW_CONVERTER_EXPORT void dew_converter_wait_local_complete(struct dew_converter_t *converter);

// Destroying a converter whose pipeline is still running races the internal
// thread pool (an enqueue after the pool stopped aborts), so callers must reach
// a quiescent state first -- dew_converter_wait_idle is the usual way.
// Language bindings whose object destruction is implicit should drain here.
//= py.drain_on_destroy: dew_converter_wait_idle
DEW_CONVERTER_EXPORT void dew_converter_destroy(struct dew_converter_t *destroy);

DEW_CONVERTER_EXPORT void dew_converter_set_file_converter_callbacks(struct dew_converter_t *converter, struct dew_converter_file_convert_callbacks_t callbacks);

DEW_CONVERTER_EXPORT void dew_converter_set_runtime_callbacks(struct dew_converter_t *converter, struct dew_converter_runtime_callbacks_t callbacks, void *user_ptr);

DEW_CONVERTER_EXPORT void dew_converter_set_compression(struct dew_converter_t *converter, enum dew_converter_compression_t compression);

DEW_CONVERTER_EXPORT void dew_converter_set_store_original_order(struct dew_converter_t *converter, bool store);

DEW_CONVERTER_EXPORT void dew_converter_set_compression_level(struct dew_converter_t *converter, int level);

// Set the target points per octree node. This doubles as the read/sort chunk size, so it is the main
// control over stored blob size (one node ~ one compressed blob per attribute). Default 200000. Must be
// called before dew_converter_add_data_file.
DEW_CONVERTER_EXPORT void dew_converter_set_node_point_limit(struct dew_converter_t *converter, uint32_t points);

/* Explicitly pin the octree coordinate scale (a single value used for all three axes). Without this the
 * converter adopts the finest native scale of the first batch of input files (LAS header scale), falling
 * back to 0.00025 when the source reports none. Must be called before dew_converter_add_data_file. */
DEW_CONVERTER_EXPORT void dew_converter_set_tree_scale(struct dew_converter_t *converter, double scale);

/* LOD nodes carry only the visual attributes (position + rgb/intensity/classification) by default;
 * pass 1 to duplicate EVERY source attribute into every LOD level (the pre-slimming behavior, at a
 * substantial size cost). Must be called before dew_converter_add_data_file. */
DEW_CONVERTER_EXPORT void dew_converter_set_lod_all_attributes(struct dew_converter_t *converter, uint8_t all);

/* Keep the dataset MUTABLE: do not seal it when the input runs out.
 *
 * Normally a tree is sealed as soon as the converter can prove no more points can land in its morton
 * range, and sealing is what makes it uploadable and then evictable from the local cache -- which is
 * also what makes it impossible to add to. A mutable dataset stays open: ingest can span sessions
 * (reopen with dew_open_file_semantics_open_existing and add more files), and nothing is uploaded
 * until it is sealed. Collapse and LOD still run, so it reads and renders like any other dataset.
 *
 * Unlike the tree-initialization setters, this may be called on a dataset that has already been
 * converted and reopened -- that is its purpose. The mode persists in the dataset.
 *
 * Call dew_converter_finalize to end it. */
DEW_CONVERTER_EXPORT void dew_converter_set_mutable(struct dew_converter_t *converter, uint8_t is_mutable);

/* 1 while the dataset is mutable (including one reopened in that state). */
DEW_CONVERTER_EXPORT uint8_t dew_converter_is_mutable(struct dew_converter_t *converter);

/* End mutable mode: seal the dataset, and in destination mode ship it.
 *
 * Blocks until everything in flight has landed, every eligible tree has been marked final, and the
 * upload backlog has drained. After this the dataset is exactly what an ordinary conversion
 * produces, and it will no longer accept new input -- a later add_data_file routes points into
 * finalized trees, which is a hard error.
 *
 * Idempotent-ish: calling it on a dataset that is not mutable simply drains, like wait_idle. */
//= blocking
DEW_CONVERTER_EXPORT void dew_converter_finalize(struct dew_converter_t *converter);

/* ---------------------------------------------------------------------------------------------
 * Amending a mutable dataset with a new attribute.
 *
 * The workflow this exists for: convert the geometry once (which is the expensive part), then add
 * intensity, then colour, then whatever else, each as a separate pass over data that may not have
 * existed when the points were ingested.
 *
 * The join. New values arrive keyed, not ordered -- the converter reordered every point by morton
 * code, split it across nodes and collapsed it, so the caller cannot know where its Nth point ended
 * up. Instead the caller names an EXISTING attribute of the dataset to join on, and supplies
 * (key, value) pairs. The key attribute must be a single-component integer type; its values are
 * widened to uint64 and must be unique across the dataset. A dataset that wants to be amendable
 * later therefore has to carry such a key from the start -- e.g. a scan id and a point index packed
 * into one uint64 by the reader that ingested it.
 *
 * What this buys, and why LOD needs no special handling: an LOD node's key values are the key values
 * of the points that were sampled into it. Joining the LOD nodes against the same table therefore
 * gives each sampled point its own true value, for free. There is no separate LOD pass and no
 * provenance metadata -- the key attribute IS the provenance.
 *
 * Points with no entry in the table get zeroes, matching what a node that lacks an attribute reads
 * back as. A partially attributed dataset is legal.
 *
 * THE KEY MUST SURVIVE TO THE LOD LEVELS, or only the leaves are amended. LOD slimming keeps just
 * rgb/intensity/classification by default (it dominated dataset size otherwise), so a dataset meant
 * to be amended later must be converted with dew_converter_set_lod_all_attributes. Without it the
 * amend is not wrong -- coarse nodes simply read the attribute back as zeros -- but the whole
 * pyramid is what a renderer draws.
 *
 * Sequence:
 *   dew_converter_set_mutable(c, 1);
 *   dew_converter_add_attribute(c, "reflectance", 11, "point_key", 9, dew_type_r32, dew_components_1);
 *   dew_converter_add_data_for_attribute(c, "reflectance", 11, keys, values, n);   // repeatable
 *   dew_converter_commit_attributes(c);
 *
 * Only on a mutable dataset: a finalized one has trees that have been banded and evicted, and an
 * amend has to read every unit it touches. */

/* Declare an attribute the dataset does not have yet, and the existing attribute to join it on.
 * Values are buffered until dew_converter_commit_attributes. Returns 0 and sets the converter to
 * the error state if the dataset is not mutable, the name is already taken, or the key attribute
 * is not a single-component integer type. */
DEW_CONVERTER_EXPORT uint8_t dew_converter_add_attribute(struct dew_converter_t *converter, const char *name, uint32_t name_size, const char *key_attribute, uint32_t key_attribute_size,
                                                         enum dew_type_t type, enum dew_components_t components);

/* Supply values for a declared attribute. `keys` is `count` uint64 key values; `values` is `count`
 * elements of the declared type and component count, contiguous. Both are copied. Callable as many
 * times as it takes to deliver the data; a repeated key overwrites its earlier value.
 *
 * Skipped by the Python bindings: `values` has no static element type -- it follows whatever
 * add_attribute declared -- so it wants a NumPy-aware wrapper rather than a generated pointer. */
//= py.skip
DEW_CONVERTER_EXPORT uint8_t dew_converter_add_data_for_attribute(struct dew_converter_t *converter, const char *name, uint32_t name_size, const uint64_t *keys, const void *values, uint64_t count);

/* Apply every declared attribute to every unit of the dataset and drop the buffered values.
 *
 * Reads each unit's key buffer, writes one new blob per unit per attribute, and leaves the rest of
 * the unit's blobs exactly where they were. Blocks until it has landed. Cost is one read and one
 * write of one attribute per unit -- it does NOT recompress the units' other attributes. */
//= blocking
DEW_CONVERTER_EXPORT void dew_converter_commit_attributes(struct dew_converter_t *converter);

// Read/sort chunk byte target (default 64 MiB): the converter ingests each input in chunks of about
// this many bytes (computed from the file's per-point width, never below the node point limit,
// capped at 8M points per chunk). Larger chunks amortize source reads and sorting; the octree still
// subdivides to node_point_limit leaves and finalized leaves are rewritten into per-node units.
// Must be called before dew_converter_add_data_file.
DEW_CONVERTER_EXPORT void dew_converter_set_read_chunk_bytes(struct dew_converter_t *converter, uint64_t bytes);

// May block on ingest backpressure, so Python bindings must release the GIL around it.
//= arrays: buffers[buffer_count]
//= blocking
DEW_CONVERTER_EXPORT void dew_converter_add_data_file(struct dew_converter_t *converter, struct dew_converter_str_buffer *buffers, uint32_t buffer_count);

DEW_CONVERTER_EXPORT void dew_converter_wait_idle(struct dew_converter_t *converter);

DEW_CONVERTER_EXPORT enum dew_converter_conversion_status_t dew_converter_status(struct dew_converter_t *converter);

DEW_CONVERTER_EXPORT bool dew_converter_get_compression_stats(struct dew_converter_t *converter, struct dew_converter_stats_t *stats);

DEW_CONVERTER_EXPORT bool dew_converter_get_perf_stats(struct dew_converter_t *converter, struct dew_converter_perf_stats_t *perf_stats);

DEW_CONVERTER_EXPORT bool dew_converter_get_live_perf_stats(struct dew_converter_t *converter, struct dew_converter_perf_stats_t *perf_stats);

#ifdef __cplusplus
}
#endif

#endif
