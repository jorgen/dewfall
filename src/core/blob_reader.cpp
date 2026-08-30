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
#include "blob_reader.hpp"

#include "compressor.hpp"
#include "loop_hop.hpp"

#include <vio/operation/work.h>

#include <chrono>
#include <future>
#include <thread>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

namespace dew::core
{

// One completion path for every way a read can finish -- cold read, any of the cache-hit branches, a
// pool-hop decompress, or cancellation. Fires on whichever thread completed the read: the storage loop
// on a miss, the CALLING thread when the cache satisfies it inline. The hook must therefore only
// enqueue work.
void complete_read_request(read_request_t &r)
{
  // Take the continuation under the same lock that publishes _done, so an awaiter racing us either
  // stores its handle before we look (and we resume it) or sees _done and declines to suspend.
  // Neither order can lose the resume.
  std::coroutine_handle<> continuation{};
  vio::event_loop_t *resume_loop = nullptr;
  {
    std::unique_lock<std::mutex> lock(r._mutex);
    r._done = true;
    continuation = r._continuation;
    resume_loop = r._continuation_loop;
    r._continuation = {};
  }
  r._block_for_read.notify_all();

  // Order matters: the hook first, the coroutine resume LAST. Resuming can run arbitrary
  // continuation code -- including code that drops the last reference to this request -- so touching
  // r afterwards would be a use-after-free.
  if (r._on_complete)
    r._on_complete(r);

  if (continuation)
  {
    if (resume_loop)
      resume_loop->run_in_loop([continuation]() { continuation.resume(); });
    else
      continuation.resume();
  }
}

void read_request_t::wait_for_read()
{
#ifdef __EMSCRIPTEN__
  // Cooperative fallback: the render path co_awaits await_on() instead, so this only runs for the
  // (currently unused-on-wasm) write/LOD read paths. Spin the browser event loop until the read lands;
  // safe only at the top of the call stack (never re-entered from inside a loop handler).
  while (!_done)
  {
    vio::wasm::pump();
    emscripten_sleep(0);
  }
#else
  std::unique_lock<std::mutex> lock(_mutex);
  _block_for_read.wait(lock, [this] { return this->_done; });
#endif
}

blob_reader_t::blob_reader_t(const std::string &url, std::string_view connection, vio::thread_pool_t &thread_pool, perf_stats_t &perf_stats, vio::event_pipe_t<dew_error_t> &storage_error_pipe, dew_error_t &error)
  : _thread_pool(thread_pool)
  , _event_loop_thread()
  , _event_loop(_event_loop_thread.event_loop())
  , _perf_stats(perf_stats)
  , _storage_error(storage_error_pipe)
  , _read_request_pipe(_event_loop, vio::event_bind_t::bind(*this, &blob_reader_t::handle_read_request))
  , _read_cache(256 * 1024 * 1024)
  , _decompressed_cache(256 * 1024 * 1024)
{
  // Per-reader credentials rather than vio's process-global override: two datasets on two endpoints
  // must be openable from one process, which is exactly the case vio's own docs say to pass the
  // connection through for. An empty view means "environment + defaults", which is what the converter
  // relies on -- it installs the override up front in dew_converter_set_destination.
  _backend = create_storage_backend(url, connection, _event_loop, error);
}

blob_reader_t::~blob_reader_t()
{
  // Safety net: join the loop before _read_cache / _backend / the pipes destruct. An in-flight
  // do_read_request touches all of them (and locks the lru cache mutex); stopping the loop first drains it.
  stop_loop();
}

void blob_reader_t::stop_loop()
{
  // The backend's keep-alive connection pool holds open libuv handles; closing the loop with any still open
  // makes ~event_loop_t's uv_loop_close abort. So drain and close everything on the loop thread first.
  if (_backend)
  {
    // Run something on the loop and wait for it. The two platforms wait in opposite ways, and getting
    // that backwards deadlocks: natively the loop has its own thread, so we park on a future and let it
    // run. Under wasm the loop is COOPERATIVE -- run_in_loop only queues, and nothing will ever dequeue
    // it while we sit on a future -- so we have to drive the loop ourselves. Bounded, because teardown
    // must terminate even if a read is wedged; an abandoned read is survivable, a hung close is not.
    auto run_on_loop_sync = [this](std::function<void()> fn) {
#ifdef __EMSCRIPTEN__
      bool finished = false;
      _event_loop.run_in_loop([&fn, &finished]() { fn(); finished = true; });
      for (int i = 0; i < 1024 && !finished; ++i)
        vio::wasm::pump();
#else
      std::promise<void> done;
      auto fut = done.get_future();
      _event_loop.run_in_loop([&fn, &done]() { fn(); done.set_value(); });
      fut.wait();
#endif
    };
    // (1) Dispatch any queued read requests into in-flight coroutines, then wait for every in-flight read to
    //     finish -- each holds the backend and a pooled connection across its network co_await. Bootstrap
    //     index-read/probe hops count too (loop_hop.hpp): they were posted to this loop BEFORE the barrier
    //     below, so by the time it returns they have raised the counter. No new reads are posted during
    //     teardown (the render loader is gone, tree loads are quiesced), so this converges; the loop
    //     advances the reads on its own thread.
    run_on_loop_sync([]() {});
#ifdef __EMSCRIPTEN__
    // Same reason as above: yielding hands off to nobody when there is no other thread. Pump instead,
    // and give up rather than spin forever -- a read still outstanding here can only be one whose fetch
    // will never land (the page is going away), and the browser reclaims it either way.
    for (int i = 0; i < 1024 && _reads_in_flight.load(std::memory_order_acquire) > 0; ++i)
      vio::wasm::pump();
#else
    while (_reads_in_flight.load(std::memory_order_acquire) > 0)
      std::this_thread::yield();
#endif
    // (2) Nothing holds the backend now. Destroy it on the loop thread (dropping the io_manager + pool ->
    //     uv_close on every idle connection), then pump the loop so those close callbacks -- deferred for TLS
    //     while close_notify flushes -- run to completion before the loop is closed.
    run_on_loop_sync([this]() { _backend.reset(); });
    for (int i = 0; i < 16; ++i)
      run_on_loop_sync([]() {});
  }

  _event_loop_thread.stop_and_join();
}

dew_error_t blob_reader_t::read_index(index_load_t &out)
{
  return _backend->read_index(out);
}

vio::task_t<dew_error_t> blob_reader_t::probe_exists_async(vio::event_loop_t &resume_loop)
{
  if (!_backend)
    co_return dew_error_t{1, "no storage backend"};
  // Hopped, not awaited directly: object_backend_t's probe co_awaits an object_info bound to
  // _event_loop, and running that from the caller's thread is the race loop_hop.hpp exists to stop.
  auto *backend = _backend.get();
  co_return co_await co_on_loop(_event_loop, resume_loop, [backend]() { return backend->probe_exists_async(); }, &_reads_in_flight);
}

vio::task_t<dew_error_t> blob_reader_t::read_index_async(index_load_t &out, vio::event_loop_t &resume_loop)
{
  if (!_backend)
    co_return dew_error_t{1, "no storage backend"};
  // `out` lives in the AWAITING coroutine's frame, which stays alive across this co_await; the hop
  // writes it on the storage loop and the mutex/run_in_loop handoff publishes it to the caller.
  auto *backend = _backend.get();
  auto *out_ptr = &out;
  co_return co_await co_on_loop(_event_loop, resume_loop, [backend, out_ptr]() { return backend->read_index_async(*out_ptr); }, &_reads_in_flight);
}

read_request_t::awaiter_t co_read(blob_reader_t &reader, storage_location_t location, read_options_t options, vio::event_loop_t &resume_loop, std::shared_ptr<read_request_t> &out)
{
  // Issue first, then hand back an awaiter over the resulting request. Issuing inside the awaiter is
  // what makes this safe to use: the completion hook is installed as part of read(), which is the
  // only point at which it can be, since a cache hit completes before read() returns.
  out = reader.read(location, std::move(options));
  return out->await_on(resume_loop);
}

void blob_reader_t::invalidate(storage_location_t location)
{
  const cache_key_t key{location.file_id, location.offset};
  _read_cache.erase(key);
  _decompressed_cache.erase(key);
}

void blob_reader_t::set_read_cache_size(uint64_t max_bytes)
{
  // No clear(): set_max_bytes evicts LRU-first down to the new cap, so a runtime cap change (budget knob,
  // heap-pressure brake) shrinks the cache without discarding the hot entries that still fit.
  _read_cache.set_max_bytes(max_bytes);
}

void blob_reader_t::set_decompressed_cache_size(uint64_t max_bytes)
{
  _decompressed_cache.set_max_bytes(max_bytes);
}

uint64_t blob_reader_t::read_cache_current_bytes()
{
  return _read_cache.current_bytes();
}

std::shared_ptr<read_request_t> blob_reader_t::read(storage_location_t location, read_options_t options)
{
  const bool raw = options.raw;
  const bool decompress_inline = options.decompress_inline;
  auto ret = std::make_shared<read_request_t>();
  ret->raw = raw;
  ret->retain_hot = options.retain_hot;
  // Installed BEFORE any completion path can run: the three cache-hit branches below finish the
  // request before read() returns, so a hook attached afterwards would never fire.
  ret->_on_complete = std::move(options.on_complete);

  cache_key_t key{location.file_id, location.offset};
  if (decompress_inline && !raw)
  {
    auto decompressed_hit = _decompressed_cache.get(key);
    if (decompressed_hit.has_value())
    {
      _perf_stats.cache_hits.fetch_add(1, std::memory_order_relaxed);
      ret->buffer = decompressed_hit->data;
      ret->buffer_info.data = ret->buffer.get();
      ret->buffer_info.size = decompressed_hit->size;
      complete_read_request(*ret);
      return ret;
    }
  }
  auto cached = _read_cache.get(key, options.retain_hot);
  if (cached.has_value())
  {
    _perf_stats.cache_hits.fetch_add(1, std::memory_order_relaxed);
    auto &cv = cached.value();
    if (raw)
    {
      // Hand back the COMPRESSED bytes unchanged; the caller (the wasm decode worker) decompresses off-thread.
      ret->buffer = cv.compressed_data;
      ret->buffer_info = dew_blob_t(ret->buffer.get(), cv.compressed_size);
      complete_read_request(*ret);
      return ret;
    }
    const bool compressed = has_compression_magic(cv.compressed_data.get(), cv.compressed_size);
#ifndef __EMSCRIPTEN__
    if (compressed && !decompress_inline)
    {
      // read() is called on the RENDER thread from the per-frame scheduler (request_load). Decompressing a
      // cache hit inline here is what spikes "Refine" -- a camera move that re-exposes many still-cached blobs
      // fires dozens of decompresses in one frame. Hand it to the shared pool instead; the request completes
      // (is_done() flips) when the worker lands, one/few frames later, off the render thread. The pool is
      // drained at teardown (~processor thread_pool.join), and this task touches neither backend nor loop.
      _thread_pool.enqueue_detached([ret, data = cv.compressed_data, size = cv.compressed_size]() {
        auto decompressed = decompress_any(data.get(), size);
        if (decompressed.error.code == 0)
        {
          ret->buffer = std::move(decompressed.data);
          ret->buffer_info.data = ret->buffer.get();
          ret->buffer_info.size = decompressed.size;
        }
        else
        {
          ret->error = std::move(decompressed.error);
        }
        complete_read_request(*ret);
      });
      return ret;
    }
#endif
    if (compressed)
    {
      auto decompressed = decompress_any(cv.compressed_data.get(), cv.compressed_size);
      if (decompressed.error.code == 0)
      {
        ret->buffer = std::move(decompressed.data);
        ret->buffer_info.data = ret->buffer.get();
        ret->buffer_info.size = decompressed.size;
        if (decompress_inline)
          _decompressed_cache.put(key, decompressed_cache_value_t{ret->buffer, ret->buffer_info.size}, ret->buffer_info.size, ret->retain_hot);
      }
      else
      {
        ret->error = std::move(decompressed.error);
      }
    }
    else
    {
      ret->buffer = cv.compressed_data;
      ret->buffer_info.data = ret->buffer.get();
      ret->buffer_info.size = cv.compressed_size;
    }
    complete_read_request(*ret);
    return ret;
  }

  _perf_stats.cache_misses.fetch_add(1, std::memory_order_relaxed);
  auto copy = ret;
  _read_request_pipe.post_event(std::move(copy), std::move(location));
  return ret;
}

void blob_reader_t::handle_read_request(std::shared_ptr<read_request_t> &&read_request, storage_location_t &&location)
{
  [](blob_reader_t *self, std::shared_ptr<read_request_t> req, storage_location_t loc) -> vio::detached_task_t
  {
    co_await self->do_read_request(std::move(req), std::move(loc));
  }(this, std::move(read_request), std::move(location));
}

vio::task_t<void> blob_reader_t::do_read_request(std::shared_ptr<read_request_t> read_request, storage_location_t location)
{
  // Track in-flight reads so stop_loop() can wait them out before tearing down the backend (each read holds
  // the backend and a pooled connection across its co_await). The guard decrements on every co_return path.
  const int in_flight = _reads_in_flight.fetch_add(1, std::memory_order_acq_rel) + 1;
  // Raise the high-water mark. A plain max: contention here is harmless, the value is diagnostic.
  int peak = _peak_reads_in_flight.load(std::memory_order_acquire);
  while (in_flight > peak && !_peak_reads_in_flight.compare_exchange_weak(peak, in_flight, std::memory_order_acq_rel))
  {
  }
  struct in_flight_guard_t
  {
    std::atomic<int> *counter;
    ~in_flight_guard_t() { counter->fetch_sub(1, std::memory_order_acq_rel); }
  } in_flight_guard{&_reads_in_flight};
  auto read_start = std::chrono::steady_clock::now();
  auto buffer = std::make_shared<uint8_t[]>(location.size);
  uint32_t bytes_read = 0;
  auto result = co_await _backend->read_blob(location, buffer.get(), bytes_read);

  if (result.code != 0)
  {
    // The waiter sees the per-request error; ALSO flag the conversion (a failed LOD/upload source
    // read means the produced dataset cannot be trusted -- e.g. an unreachable destination for a
    // spilled blob). Readers must still check per-request errors and skip, never dereference.
    read_request->error = result;
    _storage_error.post_event(std::move(result));
  }
  else
  {
    // Cache the raw compressed data before decompression
    cache_key_t key{location.file_id, location.offset};
    _read_cache.put(key, cache_value_t{buffer, location.size}, location.size, read_request->retain_hot);

    // If cancelled, skip decompression but keep cached compressed data
    if (read_request->is_cancelled())
    {
      auto read_end = std::chrono::steady_clock::now();
      auto read_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(read_end - read_start).count());
      _perf_stats.lod_read.record(location.size, read_us);

      complete_read_request(*read_request);
      co_return;
    }

    read_request->buffer = buffer;
    read_request->buffer_info.size = bytes_read;
    read_request->buffer_info.data = buffer.get();

    // Decompress if needed -- unless this is a raw read (the decode worker decompresses off-thread).
    if (!read_request->raw && read_request->buffer && has_compression_magic(read_request->buffer.get(), read_request->buffer_info.size))
    {
      auto decompressed = decompress_any(read_request->buffer.get(), read_request->buffer_info.size);
      if (decompressed.error.code == 0)
      {
        read_request->buffer = std::move(decompressed.data);
        read_request->buffer_info.data = read_request->buffer.get();
        read_request->buffer_info.size = decompressed.size;
      }
      else
      {
        read_request->error = std::move(decompressed.error);
      }
    }
  }

  auto read_end = std::chrono::steady_clock::now();
  auto read_us = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(read_end - read_start).count());
  _perf_stats.lod_read.record(location.size, read_us);

  complete_read_request(*read_request);
}

} // namespace dew::core
