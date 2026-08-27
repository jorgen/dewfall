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
#pragma once

// Awaiting a coroutine that belongs to ANOTHER event loop.
//
// THE TRAP. vio::task_t's initial_suspend is std::suspend_never (vio/task.h), so calling a coroutine
// does NOT move it anywhere: its body runs eagerly, on the CALLER's thread, up to the first real
// suspension. Call a storage-backend coroutine from another loop's thread and that thread is the one
// that reaches vio::read_file / the objstore io -- i.e. it submits uv work to a loop it does not own,
// and then races that loop's completion callback for the awaiter's `done` flag and continuation
// handle (vio::file_io_state_t has no mutex; it is written for the case where issuer and loop are the
// same thread). Two ways that hurts: every subsequent access to the coroutine frame is a data race,
// because the completion resumes it on the io loop's thread; and if the callback reads `continuation`
// before await_suspend writes it, the resume is lost outright and the awaiting coroutine hangs
// forever. Same lost-wakeup that read_request_t::awaiter_t documents and defends against.
//
// THE SEAM. co_on_loop starts the io coroutine ON the loop that owns the io, and resumes the awaiting
// coroutine on the loop the awaiting coroutine actually runs on. The handoff between the two is one
// mutex plus a run_in_loop post, both of which the sanitizers model exactly. Neither loop blocks.
//
// Sibling of loop_blocking.hpp, which solves the other half of the same problem: a thread with no loop
// of its own, which must park until the io completes.

#include "error.hpp"

#include <vio/event_loop.h>
#include <vio/task.h>

#include <atomic>
#include <coroutine>
#include <memory>
#include <mutex>
#include <utility>

namespace dew::core
{
namespace loop_hop_detail
{
struct hop_state_t
{
  std::mutex mutex;
  std::coroutine_handle<> continuation{};
  bool done = false;
  // Value-initialized: nothing may read a result the factory has not written yet.
  dew_error_t result{};
};

// The coroutine that actually drives the io, on the target loop's thread. state, factory and the two
// pointers are BY-VALUE parameters so they are copied into the coroutine frame; a lambda's captures
// would instead live in the closure temporary, which is destroyed at the first suspension (the trap
// spelled out in loop_blocking.hpp).
//
// `in_flight`, when given, is raised BEFORE the io starts -- i.e. synchronously in the run_in_loop
// task, so a teardown barrier posted to the same loop afterwards cannot observe zero and conclude the
// backend is unused -- and dropped only AFTER the resume has been posted, so a stop_loop() that waits
// this counter out knows the resume_loop was still alive when it was posted to.
template <typename Factory>
vio::task_t<void> hop_coro(std::shared_ptr<hop_state_t> state, Factory factory, vio::event_loop_t *resume_loop, std::atomic<int> *in_flight)
{
  if (in_flight)
    in_flight->fetch_add(1, std::memory_order_acq_rel);

  auto err = co_await factory();

  // Publish the result and take the continuation under ONE lock, for the same reason
  // complete_read_request does: the awaiting coroutine may be anywhere relative to us.
  std::coroutine_handle<> continuation{};
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    state->result = std::move(err);
    state->done = true;
    continuation = state->continuation;
    state->continuation = {};
  }
  // NEVER resume inline: that is precisely the thread migration this header exists to prevent.
  if (continuation)
    resume_loop->run_in_loop([continuation]() { continuation.resume(); });

  if (in_flight)
    in_flight->fetch_sub(1, std::memory_order_acq_rel);
  co_return;
}

// Same handshake as read_request_t::awaiter_t, and for the same reason: the hop can complete at any
// point relative to the suspension, so `done` and the continuation are consulted under one mutex, and
// await_suspend returns bool so a hop that landed in between declines the suspension rather than
// hanging on a resume that has already been and gone.
struct hop_awaiter_t
{
  std::shared_ptr<hop_state_t> state;

  bool await_ready() const noexcept
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    return state->done;
  }
  bool await_suspend(std::coroutine_handle<> handle) noexcept
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    if (state->done)
      return false; // finished while we were getting here: carry on without suspending
    state->continuation = handle;
    return true;
  }
  // Unlocked by construction: every path here went through state->mutex (await_ready true, or
  // await_suspend declining) or through the resume post's event pipe, which locks its own.
  dew_error_t await_resume() noexcept { return std::move(state->result); }
};
} // namespace loop_hop_detail

// Start factory()'s coroutine on `target`'s thread and resume the awaiting coroutine on `resume_loop`.
//
// `factory` must be copyable/movable and return vio::task_t<dew_error_t>; anything it captures must
// outlive the io (the awaiting coroutine's frame does, since it stays suspended across this await).
// `in_flight` is optional and is described on hop_coro above.
//
// target == resume_loop is fine: the resume is simply deferred by one turn of that loop. Under wasm
// both loops are cooperative and vio::wasm::pump() drives every registered loop to a fixed point, so
// the hop costs one more iteration of the same pump and cannot deadlock. Nothing here blocks, which is
// what makes it usable on the non-ASYNCIFY query path.
template <typename Factory>
loop_hop_detail::hop_awaiter_t co_on_loop(vio::event_loop_t &target, vio::event_loop_t &resume_loop, Factory factory, std::atomic<int> *in_flight = nullptr)
{
  auto state = std::make_shared<loop_hop_detail::hop_state_t>();
  auto *resume = &resume_loop;
  // The lambda handed to run_in_loop is NOT itself a coroutine: it just forwards into hop_coro, whose
  // by-value parameters own copies of state/factory for the lifetime of the actual io. run_in_loop's
  // task_t<void> overload awaits the returned task inside a detached_task_t, so the frame is destroyed
  // rather than leaked when it finishes.
  target.run_in_loop([state, factory = std::move(factory), resume, in_flight]() mutable -> vio::task_t<void> { return loop_hop_detail::hop_coro(state, std::move(factory), resume, in_flight); });
  return loop_hop_detail::hop_awaiter_t{std::move(state)};
}
} // namespace dew::core
