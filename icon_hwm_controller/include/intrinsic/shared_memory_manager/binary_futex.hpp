#pragma once

#include <cstdint>
#include <optional>
#include <chrono>
#include <atomic>

#include "intrinsic/utils/status.hpp"
#include "intrinsic/utils/time.hpp"

namespace intrinsic {

// A BinaryFutex class implements logic to signal between two high-performance
// processes. The futex implementation has similar semantics to a binary
// semaphore and can be shared through multiple processes via shared memory.
//
// The example below shows how a pair of binary futexes can be used to set up a
// client-server model:
//
// ```
//  const std::string request_id = "/test_futex_request";
//  const std::string response_id = "/test_futex_response";
//
//  intrinsic::icon::SharedMemoryManager shm_manager;
//  if(auto status = shm_manager.AddSegment(request_id, BinaryFutex()); !status.ok()) {
//    return status;
//  }
//  if(auto status = shm_manager.AddSegment(response_id, BinaryFutex()); !status.ok()) {
//    return status;
//  }
//
//  auto pid = fork();
//  if (pid == -1) {
//    return intrinsic::Status{
//        .code=intrinsic::StatusCode::kInternal,
//        .message=Intrinsic::StrError(errno),
//    };
//  }
//
//  // Server process
//  if (pid == 0) {
//    auto f_request =
//        intrinsic::icon::ReadWriteMemorySegment<BinaryFutex>::Get(request_id);
//    if (!f_request.ok()) {
//      return f_request.status();
//    }
//    auto f_response =
//        intrinsic::icon::ReadWriteMemorySegment<BinaryFutex>::Get(response_id);
//    if (!f_response.ok()) {
//      return f_response.status();
//    }
//
//    while (true) {
//      INTR_RETURN_IF_ERROR(f_request->GetValue().WaitFor());
//      LOG(INFO) << "Server received request. Doing some work...";
//      INTR_RETURN_IF_ERROR(f_response->GetValue().Post());
//    }
//  }
//
//  // Client process
//  auto f_request =
//      intrinsic::icon::ReadWriteMemorySegment<BinaryFutex>::Get(request_id);
//  if (!f_request.ok()) {
//    return f_request.status();
//  }
//  auto f_response =
//      intrinsic::icon::ReadWriteMemorySegment<BinaryFutex>::Get(response_id);
//  if (!f_response.ok()) {
//    return f_response.status();
//  }
//  for (int j = 0; j < 10; j++) {
//    if(auto status = f_request->GetValue().Post(); !status.ok()) {
//      return status;
//    }
//    LOG(INFO) << "Waiting on server to finish some work.";
//    if(auto status = f_response->GetValue().WaitFor(); !status.ok()) {
//      return status;
//    }
//  }
// ```
//
// More details can be found under
// https://man7.org/linux/man-pages/man2/futex.2.html
//
class BinaryFutex {
 public:
  // These constants aren't enum values, because `static_cast`ing them to
  // uint32_t everywhere would only make the implementation of BinaryFutex
  // unnecessarily verbose and harder to read.

  // This is the base state of the futex (unless it is constructed with
  // `posted == true`). A successful call to Wait() will reset the futex to this
  // value.
  static constexpr uint32_t kReady = 0;
  // Calling `Post()` on a `BinaryFutex` writes this value to the underlying
  // variable.
  static constexpr uint32_t kPosted = 1;
  // Calling `Close()` on a `BinaryFutex` writes this value to the underlying
  // variable. This state is final, i.e. a BinaryFutex that is `kClosed` will
  // never change to another state afterwards.
  static constexpr uint32_t kClosed = 2;

  // `posted` sets the initial value of the futex, i.e. if set to
  // true it equals calling `Post()`.
  // Set `private_futex` to true, if the futex is only used in one process, e.g.
  // it does not live in a shared memory segment. This can give some performance
  // benefits.
  explicit BinaryFutex(bool posted = false, bool private_futex = false);
  BinaryFutex(BinaryFutex& other) = delete;
  BinaryFutex& operator=(const BinaryFutex& other) = delete;
  BinaryFutex(BinaryFutex&& other);
  BinaryFutex& operator=(BinaryFutex&& other);
  ~BinaryFutex();

  // Posts on the futex and increases its value to one.
  // If the current value is already one, the value will not further increase.
  // If the value changed, this wakes up to one thread waiting on this
  // BinaryFutex.
  //
  // Returns an internal error if the futex could not be increased.
  // Real-time safe.
  // Thread-safe.
  RealtimeStatus Post();
  
  // Waits until the futex becomes `kPosted`, `kClosed` or the deadline expires.
  // As soon as the futex takes either of the two values above, or if the futex
  // already *has* that value when this function starts, this function
  // immediately sets the futex to `kReady` and returns OkStatus. Checks the
  // futex before checking the deadline. So if the futex is already `kPosted` or
  // `kClosed`, the deadline is not checked.
  //
  // Use `Time::max()` for an infinite deadline.
  //
  // Returns InternalError if the futex could not be accessed.
  // Returns AbortedError if the futex is `kClosed`.
  // Returns DeadlineExceededError if the futex does not become either `kPosted`
  // or `kClosed` within the deadline.
  // Returns OkStatus and resets the futex to `kReady` on success (if the futex
  // already is or becomes `kPosted` before the deadline).
  //
  // Real-time safe when `deadline` is close enough.
  // Thread-safe.
  RealtimeStatus WaitUntil(Time deadline) const;

  // Waits until the futex becomes `kPosted`, `kClosed` or the timeout expires.
  // As soon as the futex takes either of the two values above, or if the futex
  // already *has* that value when this function starts, this function
  // immediately sets the futex to `kReady` and returns OkStatus. Checks the
  // futex before checking the timeout. So if the futex is already `kPosted` or
  // `kClosed`, the timeout is not checked.
  //
  // Use `std::chrono::nanoseconds::max()` for an infinite timeout.
  //
  // Returns InternalError if the futex could not be accessed.
  // Returns AbortedError if the futex is `kClosed`.
  // Returns DeadlineExceededError if the futex does not become either `kPosted`
  // or `kClosed` within the timeout.
  // Returns OkStatus and resets the futex to `kReady` on success (if the futex
  // already is or becomes `kPosted` before the timeout).
  //
  // Real-time safe when `timeout` is close enough.
  // Thread-safe.
  RealtimeStatus WaitFor(std::chrono::nanoseconds timeout) const;
  
  // Checks (without blocking) whether the current value of the futex indicates
  // that it has been `Post()`ed.
  // If it does, this function (atomically!) resets the futex value to `kReady`.
  //
  // NOTE: If the code calling TryWait() owns the BinaryFutex, you can often be
  // certain that the BinaryFutex is not closed. In this case, it's safe to mask
  // the `nullopt` case by using `value_or(false)`. On the other hand, if you
  // call this in a tight loop until it returns true, it's definitely a good
  // idea to break from that loop on `nullopt`.
  //
  // Returns true if the futex was `Post()`ed (and this function subsequently
  // reset the value).
  // Returns false if the futex wasn't `Post()`ed, but is still active.
  // Returns nullopt if the futex is closed (see `Close()` below) and never will
  // be `Post()`ed.
  std::optional<bool> TryWait() const;

  // Returns the current value of the futex.
  // This can either be kReady, kPosted or kClosed. The returned value might be
  // outdated by the time the caller uses the value.
  //
  // Real-time safe.
  uint32_t Value() const;

  // Marks this BinaryFutex as closed and stops all blocking operations.
  //
  // Wakes all waiters, so all ongoing calls to `WaitUntil()` or `WaitFor()`
  // will immediately return AbortedError.  Any subsequent calls to `Post()`,
  // `TryWait()`, `WaitFor()` or `WaitUntil()` will *immediately* return
  // AbortedError (or, in `TryWait()`'s case, `false`).
  void Close();

 private:
  // The atomic value is marked as mutable to create a const correct public
  // interface to the futex class. A call to wait has read-only semantics while
  // a call to post has write semantics.
  static_assert(
      std::atomic<uint32_t>::is_always_lock_free,
      "Atomic operations need to be lock free for multi-process communication");
  mutable std::atomic<uint32_t> val_ = {0};
  const bool private_futex_ = false;
};

}  // namespace intrinsic
