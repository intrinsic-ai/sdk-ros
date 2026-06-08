#pragma once

#include <atomic>
#include <chrono>
#include "intrinsic/shared_memory_manager/binary_futex.hpp"
#include "intrinsic/utils/log.hpp"
#include "intrinsic/utils/status.hpp"

namespace intrinsic
{

// Lockstep is a synchronization primitive that can be used to force two threads
// to operate in lock step.
//
// Code between `StartOperationA...` and `EndOperationA` is referred to as
// "Operation A". Code between `StartOperationB...` and `EndOperationB` is
// referred to as "Operation B".
//
// The Lockstep class ensures that these operations are performed (to
// completion, with mutual exclusion) in the following order:
//
//    Operation A
//    Operation B
//    Operation A
//    Operation B
//    ...
//
// Attempts to perform these operations in any other order will block on either
// `StartOperationA...` or `StartOperationB...`.
//
// Essentially, it implements the following state machine:
//
// ====== Operation A =======| === Operation B ===
//        StartOperationB()  |
//    ┌──────────┐      ╲    |     ┌──────────┐
//    │kAFinished│ ----------|---> │kBRunning │
//    └──────────┘           |     └──────────┘
//         ^                 |          |
//         | EndOperationA() |          | EndOperationB()
//         |                 |          v
//    ┌──────────┐           |     ┌──────────┐
//    │kARunning │ <---------|---- │kBFinished│ <---- Start
//    └──────────┘           |  ╲  └──────────┘
//                       StartOperationA()
//
// The implementation is designed to be efficient (using low-level futexes) and
// is intended for realtime use.
class Lockstep {
public:
  static constexpr std::chrono::nanoseconds kResetTimeout = std::chrono::seconds(1);

  Lockstep() = default;
  Lockstep(Lockstep & other) = delete;
  Lockstep & operator=(const Lockstep & other) = delete;
  Lockstep(Lockstep && other) = delete;
  Lockstep & operator=(Lockstep && other);

  // Blocks the current thread until Operation A is ready to begin or timeout
  // has expired. Similar to StartOperationAWithDeadline except that this uses a
  // timeout instead of a deadline.
  //
  // Returns early when `Cancel()` has been called.
  // For concurrent calls, only one `StartOperationA...()` returns.
  //
  // Returns `OkStatus` on success. Otherwise, returns an error, in which case
  // user code *should not* perform Operation A. Returns `kAborted` if
  // `Cancel()` has been called. Returns `kDeadlineExceeded` if the underlying
  // Futex wait timed out or `kInternal` in case of an internal futex error.
  RealtimeStatus StartOperationAWithTimeout(std::chrono::nanoseconds timeout);

  // Blocks the current thread until Operation A is ready to begin or the
  // deadline has expired. Similar to StartOperationAWithTimeout except that
  // this uses a deadline instead of a timeout.
  //
  // Returns early when `Cancel()` has been called.
  // For concurrent calls, only one `StartOperationA...()` returns.
  //
  // Checks the condition before checking the deadline. So if the condition is
  // already fulfilled, the deadline is not checked.
  //
  // Returns `OkStatus` on success. Otherwise, returns an error, in which case
  // user code *should not* perform Operation A. Returns `kAborted` if
  // `Cancel()` has been called. Returns `kDeadlineExceeded` if the underlying
  // Futex wait timed out or `kInternal` in case of an internal futex error.
  RealtimeStatus StartOperationAWithDeadline(Time deadline);

  // Signals that Operation A has completed, potentially waking a thread that is
  // waiting on `StartOperationB...()`.
  //
  // Returns `OkStatus` on success (including if `Cancel()` has been called).
  // Returns `kFailedPrecondition` if a matching StartOperationA...() has not
  // been called.
  RealtimeStatus EndOperationA();

  // Blocks the current thread until Operation B is ready to begin or timeout
  // has expired. Similar to StartOperationBWithDeadline except that this uses a
  // timeout instead of a deadline.
  //
  // Returns early when `Cancel()` has been called.
  // For concurrent calls, only one `StartOperationB...()` returns.
  //
  // Returns `OkStatus` on success. Otherwise, returns an error, in which case
  // user code *should not* perform Operation B. Returns `kAborted` if
  // `Cancel()` has been called. Returns `kDeadlineExceeded` if the underlying
  // Futex wait timed out or `kInternal` in case of an internal futex error.
  RealtimeStatus StartOperationBWithTimeout(std::chrono::nanoseconds timeout);

  // Blocks the current thread until Operation B is ready to begin or the
  // deadline has expired. Similar to StartOperationBWithTimeout except that
  // this uses a deadline instead of a timeout.
  // Checks the condition before checking the deadline. So if the condition is
  // already fulfilled, the deadline is not checked.
  //
  // Returns early when `Cancel()` has been called.
  // For concurrent calls, only one `StartOperationB...()` returns.
  //
  // Returns `OkStatus` on success. Otherwise, returns an error, in which case
  // user code *should not* perform Operation B. Returns `kAborted` if
  // `Cancel()` has been called. Returns `kDeadlineExceeded` if the underlying
  // Futex wait timed out or `kInternal` in case of an internal futex error.
  RealtimeStatus StartOperationBWithDeadline(Time deadline);

  // Signals that Operation B has completed, potentially waking a thread that is
  // waiting on `StartOperationA...()`.
  //
  // Returns `OkStatus` on success (including if `Cancel()` has been called).
  // Returns `kFailedPrecondition` if a matching StartOperationB...() has not
  // been called.
  RealtimeStatus EndOperationB();

  // Signals to all threads waiting on either `StartOperationA...()` or
  // `StartOperationB...()` to wake up and return `kAborted`. All subsequent
  // calls to `StartOperationA...()` and `StartOperationB...()` will return
  // `kAborted` until `Reset()` is called.
  void Cancel(const log::Logger * logger);

  // Resets the lockstep to its initial state.
  // Must only be called after `Cancel`, and thus should not be called
  // inside any Operation. Returns an error if setting the futexes times out or
  // if the lockstep is not cancelled.
  RealtimeStatus Reset(std::chrono::nanoseconds timeout = kResetTimeout);

private:
  enum class State : char
  {
    kBFinished,
    kARunning,
    kAFinished,
    kBRunning,
    kCancelled
  };
  intrinsic::BinaryFutex a_finished_{/*posted=*/false};
  intrinsic::BinaryFutex b_finished_{/*posted=*/true};
  std::atomic<State> state_ = State::kBFinished;
};

}  // namespace intrinsic
