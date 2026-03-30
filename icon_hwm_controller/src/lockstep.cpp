#include "intrinsic/shared_memory_manager/lockstep.hpp"

#include <utility>

namespace intrinsic {

Lockstep& Lockstep::operator=(Lockstep&& other) {
  if (this != &other) {
    a_finished_ = std::exchange(other.a_finished_, intrinsic::BinaryFutex(false));
    b_finished_ =
        std::exchange(other.b_finished_, intrinsic::BinaryFutex(/*posted=*/true));
    state_.store(other.state_.load());
  }
  return *this;
}


RealtimeStatus Lockstep::StartOperationAWithDeadline(Time deadline) {
  // TODO(gaschler): Return early if cancelled.
  // Wait for Operation B to finish
  if (auto status = b_finished_.WaitUntil(deadline); !status.ok()) {
    return status;
  }
  if (state_.load() == State::kCancelled) {
    // Ignore error because returning Aborted to the caller is more important.
    (void)b_finished_.Post();
    RealtimeStatus status{ .code= StatusCode::kAborted};
    snprintf(status.message.data(), sizeof(status.message), 
             "Not starting operation A: lockstep has been cancelled");
    return status;
  }
  if (state_ != State::kBFinished) {
    RealtimeStatus status{ .code= StatusCode::kFailedPrecondition};
    snprintf(status.message.data(), sizeof(status.message), 
             "Not starting operation A: expected State::kBFinished");
    return status;
  }
  state_ = State::kARunning;
  return RtOkStatus();
}

RealtimeStatus Lockstep::StartOperationAWithTimeout(std::chrono::nanoseconds timeout) {
  return StartOperationAWithDeadline(Now() + timeout);
}

RealtimeStatus Lockstep::EndOperationA() {
  if (state_ == State::kCancelled) {
    return RtOkStatus();
  }
  if (state_ != State::kARunning) {
    RealtimeStatus status{ .code= StatusCode::kFailedPrecondition};
    snprintf(status.message.data(), sizeof(status.message), 
             "Not ending operation A: Did you call StartOperationA...?");
    return status;
  }
  state_ = State::kAFinished;
  return a_finished_.Post();
}

RealtimeStatus Lockstep::StartOperationBWithDeadline(Time deadline) {
  if(auto status = a_finished_.WaitUntil(deadline); !status.ok()) {
    return status;
  }
  if (state_ == State::kCancelled) {
    // Ignore error because returning Aborted to the caller is more important.
    (void)a_finished_.Post();
    RealtimeStatus status{ .code= StatusCode::kAborted};
    snprintf(status.message.data(), sizeof(status.message), 
             "Not starting operation B: lockstep has been cancelled");
    return status;
  }
  if (state_ != State::kAFinished) {
    RealtimeStatus status{ .code= StatusCode::kFailedPrecondition};
    snprintf(status.message.data(), sizeof(status.message), 
             "Expected State::kAFinished");
    return status;
  }
  state_ = State::kBRunning;
  return RtOkStatus();
}

RealtimeStatus Lockstep::StartOperationBWithTimeout(std::chrono::nanoseconds timeout) {
  return StartOperationBWithDeadline(Now() + timeout);
}

RealtimeStatus Lockstep::EndOperationB() {
  if (state_ == State::kCancelled) {
    return RtOkStatus();
  }
  if (state_ != State::kBRunning) {
    RealtimeStatus status{ .code= StatusCode::kFailedPrecondition};
    snprintf(status.message.data(), sizeof(status.message), 
             "Mismatched call to EndOperationB. Did you call StartOperationB...?");
    return status;
  }
  state_ = State::kBFinished;
  return b_finished_.Post();
}

void Lockstep::Cancel() {
  state_ = State::kCancelled;
  if (auto status = a_finished_.Post(); !status.ok()) {
    // TODO(nilsb): Add realtime logging
  }
  if (auto status = b_finished_.Post(); !status.ok()) {
    // TODO(nilsb): Add realtime logging
  }
}

RealtimeStatus Lockstep::Reset(std::chrono::nanoseconds timeout) {
  if (state_ != State::kCancelled) {
    return RealtimeStatus{ 
      .code= StatusCode::kFailedPrecondition,
      .message = "Reset expects a cancelled lockstep."
    };
  }
  // Acquire both futexes, so that any call to a `StartOperation...` function
  // will have to wait until the reset is done.
  auto deadline = Now() + timeout;
  if (auto status = a_finished_.WaitUntil(deadline); !status.ok()) {
    return status;
  }
  if (auto status = b_finished_.WaitUntil(deadline); !status.ok()) {
    return status;
  }
  state_ = State::kBFinished;
  // Let `StartOperationA...` be next.
  return b_finished_.Post();
}

}  // namespace intrinsic
