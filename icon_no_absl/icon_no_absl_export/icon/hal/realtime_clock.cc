#include "icon/hal/realtime_clock.h"

#include <cassert>

namespace intrinsic {

static constexpr std::chrono::nanoseconds kStartUpLockstepTimeout =
    std::chrono::minutes(1);

RealtimeClock::RealtimeClock(
    intrinsic::icon::SharedMemoryLockstep lockstep,
    intrinsic::icon::ReadWriteMemorySegment<RealtimeClockUpdate>
        realtime_clock_update,
    const log::Logger* logger)
    : logger_(logger),
      lockstep_(std::move(lockstep)),
      update_(std::move(realtime_clock_update)) {
  // This matches the first `EndOperationA()` in `TickBlockingWithDeadline()`.
  // See comments in `TickBlockingWithDeadline()`. During startup it might take
  // several seconds until both sides of the lockstep are available.
  if (auto status = lockstep_->StartOperationAWithTimeout(
          /*timeout=*/kStartUpLockstepTimeout);
      !status.ok()) {
    auto message_view = status.GetMessage();
    INTRINSIC_SHARED_MEMORY_LOG(FATAL, logger_,
                                "Error starting RealtimeClock: {:.{}s}",
                                message_view.data(), message_view.size());
  }
}

RealtimeClock::~RealtimeClock() {
  // This matches the final `StartOperationA()` in `TickBlockingWithDeadline()`.
  // See comments in `TickBlockingWithDeadline()`.
  if (RealtimeStatus status = lockstep_->EndOperationA(); !status.ok()) {
    auto message_view = status.GetMessage();
    INTRINSIC_SHARED_MEMORY_LOG(WARNING, logger_,
                                "Error destructing RealtimeClock: {:.{}s}",
                                message_view.data(), message_view.size());
  }
}

tl::expected<std::unique_ptr<RealtimeClock>, Status> RealtimeClock::Create(
    intrinsic::icon::SharedMemoryManager& shm_manager,
    const log::Logger* logger) {
  auto lockstep = intrinsic::icon::CreateSharedMemoryLockstep(
      shm_manager, kRealtimeClockLockstepInterfaceName, logger);
  if (!lockstep.has_value()) {
    return tl::make_unexpected(lockstep.error());
  }

  if (auto status = shm_manager.AddSegmentWithDefaultValue<RealtimeClockUpdate>(
          kRealtimeClockUpdateInterfaceName,
          /*must_be_used=*/
          false);
      !status.ok()) {
    return tl::make_unexpected(status);
  }

  auto update =
      shm_manager
          .Get<intrinsic::icon::ReadWriteMemorySegment<RealtimeClockUpdate>>(
              kRealtimeClockUpdateInterfaceName, logger);
  if (!update.has_value()) {
    return tl::make_unexpected(update.error());
  }

  return std::unique_ptr<RealtimeClock>(new RealtimeClock(
      std::move(lockstep.value()), std::move(update.value()), logger));
}

RealtimeStatus RealtimeClock::TickBlockingWithDeadline(Time current_timestamp,
                                                       Time deadline) {
  // This is called from the clock owner's thread. Everything *outside* this
  // method is "Operation A", which is the reason for the inversion here
  // (End, then Start). The initial call to StartOperationA is in the
  // constructor.

  // Store `current_timestamp` before allowing "Operation B" (the control
  // update) to run.
  update_.GetValue().cycle_start_nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          current_timestamp.time_since_epoch())
          .count();

  if (auto status = lockstep_->EndOperationA(); !status.ok()) {
    return status;
  }
  // ...
  // ICON's turn! Cyclic update occurs here in the ICON control process.
  // ...

  // For the final call to `TickBlockingWithDeadline()`, the
  // `StartOperationAWithDeadline()` here matches the `EndOperationA()` in the
  // destructor.
  return lockstep_->StartOperationAWithDeadline(deadline);
}

RealtimeStatus RealtimeClock::Reset(std::chrono::nanoseconds timeout) {
  // Cancel, in case someone is still waiting or about to wait.
  lockstep_->Cancel(logger_);
  auto reset_status = lockstep_->Reset(timeout);
  // `StartOperationA()` matches the first `EndOperationA()` in
  // `TickBlockingWithDeadline()`. See comments in `TickBlockingWithDeadline()`.
  //
  // Even if `lockstep_->Reset()` failed, we still try to start Operation A to
  // reduce the risk of getting stuck.
  auto start_status = lockstep_->StartOperationAWithTimeout(timeout);
  if (!reset_status.ok()) {
    // If `lockstep_->Reset()` did fail, return that status.
    return reset_status;
  }
  return start_status;
}

}  // namespace intrinsic
