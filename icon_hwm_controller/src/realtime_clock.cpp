#include "intrinsic/hal/realtime_clock.hpp"
#include <cassert>

namespace intrinsic {

static constexpr std::chrono::nanoseconds kStartUpLockstepTimeout = std::chrono::minutes(1);

RealtimeClock::RealtimeClock(
    intrinsic::hal::SharedMemoryLockstep lockstep,
    intrinsic::hal::ReadWriteMemorySegment<RealtimeClockUpdate> realtime_clock_update)
    : lockstep_(std::move(lockstep)),
      update_(std::move(realtime_clock_update)) {
  // This matches the first EndOperationA in TickBlockingWithTimeout. See
  // comments in TickBlockingWithTimeout.
  // During startup it might take several seconds until both sides of the
  // lockstep are available.
  if(auto status = lockstep_->StartOperationAWithTimeout(
             /*timeout=*/kStartUpLockstepTimeout); !status.ok()) {
    std::cerr << "FATAL: Error starting RealtimeClock: " << status.GetMessage() << std::endl;
    std::exit(1);
  }
}

RealtimeClock::~RealtimeClock() {
  // This matches the final StartOperationA in TickBlockingWithTimeout. See
  // comments in TickBlockingWithTimeout.
  if (RealtimeStatus status = lockstep_->EndOperationA(); !status.ok()) {
    std::cerr << "WARN: Error destructing RealtimeClock: " << status.GetMessage() << std::endl;
  }
}

tl::expected<std::unique_ptr<RealtimeClock>, Status> RealtimeClock::Create(
    intrinsic::hal::SharedMemoryManager& shm_manager) {
  auto lockstep = intrinsic::hal::CreateSharedMemoryLockstep(shm_manager, kRealtimeClockLockstepInterfaceName);
  if (!lockstep.has_value()) {
    return tl::make_unexpected(lockstep.error());
  }
  
  if (auto status = shm_manager.AddSegmentWithDefaultValue<RealtimeClockUpdate>(kRealtimeClockUpdateInterfaceName, /*must_be_used=*/false); !status.ok()) {
    return tl::make_unexpected(status);
  }

  auto update = shm_manager.Get<intrinsic::hal::ReadWriteMemorySegment<RealtimeClockUpdate>>(
    kRealtimeClockUpdateInterfaceName);
  if (!update.has_value()) {
    return tl::make_unexpected(update.error());
  }

  return std::unique_ptr<RealtimeClock>(new RealtimeClock(
      std::move(lockstep.value()),
      std::move(update.value())));
}

RealtimeStatus RealtimeClock::TickBlockingWithDeadline(
    Time current_timestamp, Time deadline) {
  // This is called from the clock owner's thread. Everything *outside* this
  // method is "Operation A", which is the reason for the inversion here
  // (End, then Start). The initial call to StartOperationA is in the
  // constructor.

  // Store `current_timestamp` before allowing "Operation B" (the control
  // update) to run.
  update_.GetValue().cycle_start_nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        current_timestamp.time_since_epoch()).count();
  
  
  if(auto status = lockstep_->EndOperationA(); !status.ok()) {
    return status;
  }
  // ...
  // ICON's turn! Cyclic update occurs here in the ICON control process.
  // ...

  // For the final call to TickBlockingWithTimeout(), the
  // StartOperationAWithDeadline() here matches the EndOperationA() in the
  // destructor.
  return lockstep_->StartOperationAWithDeadline(deadline);
}

RealtimeStatus RealtimeClock::Reset(std::chrono::nanoseconds timeout) {
   // Cancel, in case someone is still waiting or about to wait.
  lockstep_->Cancel();
  auto reset_status = lockstep_->Reset(timeout);
  // StartOperationA matches the first EndOperationA in TickBlockingWithTimeout.
  // See comments in TickBlockingWithTimeout.
  auto start_status = lockstep_->StartOperationAWithTimeout(timeout);
  if (!reset_status.ok()) {
    return reset_status;
  }
  return start_status;
}

}  // namespace intrinsic
