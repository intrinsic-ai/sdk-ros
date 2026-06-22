#ifndef ICON_HAL_REALTIME_CLOCK_H_
#define ICON_HAL_REALTIME_CLOCK_H_

#include <stdint.h>

#include <chrono>
#include <memory>
#include <string_view>

#include "icon/control/realtime_clock_interface.h"
#include "icon/interprocess/shared_memory_lockstep/shared_memory_lockstep.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/utils/attributes.h"
#include "icon/utils/log.h"
#include "icon/utils/status.h"
#include "icon/utils/time.h"

namespace intrinsic {

static constexpr std::string_view kRealtimeClockLockstepInterfaceName =
    "realtime_clock_lockstep";
static constexpr std::string_view kRealtimeClockUpdateInterfaceName =
    "realtime_clock_update";

// Payload for clock updates; gets stored in shared memory.
struct RealtimeClockUpdate {
  // Cycle start time in nanoseconds since the epoch.
  int64_t cycle_start_nanoseconds;
};

// RealtimeClock is an implementation of RealtimeClockInterface used by
// hardware modules to drive the realtime clock. It talks with the ICON server
// over shared memory.
class RealtimeClock : public RealtimeClockInterface {
 public:
  // Creates a RealtimeClock using memory segments with names specified by
  // `kRealtimeClockLockstepInterfaceName` and
  // `kRealtimeClockUpdateInterfaceName` by registering the respective segment
  // on `shm_manager`.
  static tl::expected<std::unique_ptr<RealtimeClock>, Status> Create(
      intrinsic::icon::SharedMemoryManager& shm_manager,
      const log::Logger* logger INTR_ATTRIBUTE_LIFETIME_BOUND);

  // This class is non-moveable and non-copyable to ensure that custom
  // destructor logic only ever runs once.
  RealtimeClock(const RealtimeClock& other) = delete;
  RealtimeClock& operator=(const RealtimeClock& other) = delete;
  RealtimeClock(const RealtimeClock&& other) = delete;
  RealtimeClock& operator=(const RealtimeClock&& other) = delete;

  // Signals to the ICON server that a real time cycle should begin. Blocks
  // until the cycle's update logic has completed; that is, blocks until
  // `ApplyCommand()` has completed for all hardware modules. It is the caller's
  // responsibility to further wait until the next cycle's start time before
  // calling this again.
  //
  // `current_timestamp` is considered the start time for the cycle.
  // Returns DeadlineExceededError if a tick is not finished by `deadline`.
  // Don't assume that the realtime cycle has been completed in case of such an
  // error. Use `Reset()` to recover from such a situation!
  // TODO(b/243163915): Reconsider how we pass time around ICON/timeslicer.
  RealtimeStatus TickBlockingWithDeadline(Time current_timestamp,
                                          Time deadline) override;

  // Resets the clock to its state after initialization, i.e. ready to call
  // `TickBlockingWithDeadline()`.
  // Returns DeadlineExceededError on timeout.
  RealtimeStatus Reset(std::chrono::nanoseconds timeout) override;

  ~RealtimeClock() override;

 private:
  // `lockstep` synchronizes the callsite with the ICON server's realtime update
  // loop. `realtime_clock_update` communicates the cycle start time.
  RealtimeClock(
      intrinsic::icon::SharedMemoryLockstep lockstep,
      intrinsic::icon::ReadWriteMemorySegment<RealtimeClockUpdate> update,
      const log::Logger* logger INTR_ATTRIBUTE_LIFETIME_BOUND);

  const log::Logger* logger_ = nullptr;
  intrinsic::icon::SharedMemoryLockstep lockstep_;
  intrinsic::icon::ReadWriteMemorySegment<RealtimeClockUpdate> update_;
};

}  // namespace intrinsic
#endif  // ICON_HAL_REALTIME_CLOCK_H_
