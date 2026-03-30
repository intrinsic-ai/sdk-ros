#include "intrinsic/hal/realtime_clock_interface.hpp"

#include "intrinsic/utils/time.hpp"
#include "intrinsic/utils/status.hpp"

namespace intrinsic {

RealtimeStatus RealtimeClockInterface::TickBlockingWithTimeout(
    Time current_timestamp, std::chrono::nanoseconds timeout) {
  return TickBlockingWithDeadline(current_timestamp, Now() + timeout);
}

}  // namespace intrinsic