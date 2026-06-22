#include "icon/control/realtime_clock_interface.h"

#include "icon/utils/status.h"
#include "icon/utils/time.h"

namespace intrinsic {

RealtimeStatus RealtimeClockInterface::TickBlockingWithTimeout(
    Time current_timestamp, std::chrono::nanoseconds timeout) {
  return TickBlockingWithDeadline(current_timestamp, Now() + timeout);
}

}  // namespace intrinsic
