#ifndef ICON_HAL_HARDWARE_INTERFACE_HANDLE_H_
#define ICON_HAL_HARDWARE_INTERFACE_HANDLE_H_

#include <cstdint>
#include <cstdio>
#include <tl/expected.hpp>
#include <utility>

#include "flatbuffers/flatbuffers.h"  // IWYU pragma: keep
#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "icon/hal/icon_state_register.h"  // IWYU pragma: keep
#include "icon/interprocess/shared_memory_manager/memory_segment.h"
#include "icon/utils/current_cycle.h"
#include "icon/utils/status.h"
#include "icon/utils/time.h"

namespace intrinsic::icon {

// A read-only hardware interface handle to a shared memory segment.
// Throughout the lifetime of the hardware interface, we have to keep the shared
// memory segment alive. We therefore transparently wrap the hardware interface
// around the shared memory segment, exposing solely the actual hardware
// interface type.
// Prefer using a StrictHardwareInterfaceHandle for reading commands/status
// messages.
//
// Used internally by the StrictHardwareInterfaceHandle.
template <class T>
class HardwareInterfaceHandle {
 public:
  HardwareInterfaceHandle() = default;

  // Preferred constructor.
  explicit HardwareInterfaceHandle(ReadOnlyMemorySegment<T>&& segment)
      : segment_(std::move(segment)),
        hardware_interface_(flatbuffers::GetRoot<T>(segment_.GetRawValue())) {}

  // TODO(b/218389388): Re-evaluate if we can get away without the operator* in
  // order to avoid shallow pointer copies. Same below for a
  // MutableHardwareInterface.
  const T* operator*() const { return hardware_interface_; }

  const T* operator->() const { return hardware_interface_; }

  // Returns the number of updates made to the segment. This helps detect issues
  // with missing updates.
  int64_t NumUpdates() const { return segment_.Header().NumUpdates(); }
  // Returns the time the segment was last updated. This helps detect stale
  // data.
  Time LastUpdatedTime() const { return segment_.Header().LastUpdatedTime(); }
  // Returns the cycle the segment was last updated. This helps detect stale
  // data.
  uint64_t LastUpdatedCycle() const {
    return segment_.Header().LastUpdatedCycle();
  }

 private:
  ReadOnlyMemorySegment<T> segment_;
  const T* hardware_interface_ = nullptr;
};

// A read-write hardware interface handle to a shared memory segment.
// Throughout the lifetime of the hardware interface, we have to keep the shared
// memory segment alive. We therefore transparently wrap the hardware interface
// around the shared memory segment, exposing solely the actual hardware
// interface type.
// Prefer using a MutableStrictHardwareInterfaceHandle for writing
// commands/status messages.
// Used internally by the MutableStrictHardwareInterfaceHandle.
template <class T>
class MutableHardwareInterfaceHandle {
 public:
  MutableHardwareInterfaceHandle() = default;

  // Preferred constructor.
  explicit MutableHardwareInterfaceHandle(ReadWriteMemorySegment<T>&& segment)
      : segment_(std::move(segment)),
        hardware_interface_(
            flatbuffers::GetMutableRoot<T>(segment_.GetRawValue())) {}

  T* operator*() { return hardware_interface_; }
  const T* operator*() const { return hardware_interface_; }

  T* operator->() { return hardware_interface_; }
  const T* operator->() const { return hardware_interface_; }

  // Returns the number of updates made to the segment. This helps detect issues
  // with missing updates.
  int64_t NumUpdates() const { return segment_.Header().NumUpdates(); }
  // Returns the time the segment was last updated. This helps detect stale
  // data.
  Time LastUpdatedTime() const { return segment_.Header().LastUpdatedTime(); }
  // Returns the cycle the segment was last updated. This helps detect stale
  // data.
  uint64_t LastUpdatedCycle() const {
    return segment_.Header().LastUpdatedCycle();
  }

  // Updates the `time` and current_cycle at which the segment was last updated
  // and increments an update counter.
  // The special IconState interface can be used to validate that the
  // interface was updated in the same cycle IconState reports as the current
  // cycle
  void UpdatedAt(Time time) {
    segment_.UpdatedAt(time, Cycle::GetCurrentCycle());
  }

 private:
  ReadWriteMemorySegment<T> segment_;
  T* hardware_interface_ = nullptr;
};

// Validates that the `hw_interface` was updated in the current control cycle.
// Returns FailedPreconditionError if not.
template <class HardwareInterfaceT>
inline RealtimeStatus WasUpdatedThisCycle(
    const HardwareInterfaceHandle<intrinsic_fbs::IconState>& icon_state,
    const HardwareInterfaceT& hw_interface) {
  if (static_cast<int64_t>(icon_state.LastUpdatedCycle()) !=
      icon_state->current_cycle()) [[unlikely]] {
    return {
        .code = StatusCode::kFailedPrecondition,
        .message = {"Cycle count of ICON state is inconsistent."},
    };
  }

  if (static_cast<int64_t>(hw_interface.LastUpdatedCycle()) !=
      icon_state->current_cycle()) [[unlikely]] {
    RealtimeStatus status;
    status.code = StatusCode::kFailedPrecondition;
    (void)std::snprintf(status.message.data(), status.message.size(),
                        "Command was not updated this cycle. icon_cycle (%ld) "
                        "!= command_cycle (%lu)",
                        icon_state->current_cycle(),
                        hw_interface.LastUpdatedCycle());
    return status;
  }
  return RtOkStatus();
}

// Wraps a a read-only hardware interface handle, and an IconState handle, and
// checks that `Value` was updated same cycle IconState reports as the current
// cycle.
// Use this interface to read commands/status messages.
template <class T>
class StrictHardwareInterfaceHandle {
 public:
  StrictHardwareInterfaceHandle() = default;

  // Preferred constructor.
  explicit StrictHardwareInterfaceHandle(
      HardwareInterfaceHandle<T> hardware_interface,
      HardwareInterfaceHandle<intrinsic_fbs::IconState> icon_state)
      : hardware_interface_(std::move(hardware_interface)),
        icon_state_(std::move(icon_state)) {}

  // Read only access to the stored value.
  // Checks that the value was updated same cycle IconState reports as the
  // current cycle.
  // Returns FailedPreconditionError when not.
  tl::expected<const T*, RealtimeStatus> Value() const {
    if (auto status = WasUpdatedThisCycle(icon_state_, hardware_interface_);
        !status.ok()) {
      return tl::unexpected(status);
    }
    return *hardware_interface_;
  }

  // Returns the number of updates made to the segment. This helps detect issues
  // with missing updates.
  int64_t NumUpdates() const { return hardware_interface_.NumUpdates(); }
  // Returns the time the segment was last updated. This helps detect stale
  // data.
  Time LastUpdatedTime() const { return hardware_interface_.LastUpdatedTime(); }
  // Returns the cycle the segment was last updated. This helps detect stale
  // data.
  uint64_t LastUpdatedCycle() const {
    return hardware_interface_.LastUpdatedCycle();
  }

 private:
  HardwareInterfaceHandle<T> hardware_interface_;
  HardwareInterfaceHandle<intrinsic_fbs::IconState> icon_state_;
};

// Wraps a a read-write hardware interface handle, and an IconState handle.
// Checks that `Value` was updated same cycle IconState reports as the current
// cycle.
// Use this interface to write commands/status messages.
// Also use this interface to read commands where you also have mutable access.
template <class T>
class MutableStrictHardwareInterfaceHandle {
 public:
  MutableStrictHardwareInterfaceHandle() = default;

  // Preferred constructor.
  explicit MutableStrictHardwareInterfaceHandle(
      MutableHardwareInterfaceHandle<T> hardware_interface,
      HardwareInterfaceHandle<intrinsic_fbs::IconState> icon_state)
      : hardware_interface_(std::move(hardware_interface)),
        icon_state_(std::move(icon_state)) {}

  // Mutable access to the stored value.
  // Does not check when the value was updated.
  // Call `UpdatedAt` once all values are written to update the cycle
  // information.
  T* MutableValue() { return *hardware_interface_; }

  // Read only access to the stored value.
  // Checks that the value was updated same cycle IconState reports as the
  // current cycle.
  // Returns FailedPreconditionError when not.
  tl::expected<const T*, RealtimeStatus> Value() const {
    if (auto status = WasUpdatedThisCycle(icon_state_, hardware_interface_);
        !status.ok()) {
      return tl::unexpected(status);
    }
    return *hardware_interface_;
  }

  // Returns the number of updates made to the segment. This helps detect issues
  // with missing updates.
  int64_t NumUpdates() const { return hardware_interface_.NumUpdates(); }
  // Returns the time the segment was last updated. This helps detect stale
  // data.
  Time LastUpdatedTime() const { return hardware_interface_.LastUpdatedTime(); }
  // Returns the cycle the segment was last updated. This helps detect stale
  // data.
  uint64_t LastUpdatedCycle() const {
    return hardware_interface_.LastUpdatedCycle();
  }

  // Updates the `time` and current_cycle at which the segment was last updated
  // and increments an update counter.
  void UpdatedAt(Time time) { hardware_interface_.UpdatedAt(time); }

 private:
  MutableHardwareInterfaceHandle<T> hardware_interface_;
  HardwareInterfaceHandle<intrinsic_fbs::IconState> icon_state_;
};
}  // namespace intrinsic::icon
#endif  // ICON_HAL_HARDWARE_INTERFACE_HANDLE_H_
