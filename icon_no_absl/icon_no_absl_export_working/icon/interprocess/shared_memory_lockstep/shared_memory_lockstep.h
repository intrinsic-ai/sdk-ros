#ifndef ICON_INTERPROCESS_SHARED_MEMORY_LOCKSTEP_SHARED_MEMORY_LOCKSTEP_H_
#define ICON_INTERPROCESS_SHARED_MEMORY_LOCKSTEP_SHARED_MEMORY_LOCKSTEP_H_

#include <cstdlib>
#include <iostream>
#include <string_view>
#include <tl/expected.hpp>
#include <utility>

#include "icon/interprocess/shared_memory_manager/memory_segment.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/utils/attributes.h"
#include "icon/utils/log.h"
#include "icon/utils/status.h"
#include "util/thread/lockstep.h"

namespace intrinsic::icon {

// SharedMemoryLockstep is a Lockstep synchronization primitive that is stored
// in shared memory. This can be used for synchronization across process
// boundaries.
class SharedMemoryLockstep {
 public:
  // Null SharedMemoryLockstep. Dereferencing this will check-fail. This allows
  // value semantics / move to work.
  SharedMemoryLockstep() : memory_segment_(), lockstep_(nullptr) {}

  // Creates a SharedMemoryLockstep from a Lockstep memory segment. Prefer to
  // use CreateSharedMemoryLockstep or GetSharedMemoryLockstep instead.
  explicit SharedMemoryLockstep(
      intrinsic::icon::ReadWriteMemorySegment<Lockstep>&& segment,
      const log::Logger* logger INTR_ATTRIBUTE_LIFETIME_BOUND)
      : logger_(logger),
        memory_segment_(std::move(segment)),
        lockstep_(&memory_segment_.GetValue()) {}

  // Returns true if the lockstep is attached to two instances.
  bool Connected() const;

  // Obtains the underlying shared memory Lockstep object that can be used for
  // synchronization. Returns nullptr if this is null (default-constructed).
  Lockstep* GetLockstep() { return lockstep_; }

  // Dereferencing returns the underlying Lockstep object. Check-fails if this
  // is null (default-constructed).
  Lockstep* operator*() {
    if (lockstep_ == nullptr) {
      INTRINSIC_SHARED_MEMORY_LOG(FATAL, logger_,
                                  "null SharedMemoryLockstep dereferenced");
    }
    return lockstep_;
  }
  const Lockstep* operator*() const {
    if (lockstep_ == nullptr) {
      INTRINSIC_SHARED_MEMORY_LOG(FATAL, logger_,
                                  "null SharedMemoryLockstep dereferenced");
    }
    return lockstep_;
  }
  Lockstep* operator->() {
    if (lockstep_ == nullptr) {
      INTRINSIC_SHARED_MEMORY_LOG(FATAL, logger_,
                                  "null SharedMemoryLockstep dereferenced");
    }
    return lockstep_;
  }
  const Lockstep* operator->() const {
    if (lockstep_ == nullptr) {
      INTRINSIC_SHARED_MEMORY_LOG(FATAL, logger_,
                                  "null SharedMemoryLockstep dereferenced");
    }
    return lockstep_;
  }

 private:
  const log::Logger* logger_ = nullptr;
  // Hold onto the memory segment, since it is refcounted.
  intrinsic::icon::ReadWriteMemorySegment<Lockstep> memory_segment_;
  // Raw pointer into the memory segment, for convenience.
  Lockstep* lockstep_;
};

// Creates a SharedMemoryLockstep whose shared memory is managed by `manager`
// and is stored in a segment named `memory_name`. The `manager` must outlive
// the returned SharedMemoryLockstep.
tl::expected<SharedMemoryLockstep, Status> CreateSharedMemoryLockstep(
    intrinsic::icon::SharedMemoryManager& manager INTR_ATTRIBUTE_LIFETIME_BOUND,
    std::string_view memory_name,
    const log::Logger* logger INTR_ATTRIBUTE_LIFETIME_BOUND);

// Convenience method for tests.
// Returns a SharedMemoryLockstep in a shared memory segment named
// `memory_name` that was created on `manager`.
// The `manager` must outlive the returned SharedMemoryLockstep.
tl::expected<SharedMemoryLockstep, Status> GetSharedMemoryLockstep(
    const intrinsic::icon::SharedMemoryManager& manager
        INTR_ATTRIBUTE_LIFETIME_BOUND,
    std::string_view memory_name,
    const log::Logger* logger INTR_ATTRIBUTE_LIFETIME_BOUND);

}  // namespace intrinsic::icon
#endif  // ICON_INTERPROCESS_SHARED_MEMORY_LOCKSTEP_SHARED_MEMORY_LOCKSTEP_H_
