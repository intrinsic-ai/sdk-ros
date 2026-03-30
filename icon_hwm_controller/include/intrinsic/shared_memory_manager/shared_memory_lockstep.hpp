#pragma once

#include <utility>
#include <cassert>
#include "intrinsic/shared_memory_manager/lockstep.hpp"
#include "intrinsic/shared_memory_manager/memory_segment.hpp"
#include "intrinsic/shared_memory_manager/shared_memory_manager.hpp"
#include "intrinsic/utils/status.hpp"
#include <string_view>
#include <tl/expected.hpp>

namespace intrinsic::hal {

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
  explicit SharedMemoryLockstep(intrinsic::hal::ReadWriteMemorySegment<Lockstep>&& segment)
      : memory_segment_(std::move(segment)),
        lockstep_(&memory_segment_.GetValue()) {}

  // Returns true if the lockstep is attached to two instances.
  bool Connected() const { return lockstep_ != nullptr; }
  
  // Obtains the underlying shared memory Lockstep object that can be used for
  // synchronization. Returns nullptr if this is null (default-constructed).
  Lockstep* GetLockstep() { return lockstep_; }


  // Dereferencing returns the underlying Lockstep object. Check-fails if this
  // is null (default-constructed).
  Lockstep* operator*() {
    assert(lockstep_ != nullptr && "null SharedMemoryLockstep dereferenced");
    return lockstep_;
  }
  const Lockstep* operator*() const {
    assert(lockstep_ != nullptr && "null SharedMemoryLockstep dereferenced");
    return lockstep_;
  }
  Lockstep* operator->() {
    assert(lockstep_ != nullptr && "null SharedMemoryLockstep dereferenced");
    return lockstep_;
  }
  const Lockstep* operator->() const {
    assert(lockstep_ != nullptr && "null SharedMemoryLockstep dereferenced");
    return lockstep_;
  }

 private:
  // Hold onto the memory segment, since it is refcounted.
  intrinsic::hal::ReadWriteMemorySegment<Lockstep> memory_segment_;
  // Raw pointer into the memory segment, for convenience.
  Lockstep* lockstep_;
};


// Creates a SharedMemoryLockstep whose shared memory is managed by `manager`
// and is stored in a segment named `memory_name`. The `manager` must outlive
// the returned SharedMemoryLockstep.
tl::expected<SharedMemoryLockstep, Status> CreateSharedMemoryLockstep(
    intrinsic::hal::SharedMemoryManager& manager, std::string_view memory_name);

// Convenience method for tests.
// Returns a SharedMemoryLockstep in a shared memory segment named
// `memory_name` that was created on `manager`.
// The `manager` must outlive the returned SharedMemoryLockstep.
tl::expected<SharedMemoryLockstep, Status> GetSharedMemoryLockstep(
    const intrinsic::hal::SharedMemoryManager& manager, std::string_view memory_name);

}  // namespace intrinsic::hal
