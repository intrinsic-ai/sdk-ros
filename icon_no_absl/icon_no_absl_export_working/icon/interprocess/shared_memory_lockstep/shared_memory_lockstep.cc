#include "icon/interprocess/shared_memory_lockstep/shared_memory_lockstep.h"

#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"

namespace intrinsic::icon {

namespace {

tl::expected<SharedMemoryLockstep, Status> GetSharedMemoryLockstep(
    const SegmentNameToFileDescriptorMap& segment_name_to_file_descriptor_map,
    std::string_view memory_name, const log::Logger* logger) {
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto segment,
      ReadWriteMemorySegment<Lockstep>::Get(segment_name_to_file_descriptor_map,
                                            memory_name, logger));
  return SharedMemoryLockstep(std::move(segment), logger);
}

}  // namespace

bool SharedMemoryLockstep::Connected() const {
  if (!memory_segment_.IsValid()) {
    return false;
  }
  return memory_segment_.Header().WriterRefCount() == 2;
}

tl::expected<SharedMemoryLockstep, Status> CreateSharedMemoryLockstep(
    SharedMemoryManager& manager, std::string_view memory_name,
    const log::Logger* logger) {
  INTR_RETURN_UNEXPECTED_IF_ERROR(
      manager.AddSegment(memory_name, false, Lockstep()));

  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto segment,
      manager.Get<ReadWriteMemorySegment<Lockstep>>(memory_name, logger));

  return SharedMemoryLockstep(std::move(segment), logger);
}

tl::expected<SharedMemoryLockstep, Status> GetSharedMemoryLockstep(
    const SharedMemoryManager& manager, std::string_view memory_name,
    const log::Logger* logger) {
  return GetSharedMemoryLockstep(manager.SegmentNameToFileDescriptorMap(),
                                 memory_name, logger);
}

}  // namespace intrinsic::icon
