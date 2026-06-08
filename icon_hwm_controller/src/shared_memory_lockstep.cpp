#include "intrinsic/shared_memory_manager/shared_memory_lockstep.hpp"

namespace intrinsic::hal
{


namespace
{

tl::expected<SharedMemoryLockstep, Status> GetSharedMemoryLockstep(
  const SegmentNameToFileDescriptorMap & segment_name_to_file_descriptor_map,
  std::string_view memory_name,
  const log::Logger * logger INTR_ATTRIBUTE_LIFETIME_BOUND)
{
  auto segment = ReadWriteMemorySegment<Lockstep>::Get(
      segment_name_to_file_descriptor_map, memory_name, logger);
  if (!segment.has_value()) {
    return tl::unexpected(segment.error());
  }
  return SharedMemoryLockstep(std::move(segment.value()), logger);
}

}  // namespace

  // Implementation of Connected is inline in header.


tl::expected<SharedMemoryLockstep, Status> CreateSharedMemoryLockstep(
  SharedMemoryManager & manager, std::string_view memory_name, const log::Logger * logger)
{
  if(auto status = manager.AddSegment(memory_name, false, Lockstep()); !status.ok()) {
    return tl::unexpected(status);
  }

  auto segment = manager.Get<ReadWriteMemorySegment<Lockstep>>(memory_name, logger);
  if (!segment.has_value()) {
    return tl::unexpected(segment.error());
  }

  return SharedMemoryLockstep(std::move(segment.value()), logger);
}

tl::expected<SharedMemoryLockstep, Status> GetSharedMemoryLockstep(
  const SharedMemoryManager & manager, std::string_view memory_name, const log::Logger * logger)
{
  return GetSharedMemoryLockstep(manager.SegmentNameToFileDescriptorMap(),
                                 memory_name,
                                 logger);
}


}  // namespace intrinsic::hal
