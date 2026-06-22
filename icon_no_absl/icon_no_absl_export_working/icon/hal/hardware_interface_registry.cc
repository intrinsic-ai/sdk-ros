#include "icon/hal/hardware_interface_registry.h"

#include <stdint.h>

#include <cstring>
#include <string>
#include <string_view>

#include "flatbuffers/detached_buffer.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"

namespace intrinsic::icon {

HardwareInterfaceRegistry::HardwareInterfaceRegistry(
    SharedMemoryManager& shared_memory_manager)
    : shm_manager_(&shared_memory_manager) {}

Status HardwareInterfaceRegistry::AdvertiseInterfaceT(
    std::string_view interface_name, bool must_be_used,
    const flatbuffers::DetachedBuffer& buffer, std::string_view type_id) {
  // Create a shared memory segment that is big enough to hold the SegmentHeader
  // and the flatbuffer payload.
  // TODO(b/217426784): Make `AddSegment` return a pointer to the allocated
  // data when being created successfully.

  INTR_RETURN_STATUS_IF_ERROR(shm_manager_->AddSegment(
      interface_name, must_be_used, buffer.size(), std::string(type_id)));
  uint8_t* const shm_data = shm_manager_->GetRawValue(interface_name);
  std::memcpy(shm_data, buffer.data(), buffer.size());

  return OkStatus();
}
}  // namespace intrinsic::icon
