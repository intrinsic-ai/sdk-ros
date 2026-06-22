#ifndef ICON_HAL_GET_HARDWARE_INTERFACE_H_
#define ICON_HAL_GET_HARDWARE_INTERFACE_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <tl/expected.hpp>
#include <utility>
#include <vector>

#include "flatbuffers/table.h"
#include "flatbuffers/verifier.h"
#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "icon/hal/hardware_interface_handle.h"
#include "icon/hal/hardware_interface_traits.h"
#include "icon/hal/icon_state_register.h"  // IWYU pragma: keep
#include "icon/interprocess/shared_memory_manager/domain_socket_utils.h"
#include "icon/interprocess/shared_memory_manager/memory_segment.h"
#include "icon/interprocess/shared_memory_manager/segment_header.h"
#include "icon/interprocess/shared_memory_manager/segment_info_utils.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/utils/attributes.h"
#include "icon/utils/log.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"

namespace intrinsic::icon {
// LINT.IfChange
static constexpr char kModuleInfoName[] = "intrinsic_module_info";
// LINT.ThenChange(//flatbuffer_definitions/icon/hal/tools/hardware_module_introspection.cc)

// Returns an error if the type or version of the segment header doesn't match
// SegmentHeader::ExpectedVersion().
// The parameter `interface_name` is used for error reporting.
template <class HardwareInterfaceT>
Status SegmentHeaderIsValid(const SegmentHeader& segment_header,
                            std::string_view interface_name) {
  if (segment_header.Version() != SegmentHeader::ExpectedVersion()) {
    return {
        .code = StatusCode::kInvalidArgument,
        .message = (std::stringstream()
                    << "Version mismatch: Interface '" << interface_name
                    << "' has version '" << segment_header.Version()
                    << "' but expected version '"
                    << SegmentHeader::ExpectedVersion() << "'")
                       .str(),
    };
  }

  if (segment_header.Type().TypeID() !=
      hardware_interface_traits::TypeID<HardwareInterfaceT>::kTypeString) {
    return {
        .code = StatusCode::kInvalidArgument,
        .message =
            (std::stringstream()
             << "Type mismatch: Interface '" << interface_name << "' has type '"
             << segment_header.Type().TypeID() << "' but expected type '"
             << hardware_interface_traits::TypeID<
                    HardwareInterfaceT>::kTypeString
             << "'")
                .str(),
    };
  }
  return OkStatus();
}

// This function verifies that flatbuffer tables are internally consistent by
// running the flatbuffer verifier if `HardwareInterfaceT` is a flatbuffer
// table. The verifier checks all offsets, all sizes of fields, and null
// termination of strings to ensure that when a buffer is accessed, all reads
// will end up inside the buffer.
// Skips verification for all other types because they are either a flatbuffer
// struct (and those don't support verification), or not a flatbuffer.
// For those types MemorySegment::Get() already performs all the checking we
// can do. For flatbuffer structs, that simpler check is correct, but for
// non-flatbuffer types, it might not be.
//
// `HardwareInterfaceT` is the type of the data stored in the shared memory
// segment.
//
// The parameter `buffer` is a pointer to the start of the payload of a hardware
// interface's shared memory segment.
// The parameter `buffer_size` is the size of the buffer.
// The parameter `interface_name` is only used for error reporting.
//
// Returns InvalidArgumentError if `HardwareInterfaceT` is a flatbuffer table,
// but the buffer failed verification.
template <class HardwareInterfaceT>
Status FlatbufferIsValid(const uint8_t* buffer, size_t buffer_size,
                         std::string_view interface_name) {
  // Only run verifier if `HardwareInterfaceT` is a flatbuffer table.
  if constexpr (std::is_base_of_v<flatbuffers::Table, HardwareInterfaceT>) {
    flatbuffers::Verifier verifier(/*buf=*/buffer,
                                   /*size*/ buffer_size);
    if (!verifier.VerifyBuffer<HardwareInterfaceT>()) {
      return Status{
          .code = StatusCode::kInvalidArgument,
          .message =
              (std::stringstream()
               << "Flatbuffer verification failed for interface '"
               << interface_name
               << "'. This can be due to a version mismatch of your resources.")
                  .str(),
      };
    }
  }
  return OkStatus();
}

// Returns a handle to a HardwareInterfaceT.
// The HardwareInterfaceT must be registered with the
// INTRINSIC_ADD_HARDWARE_INTERFACE macro.
template <class HardwareInterfaceT>
inline tl::expected<HardwareInterfaceHandle<HardwareInterfaceT>, Status>
GetInterfaceHandle(
    const SegmentNameToFileDescriptorMap& segment_name_to_file_descriptor_map,
    std::string_view interface_name) {
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto ro_segment,
      ReadOnlyMemorySegment<HardwareInterfaceT>::Get(
          segment_name_to_file_descriptor_map, interface_name));
  INTR_RETURN_UNEXPECTED_IF_ERROR(SegmentHeaderIsValid<HardwareInterfaceT>(
      ro_segment.Header(), interface_name));
  INTR_RETURN_UNEXPECTED_IF_ERROR(FlatbufferIsValid<HardwareInterfaceT>(
      ro_segment.GetRawValue(), ro_segment.ValueSize(), interface_name));

  return HardwareInterfaceHandle<HardwareInterfaceT>(std::move(ro_segment));
}

// Returns a mutable handle to a HardwareInterfaceT.
// The HardwareInterfaceT must be registered with the
// INTRINSIC_ADD_HARDWARE_INTERFACE macro.
template <class HardwareInterfaceT>
inline tl::expected<MutableHardwareInterfaceHandle<HardwareInterfaceT>, Status>
GetMutableInterfaceHandle(
    const SegmentNameToFileDescriptorMap& segment_name_to_file_descriptor_map,
    std::string_view interface_name) {
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto rw_segment,
      ReadWriteMemorySegment<HardwareInterfaceT>::Get(
          segment_name_to_file_descriptor_map, interface_name));
  INTR_RETURN_UNEXPECTED_IF_ERROR(SegmentHeaderIsValid<HardwareInterfaceT>(
      rw_segment.Header(), interface_name));
  INTR_RETURN_UNEXPECTED_IF_ERROR(FlatbufferIsValid<HardwareInterfaceT>(
      rw_segment.GetRawValue(), rw_segment.ValueSize(), interface_name));

  return MutableHardwareInterfaceHandle<HardwareInterfaceT>(
      std::move(rw_segment));
}

// Returns a handle to a HardwareInterfaceT.
// The HardwareInterfaceT must be registered with the
// INTRINSIC_ADD_HARDWARE_INTERFACE macro.
template <class HardwareInterfaceT>
inline tl::expected<HardwareInterfaceHandle<HardwareInterfaceT>, Status>
GetInterfaceHandle(const SharedMemoryManager& shared_memory_manager,
                   std::string_view interface_name,
                   const log::Logger* logger INTR_ATTRIBUTE_LIFETIME_BOUND) {
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto ro_segment,
      shared_memory_manager.Get<ReadOnlyMemorySegment<HardwareInterfaceT>>(
          interface_name, logger));
  INTR_RETURN_UNEXPECTED_IF_ERROR(SegmentHeaderIsValid<HardwareInterfaceT>(
      ro_segment.Header(), interface_name));
  INTR_RETURN_UNEXPECTED_IF_ERROR(FlatbufferIsValid<HardwareInterfaceT>(
      ro_segment.GetRawValue(), ro_segment.ValueSize(), interface_name));

  return HardwareInterfaceHandle<HardwareInterfaceT>(std::move(ro_segment));
}

// Returns a mutable handle to a HardwareInterfaceT.
// The HardwareInterfaceT must be registered with the
// INTRINSIC_ADD_HARDWARE_INTERFACE macro.
template <class HardwareInterfaceT>
inline tl::expected<MutableHardwareInterfaceHandle<HardwareInterfaceT>, Status>
GetMutableInterfaceHandle(const SharedMemoryManager& shared_memory_manager,
                          std::string_view interface_name,
                          const log::Logger* logger
                              INTR_ATTRIBUTE_LIFETIME_BOUND) {
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto rw_segment,
      shared_memory_manager.Get<ReadWriteMemorySegment<HardwareInterfaceT>>(
          interface_name, logger));
  INTR_RETURN_UNEXPECTED_IF_ERROR(SegmentHeaderIsValid<HardwareInterfaceT>(
      rw_segment.Header(), interface_name));
  INTR_RETURN_UNEXPECTED_IF_ERROR(FlatbufferIsValid<HardwareInterfaceT>(
      rw_segment.GetRawValue(), rw_segment.ValueSize(), interface_name));

  return MutableHardwareInterfaceHandle<HardwareInterfaceT>(
      std::move(rw_segment));
}

// Returns a handle to a HardwareInterfaceT that checks that it was
// updated in the same ICON cycle as the "icon_state" interface.
// The HardwareInterfaceT must be registered with the
// INTRINSIC_ADD_HARDWARE_INTERFACE macro.
template <class HardwareInterfaceT>
inline tl::expected<StrictHardwareInterfaceHandle<HardwareInterfaceT>, Status>
GetStrictInterfaceHandle(const SharedMemoryManager& shared_memory_manager,
                         std::string_view interface_name,
                         const log::Logger* logger
                             INTR_ATTRIBUTE_LIFETIME_BOUND) {
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto handle, GetInterfaceHandle<HardwareInterfaceT>(
                       shared_memory_manager, interface_name, logger));
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto icon_state,
      GetInterfaceHandle<intrinsic_fbs::IconState>(
          shared_memory_manager, kIconStateInterfaceName, logger));

  return StrictHardwareInterfaceHandle<HardwareInterfaceT>(
      std::move(handle), std::move(icon_state));
}

// Returns a mutable handle to a HardwareInterfaceT that checks that it was
// updated in the same ICON cycle as the "icon_state" interface.
// The HardwareInterfaceT must be registered with the
// INTRINSIC_ADD_HARDWARE_INTERFACE macro.
template <class HardwareInterfaceT>
inline tl::expected<MutableStrictHardwareInterfaceHandle<HardwareInterfaceT>,
                    Status>
GetMutableStrictInterfaceHandle(
    const SharedMemoryManager& shared_memory_manager,
    std::string_view interface_name,
    const log::Logger* logger INTR_ATTRIBUTE_LIFETIME_BOUND) {
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto handle, GetMutableInterfaceHandle<HardwareInterfaceT>(
                       shared_memory_manager, interface_name, logger));
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      auto icon_state,
      GetInterfaceHandle<intrinsic_fbs::IconState>(
          shared_memory_manager, kIconStateInterfaceName, logger));

  return MutableStrictHardwareInterfaceHandle<HardwareInterfaceT>(
      std::move(handle), std::move(icon_state));
}

// Returns information about the exported interfaces from a hardware module.
inline tl::expected<ReadOnlyMemorySegment<intrinsic_fbs::SegmentInfo>, Status>
GetHardwareModuleInfo(
    const SegmentNameToFileDescriptorMap& segment_name_to_file_descriptor_map,
    const log::Logger* logger INTR_ATTRIBUTE_LIFETIME_BOUND) {
  return ReadOnlyMemorySegment<intrinsic_fbs::SegmentInfo>::Get(
      segment_name_to_file_descriptor_map, icon::kModuleInfoName, logger);
}

// Extracts the names of the shared memory segments.
inline tl::expected<std::vector<std::string>, Status>
GetInterfacesFromModuleInfo(const intrinsic_fbs::SegmentInfo& segment_info) {
  return GetNamesFromSegmentInfo(segment_info);
}

// Extracts the names of the shared memory segments that are marked as required.
//
// Subset of GetInterfacesFromModuleInfo.
inline tl::expected<std::vector<std::string>, Status>
GetRequiredInterfacesFromModuleInfo(
    const intrinsic_fbs::SegmentInfo& segment_info) {
  return GetRequiredInterfaceNamesFromSegmentInfo(segment_info);
}

}  // namespace intrinsic::icon
#endif  // ICON_HAL_GET_HARDWARE_INTERFACE_H_
