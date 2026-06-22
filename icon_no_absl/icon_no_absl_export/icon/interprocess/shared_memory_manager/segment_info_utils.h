#ifndef ICON_INTERPROCESS_SHARED_MEMORY_MANAGER_SEGMENT_INFO_UTILS_H_
#define ICON_INTERPROCESS_SHARED_MEMORY_MANAGER_SEGMENT_INFO_UTILS_H_
// This header file provides a few convenience functions to access general
// information about a set of shared memory data.
// It specifically provides access to a `SegmentInfo` struct as defined in
// `segment_info.fbs`.

#include <string>
#include <tl/expected.hpp>
#include <vector>

#include "flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "icon/utils/status.h"

namespace intrinsic::icon {

// Extracts the SegmentNames from the SegmentInfo struct.
tl::expected<std::vector<std::string>, Status> GetNamesFromSegmentInfo(
    const intrinsic_fbs::SegmentInfo& segment_info);

// Extracts the SegmentNames from the FileDescriptorNames struct.
tl::expected<std::vector<std::string>, Status> GetNamesFromFileDescriptorNames(
    const intrinsic_fbs::FileDescriptorNames& file_descriptor_names);

// Extracts the SegmentNames that are marked as required from the SegmentInfo
// struct.
tl::expected<std::vector<std::string>, Status>
GetRequiredInterfaceNamesFromSegmentInfo(
    const intrinsic_fbs::SegmentInfo& segment_info);

}  // namespace intrinsic::icon
#endif  // ICON_INTERPROCESS_SHARED_MEMORY_MANAGER_SEGMENT_INFO_UTILS_H_
