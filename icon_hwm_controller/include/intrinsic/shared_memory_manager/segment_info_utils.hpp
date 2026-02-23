#pragma once

// This header file provides a few convenience functions to access general
// information about a set of shared memory data.
// It specifically provides access to a `SegmentInfo` struct as defined in
// `segment_info.fbs`.

#include <string>
#include <vector>
#include <tl/expected.hpp>

#include "intrinsic/utils/status.hpp"
#include "hwm_fbs/segment_info.fbs.h"

namespace intrinsic::hal
{

// Extracts the SegmentNames from the SegmentInfo struct.
tl::expected<std::vector<std::string>, Status> GetNamesFromSegmentInfo(
  const intrinsic_fbs::SegmentInfo & segment_info);

// Extracts the SegmentNames from the FileDescriptorNames struct.
tl::expected<std::vector<std::string>, Status> GetNamesFromFileDescriptorNames(
  const intrinsic_fbs::FileDescriptorNames & file_descriptor_names);

// Extracts the SegmentNames that are marked as required from the SegmentInfo
// struct.
tl::expected<std::vector<std::string>, Status>
GetRequiredInterfaceNamesFromSegmentInfo(
  const intrinsic_fbs::SegmentInfo & segment_info);

}  // namespace intrinsic::hal
