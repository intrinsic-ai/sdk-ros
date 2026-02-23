#pragma once

#include <cstdint>

#include "intrinsic/utils/status.hpp"
#include "flatbuffers/detached_buffer.h"
#include "intrinsic/hal/hardware_interface_handle.hpp"
#include "hwm_fbs/joint_limits.fbs.h"
#include "intrinsic/kinematics/types/joint_limits.hpp"
// #include "intrinsic/kinematics/types/joint_limits.pb.h"

namespace intrinsic_fbs
{

flatbuffers::DetachedBuffer BuildJointLimits(uint32_t num_dof);

}  // namespace intrinsic_fbs

namespace intrinsic::hal
{

#if 0

// Parses a JointLimits protobuf into a JointLimits hardware interface handle.
// Returns kInvalidArgument if the number of joints in the non-empty protobuf
// fields are not equal to the ones in the flatbuffer handle. Expects all
// non-empty JointLimits proto fields to have the same size and each flatbuffer
// field to match that size.
absl::Status ParseProtoJointLimits(
  const intrinsic_proto::JointLimits & pb_limits,
  intrinsic::icon::MutableHardwareInterfaceHandle<intrinsic_fbs::JointLimits> &
  fb_limits);

#endif

// Copies a JointLimits struct to a JointLimits flatbuffer. Fails if the number
// of joints in the struct does not match the size of the flatbuffer.
RealtimeStatus CopyTo(
  const JointLimits & limits,
  intrinsic_fbs::JointLimits & fb_limits);

// Copies a JointLimits flatbuffer to a JointLimits struct.
RealtimeStatus CopyTo(
  const intrinsic_fbs::JointLimits & fb_limits,
  JointLimits & limits);

}  // namespace intrinsic::hal
