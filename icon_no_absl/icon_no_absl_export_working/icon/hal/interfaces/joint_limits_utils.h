#ifndef ICON_HAL_INTERFACES_JOINT_LIMITS_UTILS_H_
#define ICON_HAL_INTERFACES_JOINT_LIMITS_UTILS_H_

#include <cstdint>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
#include "icon/hal/hardware_interface_handle.h"
#include "icon/utils/attributes.h"
#include "icon/utils/status.h"
#include "kinematics/types/joint_limits.h"

namespace intrinsic_fbs {

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointLimits(
    uint32_t num_dof);

}  // namespace intrinsic_fbs

namespace intrinsic::icon {

// Copies a JointLimits struct to a JointLimits flatbuffer. Fails if the number
// of joints in the struct does not match the size of the flatbuffer.
INTR_MUST_USE_RESULT RealtimeStatus
CopyTo(const JointLimits& limits, intrinsic_fbs::JointLimits& fb_limits);

// Copies a JointLimits flatbuffer to a JointLimits struct.
INTR_MUST_USE_RESULT RealtimeStatus
CopyTo(const intrinsic_fbs::JointLimits& fb_limits, JointLimits& limits);

}  // namespace intrinsic::icon
#endif  // ICON_HAL_INTERFACES_JOINT_LIMITS_UTILS_H_
