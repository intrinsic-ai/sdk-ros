#ifndef ICON_HAL_INTERFACES_JOINT_STATE_UTILS_H_
#define ICON_HAL_INTERFACES_JOINT_STATE_UTILS_H_

#include <cstdint>

#include "flatbuffers/detached_buffer.h"
#include "icon/utils/attributes.h"

namespace intrinsic_fbs {

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointPositionState(
    uint32_t num_dof);

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointVelocityState(
    uint32_t num_dof);

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointAccelerationState(
    uint32_t num_dof);

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointTorqueState(
    uint32_t num_dof);

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointCommandedPosition(
    uint32_t num_dof);

}  // namespace intrinsic_fbs
#endif  // ICON_HAL_INTERFACES_JOINT_STATE_UTILS_H_
