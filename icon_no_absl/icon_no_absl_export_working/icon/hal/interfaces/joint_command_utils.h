#ifndef ICON_HAL_INTERFACES_JOINT_COMMAND_UTILS_H_
#define ICON_HAL_INTERFACES_JOINT_COMMAND_UTILS_H_

#include <cstdint>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
#include "icon/utils/attributes.h"
#include "icon/utils/status.h"

namespace intrinsic_fbs {

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointPositionCommand(
    uint32_t num_dof);

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointVelocityCommand(
    uint32_t num_dof);

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildJointTorqueCommand(
    uint32_t num_dof);

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer
BuildJointAccelerationAndTorqueCommand(uint32_t num_dof);

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildHandGuidingCommand();

INTR_MUST_USE_RESULT intrinsic::RealtimeStatus CopyTo(
    const JointPositionCommand& src, JointPositionCommand& dest);

}  // namespace intrinsic_fbs
#endif  // ICON_HAL_INTERFACES_JOINT_COMMAND_UTILS_H_
