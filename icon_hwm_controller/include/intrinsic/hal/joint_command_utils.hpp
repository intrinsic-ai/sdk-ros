#pragma once


#include <cstdint>

#include "flatbuffers/detached_buffer.h"
#include "hwm_fbs/joint_command.fbs.h"
#include "intrinsic/utils/status.hpp"

namespace intrinsic_fbs
{

flatbuffers::DetachedBuffer BuildJointPositionCommand(uint32_t num_dof);

flatbuffers::DetachedBuffer BuildJointVelocityCommand(uint32_t num_dof);

flatbuffers::DetachedBuffer BuildJointTorqueCommand(uint32_t num_dof);

flatbuffers::DetachedBuffer BuildJointAccelerationAndTorqueCommand(
  uint32_t num_dof);

flatbuffers::DetachedBuffer BuildHandGuidingCommand();

intrinsic::RealtimeStatus CopyTo(
  const JointPositionCommand & src,
  JointPositionCommand & dest);

}  // namespace intrinsic_fbs
