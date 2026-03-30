#pragma once

#include <cstdint>

#include "flatbuffers/detached_buffer.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildJointPositionState(uint32_t num_dof);

flatbuffers::DetachedBuffer BuildJointVelocityState(uint32_t num_dof);

flatbuffers::DetachedBuffer BuildJointAccelerationState(uint32_t num_dof);

flatbuffers::DetachedBuffer BuildJointTorqueState(uint32_t num_dof);

flatbuffers::DetachedBuffer BuildJointCommandedPosition(uint32_t num_dof);

}  // namespace intrinsic_fbs