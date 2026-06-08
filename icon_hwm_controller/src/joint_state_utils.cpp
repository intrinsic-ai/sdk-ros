#include "intrinsic/hal/joint_state_utils.hpp"

#include <cstdint>
#include <vector>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "hwm_fbs/joint_state.fbs.h"

namespace intrinsic_fbs
{

flatbuffers::DetachedBuffer BuildJointPositionState(uint32_t num_dof)
{
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);

  std::vector<double> zeros(num_dof, 0.0);
  auto default_pos = builder.CreateVector(zeros);
  auto position_state = CreateJointPositionState(builder, default_pos);
  builder.Finish(position_state);
  return builder.Release();
}

flatbuffers::DetachedBuffer BuildJointVelocityState(uint32_t num_dof)
{
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);

  std::vector<double> zeros(num_dof, 0.0);
  auto default_vel = builder.CreateVector(zeros);
  auto velocity_state = CreateJointVelocityState(builder, default_vel);
  builder.Finish(velocity_state);
  return builder.Release();
}

flatbuffers::DetachedBuffer BuildJointAccelerationState(uint32_t num_dof)
{
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);

  std::vector<double> zeros(num_dof, 0.0);
  auto default_acc = builder.CreateVector(zeros);
  auto acceleration_state = CreateJointAccelerationState(builder, default_acc);
  builder.Finish(acceleration_state);
  return builder.Release();
}

flatbuffers::DetachedBuffer BuildJointTorqueState(uint32_t num_dof)
{
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);

  std::vector<double> zeros(num_dof, 0.0);
  auto default_torque = builder.CreateVector(zeros);
  auto torque_state = CreateJointTorqueState(builder, default_torque);
  builder.Finish(torque_state);
  return builder.Release();
}

flatbuffers::DetachedBuffer BuildJointCommandedPosition(uint32_t num_dof)
{
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);

  std::vector<double> zeros(num_dof, 0.0);
  auto default_pos = builder.CreateVector(zeros);
  auto default_ff_vel = builder.CreateVector(zeros);
  auto default_ff_acc = builder.CreateVector(zeros);
  auto position_command = CreateJointCommandedPosition(
      builder, default_pos, default_ff_vel, default_ff_acc);
  builder.Finish(position_command);
  return builder.Release();
}

}  // namespace intrinsic_fbs
