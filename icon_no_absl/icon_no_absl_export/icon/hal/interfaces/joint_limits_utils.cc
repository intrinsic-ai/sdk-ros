#include "icon/hal/interfaces/joint_limits_utils.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
#include "icon/hal/hardware_interface_handle.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"
#include "kinematics/types/joint_limits.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildJointLimits(uint32_t num_dof) {
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);

  std::vector<double> zeros(num_dof, 0.0);
  auto min_pos = builder.CreateVector(zeros);
  auto max_pos = builder.CreateVector(zeros);
  auto max_vel = builder.CreateVector(zeros);
  auto max_acc = builder.CreateVector(zeros);
  auto max_jerk = builder.CreateVector(zeros);
  auto max_effort = builder.CreateVector(zeros);
  auto joint_limits =
      CreateJointLimits(builder, min_pos, max_pos, false, max_vel, false,
                        max_acc, false, max_jerk, false, max_effort);
  builder.Finish(joint_limits);
  return builder.Release();
}

}  // namespace intrinsic_fbs

namespace intrinsic::icon {

// Checks that a std::span<double> and a Vector<double> from a
// flatbuffer have the same size. The field name is only used for the error
// message if sizes are not equal. Returns InvalidArgumentError otherwise.
RealtimeStatus CheckSizeEqual(std::span<const double> field,
                              const flatbuffers::Vector<double>& fb_field,
                              std::string_view field_name) {
  int fb_num_joints = fb_field.size();
  int num_joints = field.size();
  if (fb_num_joints != num_joints) {
    auto status = RealtimeStatus{
        .code = StatusCode::kInvalidArgument,
    };
    (void)std::snprintf(status.message.data(), status.message.size(),
                        "JointLimits Flatbuffer expects %d joints but the "
                        "field '%.*s' contains %d values",
                        fb_num_joints,
                        static_cast<int>(std::min(
                            static_cast<std::string_view::size_type>(INT_MAX),
                            field_name.size())),
                        field_name.data(), num_joints);
    return status;
  }
  return RtOkStatus();
}

RealtimeStatus ToFlatbufferWithSizeCheck(std::span<const double> field,
                                         flatbuffers::Vector<double>& fb_field,
                                         std::string_view field_name) {
  if (const auto status = CheckSizeEqual(field, fb_field, field_name);
      !status.ok()) {
    return status;
  }
  for (size_t i = 0; i < fb_field.size(); i++) {
    fb_field.Mutate(i, field[i]);
  }
  return RtOkStatus();
}

RealtimeStatus CopyTo(const JointLimits& limits,
                      intrinsic_fbs::JointLimits& fb_limits) {
  INTR_RETURN_STATUS_IF_ERROR(ToFlatbufferWithSizeCheck(
      std::span<const double>(limits.min_position.data(),
                              limits.min_position.size()),
      *fb_limits.mutable_min_position(), "min_position"));
  INTR_RETURN_STATUS_IF_ERROR(ToFlatbufferWithSizeCheck(
      std::span<const double>(limits.max_position.data(),
                              limits.max_position.size()),
      *fb_limits.mutable_max_position(), "max_position"));
  INTR_RETURN_STATUS_IF_ERROR(ToFlatbufferWithSizeCheck(
      std::span<const double>(limits.max_velocity.data(),
                              limits.max_velocity.size()),
      *fb_limits.mutable_max_velocity(), "max_velocity"));
  INTR_RETURN_STATUS_IF_ERROR(ToFlatbufferWithSizeCheck(
      std::span<const double>(limits.max_acceleration.data(),
                              limits.max_acceleration.size()),
      *fb_limits.mutable_max_acceleration(), "max_acceleration"));
  INTR_RETURN_STATUS_IF_ERROR(ToFlatbufferWithSizeCheck(
      std::span<const double>(limits.max_jerk.data(), limits.max_jerk.size()),
      *fb_limits.mutable_max_jerk(), "max_jerk"));
  INTR_RETURN_STATUS_IF_ERROR(ToFlatbufferWithSizeCheck(
      std::span<const double>(limits.max_torque.data(),
                              limits.max_torque.size()),
      *fb_limits.mutable_max_effort(), "max_effort"));

  fb_limits.mutate_has_velocity_limits(true);
  fb_limits.mutate_has_acceleration_limits(true);
  fb_limits.mutate_has_jerk_limits(true);
  fb_limits.mutate_has_effort_limits(true);

  return RtOkStatus();
}

RealtimeStatus CopyTo(const intrinsic_fbs::JointLimits& fb_limits,
                      JointLimits& limits) {
  const size_t num_joints = fb_limits.min_position()->size();
  INTR_RETURN_STATUS_IF_ERROR(limits.SetSize(num_joints));
  for (size_t i = 0; i < num_joints; ++i) {
    limits.min_position[i] = fb_limits.min_position()->Get(i);
    limits.max_position[i] = fb_limits.max_position()->Get(i);
    if (fb_limits.has_velocity_limits()) {
      limits.max_velocity[i] = fb_limits.max_velocity()->Get(i);
    }
    if (fb_limits.has_acceleration_limits()) {
      limits.max_acceleration[i] = fb_limits.max_acceleration()->Get(i);
    }
    if (fb_limits.has_jerk_limits()) {
      limits.max_jerk[i] = fb_limits.max_jerk()->Get(i);
    }
    if (fb_limits.has_effort_limits()) {
      limits.max_torque[i] = fb_limits.max_effort()->Get(i);
    }
  }
  return RtOkStatus();
}

}  // namespace intrinsic::icon
