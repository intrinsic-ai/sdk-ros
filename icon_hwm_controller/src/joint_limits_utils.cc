#include "intrinsic/hal/joint_limits_utils.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string_view>
#include <span>

#include "intrinsic/utils/status.hpp"
#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "intrinsic/hal/hardware_interface_handle.hpp"
#include "hwm_fbs/joint_limits.fbs.h"
#include "intrinsic/kinematics/types/joint_limits.hpp"

namespace intrinsic_fbs {

  flatbuffers::DetachedBuffer BuildJointLimits(uint32_t num_dof)
  {
    flatbuffers::FlatBufferBuilder builder;
    builder.ForceDefaults(true);

    std::vector < double > zeros(num_dof, 0.0);
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

namespace intrinsic::hal {

// Checks that a std::span<double> and a Vector<double> from a
// flatbuffer have the same size. The field name is only used for the error
// message if sizes are not equal. Returns InvalidArgumentError otherwise.
  RealtimeStatus CheckSizeEqual(
    std::span < const double > field,
    const flatbuffers::Vector < double > &fb_field,
    std::string_view field_name)
  {
    int fb_num_joints = fb_field.size();
    int num_joints = field.size();
    if (fb_num_joints != num_joints) {
      auto status = RealtimeStatus {
        .code = StatusCode::kInvalidArgument,
      };
      (void) std::snprintf(
        status.message.data(),
        status.message.size(),
        "JointLimits Flatbuffer expects %d joints but the field '%.*s' contains %d values",
        fb_num_joints, field_name.size(), field_name.data(), num_joints);
      return status;
    }
    return RtOkStatus();
  }

  RealtimeStatus ToFlatbufferWithSizeCheck(
    std::span < const double > field,
    flatbuffers::Vector < double > &fb_field,
    std::string_view field_name)
  {
    if(const auto status = CheckSizeEqual(field, fb_field, field_name); !status.ok()) {
      return status;
    }
    for (int i = 0; i < fb_field.size(); i++) {
      fb_field.Mutate(i, field[i]);
    }
    return RtOkStatus();
  }

  RealtimeStatus CopyTo(
    const JointLimits & limits,
    intrinsic_fbs::JointLimits & fb_limits)
  {
    if(const auto status = ToFlatbufferWithSizeCheck(
         std::span < const double > (limits.min_position.data(), limits.min_position.size()),
         *fb_limits.mutable_min_position(), "min_position");
      !status.ok())
    {
      return status;
    }
    if(const auto status = ToFlatbufferWithSizeCheck(
         std::span < const double > (limits.max_position.data(), limits.max_position.size()),
         *fb_limits.mutable_max_position(), "max_position"); !status.ok())
    {
      return status;
    }
    if (const auto status = ToFlatbufferWithSizeCheck(
          std::span < const double > (limits.max_velocity.data(), limits.max_velocity.size()),
          *fb_limits.mutable_max_velocity(), "max_velocity"); !status.ok())
    {
      return status;
    }
    if (const auto status = ToFlatbufferWithSizeCheck(
          std::span < const double > (limits.max_acceleration.data(),
        limits.max_acceleration.size()),
          *fb_limits.mutable_max_acceleration(),
      "max_acceleration"); !status.ok())
    {
      return status;
    }
    if (const auto status = ToFlatbufferWithSizeCheck(
          std::span < const double > (limits.max_jerk.data(), limits.max_jerk.size()),
          *fb_limits.mutable_max_jerk(), "max_jerk"); !status.ok())
    {
      return status;
    }
    if (const auto status = ToFlatbufferWithSizeCheck(
          std::span < const double > (limits.max_torque.data(), limits.max_torque.size()),
          *fb_limits.mutable_max_effort(), "max_effort"); !status.ok())
    {
      return status;
    }

    fb_limits.mutate_has_velocity_limits(true);
    fb_limits.mutate_has_acceleration_limits(true);
    fb_limits.mutate_has_jerk_limits(true);
    fb_limits.mutate_has_effort_limits(true);

    return RtOkStatus();
  }

  RealtimeStatus CopyTo(
    const intrinsic_fbs::JointLimits & fb_limits,
    JointLimits & limits)
  {
    const size_t num_joints = fb_limits.min_position()->size();
    if (const auto status = limits.SetSize(num_joints); !status.ok()) {
      return status;
    }
    for (int i = 0; i < num_joints; ++i) {
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

#if 0
  absl::Status ParseProtoJointLimits(
    const intrinsic_proto::JointLimits & pb_limits,
    icon::MutableHardwareInterfaceHandle < intrinsic_fbs::JointLimits > &
    fb_limits)
  {
    fb_limits->mutate_has_velocity_limits(pb_limits.has_max_velocity());
    fb_limits->mutate_has_acceleration_limits(pb_limits.has_max_acceleration());
    fb_limits->mutate_has_jerk_limits(pb_limits.has_max_jerk());
    fb_limits->mutate_has_effort_limits(pb_limits.has_max_effort());

    if (pb_limits.min_position().values_size() !=
      pb_limits.max_position().values_size() ||
      (pb_limits.has_max_velocity() &&
      pb_limits.min_position().values_size() !=
      pb_limits.max_velocity().values_size()) ||
      (pb_limits.has_max_acceleration() &&
      pb_limits.min_position().values_size() !=
      pb_limits.max_acceleration().values_size()) ||
      (pb_limits.has_max_jerk() && pb_limits.min_position().values_size() !=
      pb_limits.max_jerk().values_size()) ||
      (pb_limits.has_max_effort() &&
      pb_limits.min_position().values_size() !=
      pb_limits.max_effort().values_size()))
    {
      return absl::InvalidArgumentError(
        absl::StrFormat("All non-empty fields in JointLimits proto must have "
                        "the same size. Sizes are: "
                        "min_position:%d, max_position:%d, max_velocity:%d, "
                        "max_acceleration:%d, max_jerk:%d, max_effort:%d. ",
                        pb_limits.min_position().values_size(),
                        pb_limits.max_position().values_size(),
                        pb_limits.max_velocity().values_size(),
                        pb_limits.max_acceleration().values_size(),
                        pb_limits.max_jerk().values_size(),
                        pb_limits.max_effort().values_size()));
    }

    INTR_RETURN_IF_ERROR(ToFlatbufferWithSizeCheck(
      pb_limits.min_position().values(), *fb_limits->mutable_min_position(),
      "min_position"));
    INTR_RETURN_IF_ERROR(ToFlatbufferWithSizeCheck(
      pb_limits.max_position().values(), *fb_limits->mutable_max_position(),
      "max_position"));
    if (pb_limits.has_max_velocity()) {
      INTR_RETURN_IF_ERROR(ToFlatbufferWithSizeCheck(
        pb_limits.max_velocity().values(), *fb_limits->mutable_max_velocity(),
        "max_velocity"));
    }
    if (pb_limits.has_max_acceleration()) {
      INTR_RETURN_IF_ERROR(ToFlatbufferWithSizeCheck(
        pb_limits.max_acceleration().values(),
        *fb_limits->mutable_max_acceleration(), "max_acceleration"));
    }
    if (pb_limits.has_max_jerk()) {
      INTR_RETURN_IF_ERROR(
        ToFlatbufferWithSizeCheck(pb_limits.max_jerk().values(),
                                  *fb_limits->mutable_max_jerk(), "max_jerk"));
    }
    if (pb_limits.has_max_effort()) {
      INTR_RETURN_IF_ERROR(ToFlatbufferWithSizeCheck(
        pb_limits.max_effort().values(), *fb_limits->mutable_max_effort(),
        "max_effort"));
    }

    return absl::OkStatus();
  }
#endif

}  // namespace intrinsic::hal
