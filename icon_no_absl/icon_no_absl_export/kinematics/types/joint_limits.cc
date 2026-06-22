#include "kinematics/types/joint_limits.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <tl/expected.hpp>

#include "eigenmath/types.h"
#include "icon/utils/status.h"

namespace intrinsic {

tl::expected<JointLimits, RealtimeStatus> JointLimits::Unlimited(size_t size) {
  JointLimits limits;
  if (auto status = limits.SetSize(size); !status.ok()) {
    return tl::unexpected(status);
  }
  limits.SetUnlimited();
  return limits;
}

eigenmath::VectorXd::Index JointLimits::size() const {
  return min_position.size();
}

bool JointLimits::IsSizeConsistent() const {
  const eigenmath::VectorXd::Index size = this->size();
  return (min_position.rows() == size) && (max_position.rows() == size) &&
         (max_velocity.rows() == size) && (max_acceleration.rows() == size) &&
         (max_jerk.rows() == size) && (max_torque.rows() == size);
}

RealtimeStatus JointLimits::SetSize(eigenmath::VectorXd::Index size) {
  if (size > kMaxSize) {
    auto status = RealtimeStatus{
        .code = StatusCode::kInvalidArgument,
    };
    (void)std::snprintf(status.message.data(), status.message.size(),
                        "size=%ld exceeds max size=%ld", size, kMaxSize);
    return status;
  }
  min_position = eigenmath::VectorNd::Constant(size, 0.);
  max_position = eigenmath::VectorNd::Constant(size, 0.);
  max_velocity = eigenmath::VectorNd::Constant(size, 0.);
  max_acceleration = eigenmath::VectorNd::Constant(size, 0.);
  max_jerk = eigenmath::VectorNd::Constant(size, 0.);
  max_torque = eigenmath::VectorNd::Constant(size, 0.);
  return RtOkStatus();
}

void JointLimits::SetUnlimited() {
  min_position.setConstant(-std::numeric_limits<double>::infinity());
  max_position.setConstant(std::numeric_limits<double>::infinity());
  max_velocity.setConstant(std::numeric_limits<double>::infinity());
  max_acceleration.setConstant(std::numeric_limits<double>::infinity());
  max_jerk.setConstant(std::numeric_limits<double>::infinity());
  max_torque.setConstant(std::numeric_limits<double>::infinity());
}

bool JointLimits::IsValid() const {
  if (!IsSizeConsistent()) {
    return false;
  }

  if (size() == 0) {
    return true;
  }

  return (max_position - min_position).minCoeff() >= 0 &&
         (max_velocity.array() >= 0).all() &&
         (max_acceleration.array() >= 0).all() &&
         (max_jerk.array() >= 0).all() && (max_torque.array() >= 0).all();
}

JointLimits CreateSimpleJointLimits(int ndof, double max_position,
                                    double max_velocity,
                                    double max_acceleration, double max_jerk) {
  JointLimits limits;
  if (auto status = limits.SetSize(ndof); !status.ok()) {
    std::exit(1);
  }
  limits.SetUnlimited();
  for (int i = 0; i < ndof; ++i) {
    limits.min_position[i] = -max_position;
    limits.max_position[i] = max_position;
    limits.max_velocity[i] = max_velocity;
    limits.max_acceleration[i] = max_acceleration;
    limits.max_jerk[i] = max_jerk;
  }
  return limits;
}

JointLimits CreateSimpleJointLimits(int ndof, double max_position,
                                    double max_velocity,
                                    double max_acceleration, double max_jerk,
                                    double max_effort) {
  JointLimits limits;
  if (auto status = limits.SetSize(ndof); !status.ok()) {
    std::exit(1);
  }
  limits.SetUnlimited();
  for (int i = 0; i < ndof; ++i) {
    limits.min_position[i] = -max_position;
    limits.max_position[i] = max_position;
    limits.max_velocity[i] = max_velocity;
    limits.max_acceleration[i] = max_acceleration;
    limits.max_jerk[i] = max_jerk;
    limits.max_torque[i] = max_effort;
  }
  return limits;
}

bool CompareDoubles(const double lhs, const double rhs) {
  if (std::isinf(lhs) && std::isinf(rhs)) {
    return true;
  }
  if (std::isnan(lhs) && std::isnan(rhs)) {
    return true;
  }
  return lhs == rhs;
}

// Compares two vectors. Inf and NaN are considered equal.
bool CompareVector(const eigenmath::VectorNd& vec1,
                   const eigenmath::VectorNd& vec2) {
  if (vec1.size() != vec2.size()) {
    return false;
  }
  for (int i = 0; i < vec1.size(); ++i) {
    if (!CompareDoubles(vec1[i], vec2[i])) {
      return false;
    }
  }
  return true;
}

bool JointLimits::operator==(const JointLimits& other) const {
  return CompareVector(min_position, other.min_position) &&
         CompareVector(max_position, other.max_position) &&
         CompareVector(max_velocity, other.max_velocity) &&
         CompareVector(max_acceleration, other.max_acceleration) &&
         CompareVector(max_jerk, other.max_jerk) &&
         CompareVector(max_torque, other.max_torque);
}

}  // namespace intrinsic
