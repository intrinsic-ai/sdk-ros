#include "intrinsic/kinematics/types/joint_limits.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>

#include "intrinsic/utils/status.hpp"
#include "intrinsic/eigenmath/types.hpp"

namespace intrinsic {
namespace {

using eigenmath::VectorNd;
// using eigenmath::testing::IsApprox;
// using icon::RepeatedDoubleToVectorXd;
using ::testing::ElementsAre;

TEST(JointLimits, DefaultIsValid) { EXPECT_TRUE(JointLimits().IsValid()); }

TEST(JointLimits, UnlimitedIsValid) {
  auto limits = JointLimits::Unlimited(3);
  ASSERT_TRUE(limits.has_value()) << limits.error();
  EXPECT_TRUE(limits->IsValid());
}

TEST(JointLimits, IsValidWrongSize) {
  auto limits = JointLimits::Unlimited(3);
  ASSERT_TRUE(limits.has_value()) << limits.error();
  limits->min_position = VectorNd{{-1, -3}};
  limits->max_position = VectorNd{{1, 3}};
  EXPECT_FALSE(limits->IsValid());
}

TEST(JointLimits, IsValidBadLimits) {
  auto limits = JointLimits::Unlimited(2);
  ASSERT_TRUE(limits.has_value()) << limits.error();
  limits->min_position = VectorNd{{-1, 3}};
  limits->max_position = VectorNd{{1, 2}};
  EXPECT_FALSE(limits->IsValid());
}

TEST(JointLimits, IsValidEquals) {
  auto limits = JointLimits::Unlimited(2);
  ASSERT_TRUE(limits.has_value()) << limits.error();
  limits->min_position = VectorNd{{1, 2}};
  limits->max_position = VectorNd{{1, 3}};
  EXPECT_TRUE(limits->IsValid());
}

#if 0
TEST(JointLimits, Malloc) {
  auto limits = JointLimits::Unlimited(2);
  ASSERT_TRUE(limits.has_value()) << limits.error();
  limits->min_position = VectorNd{{1, 2}};
  limits->max_position = VectorNd{{1, 3}};
  IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
  bool is_valid = limits->IsValid();
  IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
  EXPECT_TRUE(is_valid);
}
#endif

TEST(JointLimits, SetSizeOnly) {
  JointLimits limits;
  {auto s = limits.SetSize(2); ASSERT_TRUE(s.ok()) << s;}
  limits.min_position = VectorNd{{-1, 2}};
  limits.max_position = VectorNd{{1, 3}};
  EXPECT_TRUE(limits.IsValid());
}

TEST(JointLimits, FromNegativeOfMaxDouble) {
  JointLimits limits;
  {auto s = limits.SetSize(2); ASSERT_TRUE(s.ok()) << s;}
  double double_max = std::numeric_limits<double>::max();
  limits.min_position = VectorNd{{-double_max, -double_max}};
  limits.max_position = VectorNd{{double_max, double_max}};
  EXPECT_TRUE(limits.IsValid());
}

TEST(JointLimits, CreateSimpleJointLimits) {
  {
    auto limits = CreateSimpleJointLimits(
        /*ndof=*/2,
        /*max_position=*/3,
        /*max_velocity=*/4,
        /*max_acceleration=*/5,
        /*max_jerk=*/6);
    EXPECT_THAT(limits.min_position, ElementsAre(-3, -3));
    EXPECT_THAT(limits.max_position, ElementsAre(3, 3));
    EXPECT_THAT(limits.max_velocity, ElementsAre(4, 4));
    EXPECT_THAT(limits.max_acceleration, ElementsAre(5, 5));
    EXPECT_THAT(limits.max_jerk, ElementsAre(6, 6));
  }

  {
    auto inf = std::numeric_limits<double>::infinity();
    auto limits = CreateSimpleJointLimits(
        /*ndof=*/2,
        /*max_position=*/1,
        /*max_velocity=*/inf,
        /*max_acceleration=*/inf,
        /*max_jerk=*/inf);
    EXPECT_THAT(limits.min_position, ElementsAre(-1, -1));
    EXPECT_THAT(limits.max_position, ElementsAre(1, 1));
    EXPECT_THAT(limits.max_velocity, ElementsAre(inf, inf));
    EXPECT_THAT(limits.max_acceleration, ElementsAre(inf, inf));
    EXPECT_THAT(limits.max_jerk, ElementsAre(inf, inf));
  }
}

TEST(JointLimits, EqualForSameLimits) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  EXPECT_EQ(base, other);
}

TEST(JointLimits, EqualForNanPositionLimits) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  base.min_position[0] = std::numeric_limits<double>::quiet_NaN();
  other.min_position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(base, other);
}

TEST(JointLimits, EqualForInfPositionLimits) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  base.min_position[0] = std::numeric_limits<double>::infinity();
  other.min_position[0] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(base, other);
}

TEST(JointLimits, NotEqualForDifferentMinPosition) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  other.min_position[0] = 2;
  EXPECT_NE(base, other);
}

TEST(JointLimits, NotEqualForDifferentMaxPosition) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  other.max_position[0] = 2;
  EXPECT_NE(base, other);
}

TEST(JointLimits, NotEqualForDifferentMaxVelocity) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  other.max_velocity[0] = 2;
  EXPECT_NE(base, other);
}

TEST(JointLimits, NotEqualForDifferentMaxAcceleration) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  other.max_acceleration[0] = 2;
  EXPECT_NE(base, other);
}

TEST(JointLimits, NotEqualForDifferentMaxJerk) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  other.max_jerk[0] = 2;
  EXPECT_NE(base, other);
}

TEST(JointLimits, NotEqualForDifferentMaxTorque) {
  JointLimits base = CreateSimpleJointLimits(/*ndof=*/2, 1, 1, 1, 1, 1);
  JointLimits other = base;
  other.max_torque[0] = 2;
  EXPECT_NE(base, other);
}

}  // namespace
}  // namespace intrinsic
