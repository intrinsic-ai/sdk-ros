#include "intrinsic/hal/joint_limits_utils.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

#include "intrinsic/utils/status.hpp"
#include "flatbuffers/buffer.h"
#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/verifier.h"
#include "intrinsic/hal/hardware_interface_handle.hpp"
#include "intrinsic/hal/hardware_interface_registry.hpp"
#include "intrinsic/hal/hardware_interface_traits.hpp"
#include "hwm_fbs/joint_limits.fbs.h"
#include "intrinsic/shared_memory_manager/shared_memory_manager.hpp"
#include "intrinsic/shared_memory_manager/testing/unique_segment_name.hpp"
#include "intrinsic/kinematics/types/joint_limits.hpp"

using ::intrinsic_fbs::BuildJointLimits;
using ::intrinsic_fbs::JointLimits;
using ::testing::FieldsAre;
using ::testing::ElementsAre;
using ::testing::HasSubstr;

namespace intrinsic::hal::hardware_interface_traits
{

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointLimits,
                                 intrinsic_fbs::BuildJointLimits,
                                 "intrinsic_fbs.JointLimits");

}  // namespace intrinsic::hal::hardware_interface_traits

namespace intrinsic::hardware
{
namespace
{

TEST(JointLimitsTest, CreateJointLimitsWithSize) {
  uint32_t num_dof = 6;
  flatbuffers::DetachedBuffer buffer = BuildJointLimits(num_dof);
  flatbuffers::Verifier verifier(buffer.data(), buffer.size());
  ASSERT_EQ(verifier.VerifyBuffer<intrinsic_fbs::JointLimits>(), true);

  const auto state =
    flatbuffers::GetMutableRoot<intrinsic_fbs::JointLimits>(buffer.data());
  ASSERT_NE(state, nullptr);

  ASSERT_EQ(state->min_position()->size(), num_dof);
  ASSERT_EQ(state->max_position()->size(), num_dof);
  ASSERT_EQ(state->has_velocity_limits(), false);
  ASSERT_EQ(state->max_velocity()->size(), num_dof);
  ASSERT_EQ(state->has_acceleration_limits(), false);
  ASSERT_EQ(state->max_acceleration()->size(), num_dof);
  ASSERT_EQ(state->has_jerk_limits(), false);
  ASSERT_EQ(state->max_jerk()->size(), num_dof);
  ASSERT_EQ(state->has_effort_limits(), false);
  ASSERT_EQ(state->max_effort()->size(), num_dof);
  for (size_t i = 0; i < num_dof; ++i) {
    EXPECT_EQ(state->min_position()->Get(i), 0.0);
    EXPECT_EQ(state->max_position()->Get(i), 0.0);
    EXPECT_EQ(state->max_velocity()->Get(i), 0.0);
    EXPECT_EQ(state->max_acceleration()->Get(i), 0.0);
    EXPECT_EQ(state->max_jerk()->Get(i), 0.0);
    EXPECT_EQ(state->max_effort()->Get(i), 0.0);
  }
}

}  // namespace
}  // namespace intrinsic::hardware

namespace intrinsic::hal
{
namespace
{

class LimitsFixture : public ::testing::Test {
public:
  void SetUp() override
  {
    auto shm_manager =
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "module_name", nullptr);
    ASSERT_TRUE(shm_manager.has_value()) << shm_manager.error();
    shm_manager_ = std::move(*shm_manager);
    registry_ = HardwareInterfaceRegistry(*shm_manager_);
  }

protected:
  std::unique_ptr<SharedMemoryManager> shm_manager_;
  // Holds a reference to shm_manager_, do not move above shm_manager_.
  std::optional<intrinsic::hal::HardwareInterfaceRegistry> registry_;

  // Returns a mutable hardware interface handle for joint limits with
  // 'num_dofs'
  // joints.
  MutableHardwareInterfaceHandle<intrinsic_fbs::JointLimits>
  CreateJointLimitsFbHandle(int num_dofs)
  {
    if(num_dofs <= 0) {
      std::exit(1);
    }
    return registry_
           ->AdvertiseMutableInterface<intrinsic_fbs::JointLimits>("joint_limits",
                                                                   /*logger=*/nullptr,
                                                                   num_dofs)
           .value();
  }
};

TEST_F(LimitsFixture, CopyToFlatbufferCopiesCorrectly) {
  auto fb_limits = CreateJointLimitsFbHandle(3);

  JointLimits limits;
  {
    auto s = limits.SetSize(3);
    ASSERT_EQ(s.code, StatusCode::kOk) << s;
  }
  limits.min_position << 1.1, 1.2, 1.3;
  limits.max_position << 2.1, 2.2, 2.3;
  limits.max_velocity << 3.1, 3.2, 3.3;
  limits.max_acceleration << 4.1, 4.2, 4.3;
  limits.max_jerk << 5.1, 5.2, 5.3;
  limits.max_torque << 6.1, 6.2, 6.3;

  {
    auto s = CopyTo(limits, **fb_limits);
    ASSERT_EQ(s.code, StatusCode::kOk) << s;
  }
  EXPECT_THAT(*fb_limits->min_position(), ElementsAre(1.1, 1.2, 1.3));
  EXPECT_THAT(*fb_limits->max_position(), ElementsAre(2.1, 2.2, 2.3));
  EXPECT_THAT(*fb_limits->max_velocity(), ElementsAre(3.1, 3.2, 3.3));
  EXPECT_THAT(*fb_limits->max_acceleration(), ElementsAre(4.1, 4.2, 4.3));
  EXPECT_THAT(*fb_limits->max_jerk(), ElementsAre(5.1, 5.2, 5.3));
  EXPECT_THAT(*fb_limits->max_effort(), ElementsAre(6.1, 6.2, 6.3));
  EXPECT_TRUE(fb_limits->has_velocity_limits());
  EXPECT_TRUE(fb_limits->has_acceleration_limits());
  EXPECT_TRUE(fb_limits->has_jerk_limits());
  EXPECT_TRUE(fb_limits->has_effort_limits());
}

TEST_F(LimitsFixture, CopyInfinityToFlatbufferCopiesCorrectly) {
  auto fb_limits = CreateJointLimitsFbHandle(1);

  JointLimits limits;
  {
    auto s = limits.SetSize(1);
    ASSERT_EQ(s.code, StatusCode::kOk) << s;
  }
  limits.max_jerk << std::numeric_limits<double>::infinity();

  {
    auto s = CopyTo(limits, **fb_limits);
    ASSERT_EQ(s.code, StatusCode::kOk) << s;
  }
  EXPECT_THAT(*fb_limits->max_jerk(),
              ElementsAre(std::numeric_limits<double>::infinity()));
}

TEST_F(LimitsFixture, CopyToStructCopiesCorrectly) {
  auto fb_limits = CreateJointLimitsFbHandle(3);

  fb_limits->mutable_min_position()->Mutate(0, 1.1);
  fb_limits->mutable_min_position()->Mutate(1, 1.2);
  fb_limits->mutable_min_position()->Mutate(2, 1.3);
  fb_limits->mutable_max_position()->Mutate(0, 2.1);
  fb_limits->mutable_max_position()->Mutate(1, 2.2);
  fb_limits->mutable_max_position()->Mutate(2, 2.3);
  fb_limits->mutable_max_velocity()->Mutate(0, 3.1);
  fb_limits->mutable_max_velocity()->Mutate(1, 3.2);
  fb_limits->mutable_max_velocity()->Mutate(2, 3.3);
  fb_limits->mutable_max_acceleration()->Mutate(0, 4.1);
  fb_limits->mutable_max_acceleration()->Mutate(1, 4.2);
  fb_limits->mutable_max_acceleration()->Mutate(2, 4.3);
  fb_limits->mutable_max_jerk()->Mutate(0, 5.1);
  fb_limits->mutable_max_jerk()->Mutate(1, 5.2);
  fb_limits->mutable_max_jerk()->Mutate(2, 5.3);
  fb_limits->mutable_max_effort()->Mutate(0, 6.1);
  fb_limits->mutable_max_effort()->Mutate(1, 6.2);
  fb_limits->mutable_max_effort()->Mutate(2, 6.3);
  fb_limits->mutate_has_velocity_limits(true);
  fb_limits->mutate_has_acceleration_limits(true);
  fb_limits->mutate_has_jerk_limits(true);
  fb_limits->mutate_has_effort_limits(true);

  JointLimits got_limits;
  {
    auto s = CopyTo(**fb_limits, got_limits);
    ASSERT_EQ(s.code, StatusCode::kOk) << s;
  }
  EXPECT_EQ(got_limits.size(), 3);

  EXPECT_THAT(got_limits.min_position, ElementsAre(1.1, 1.2, 1.3));
  EXPECT_THAT(got_limits.max_position, ElementsAre(2.1, 2.2, 2.3));
  EXPECT_THAT(got_limits.max_velocity, ElementsAre(3.1, 3.2, 3.3));
  EXPECT_THAT(got_limits.max_acceleration, ElementsAre(4.1, 4.2, 4.3));
  EXPECT_THAT(got_limits.max_jerk, ElementsAre(5.1, 5.2, 5.3));
  EXPECT_THAT(got_limits.max_torque, ElementsAre(6.1, 6.2, 6.3));
}

TEST_F(LimitsFixture, CopyInfinityToStructCopiesCorrectly) {
  auto fb_limits = CreateJointLimitsFbHandle(1);

  fb_limits->mutable_max_jerk()->Mutate(
      0, std::numeric_limits<double>::infinity());
  fb_limits->mutate_has_jerk_limits(true);

  JointLimits got_limits;
  {
    auto s = CopyTo(**fb_limits, got_limits);
    ASSERT_EQ(s.code, StatusCode::kOk) << s;
  }
  EXPECT_EQ(got_limits.size(), 1);

  EXPECT_THAT(got_limits.max_jerk,
              ElementsAre(std::numeric_limits<double>::infinity()));
}

}  // namespace
}  // namespace intrinsic::hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
