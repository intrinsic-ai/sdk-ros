#include "intrinsic/hal/hardware_interface_registry.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "intrinsic/utils/status.hpp"
#include "flatbuffers/detached_buffer.h"
#include "intrinsic/hal/get_hardware_interface.hpp"
#include "intrinsic/hal/hardware_interface_handle.hpp"
#include "intrinsic/hal/hardware_interface_traits.hpp"
#include "intrinsic/hal/icon_state_register.hpp"  // IWYU pragma: keep
#include "hwm_fbs/icon_state.fbs.h"
#include "hwm_fbs/joint_limits.fbs.h"
#include "intrinsic/hal/joint_limits_utils.hpp"
#include "intrinsic/shared_memory_manager/segment_header.hpp"
#include "intrinsic/shared_memory_manager/shared_memory_manager.hpp"
#include "intrinsic/shared_memory_manager/testing/unique_segment_name.hpp"
#include "intrinsic/utils/time.hpp"
#include "intrinsic/utils/current_cycle.hpp"

namespace intrinsic::hal
{

namespace hardware_interface_traits
{

// Registers the JointLimits interface for the registry.
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointLimits,
                                 intrinsic_fbs::BuildJointLimits,
                                 "intrinsic_fbs.JointLimits")

}  // namespace hardware_interface_traits

namespace
{

using ::testing::FieldsAre;
using ::testing::HasSubstr;

constexpr size_t kNumDof = 2;
constexpr char kHwmName[] = "my_test_hardware_module";

class RegistryTestFixture : public ::testing::Test {
public:
  void SetUp() override
  {
    memory_namespace_ = UniqueMemoryNamespace();
    {
      auto shm_manager = SharedMemoryManager::Create(memory_namespace_, kHwmName);
      ASSERT_TRUE(shm_manager.has_value()) << shm_manager.error();
      shm_manager_ = std::move(*shm_manager);
    }

    registry_ = intrinsic::hal::HardwareInterfaceRegistry(*shm_manager_);

    auto expected_interface =
      registry_->AdvertiseMutableInterface<intrinsic_fbs::IconState>(
            kIconStateInterfaceName);
    ASSERT_TRUE(expected_interface.has_value()) << expected_interface.error();
    icon_state_interface_ = std::move(*expected_interface);
  }

  void IncrementIconCycle()
  {
    Cycle::IncrementCurrentCycle();
    icon_state_interface_.UpdatedAt(intrinsic::Now());
    icon_state_interface_->mutate_current_cycle(Cycle::GetCurrentCycle());
  }

protected:
  std::unique_ptr<SharedMemoryManager> shm_manager_;
  std::optional<intrinsic::hal::HardwareInterfaceRegistry> registry_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::IconState>
  icon_state_interface_;
  std::string memory_namespace_;
};

TEST_F(RegistryTestFixture, StrictInterfaceVerifiesCycle) {
  auto interface =
    registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", kNumDof);
  ASSERT_TRUE(interface.has_value()) << interface.error();
  {      // Access is not OK because ICON was never ticked.
    auto value = interface->Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(),
                HasSubstr("inconsistent"));
  }
  IncrementIconCycle();
  {
  // Access is not OK because the command was not updated in this ICON cycle
    auto value = interface->Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(),
                HasSubstr("command_cycle"));
  }
  auto mutable_interface =
    GetMutableStrictInterfaceHandle<intrinsic_fbs::JointLimits>(
          *shm_manager_, "joint_limits");
  ASSERT_TRUE(mutable_interface.has_value()) << mutable_interface.error();

  // Updates the interface so that access is valid.
  mutable_interface->UpdatedAt(intrinsic::Now());
  {
    auto value = interface->Value();
    EXPECT_TRUE(value.has_value()) << value.error();
  }
}

TEST_F(RegistryTestFixture, MutableStrictInterfaceVerifiesCycle) {
  auto interface =
    registry_->AdvertiseMutableStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", kNumDof);
  ASSERT_TRUE(interface.has_value()) << interface.error();
  {
  // Access is not OK because ICON was never ticked.
    auto value = interface->Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(),
                HasSubstr("inconsistent"));
  }
  IncrementIconCycle();
  {
  // Access is not OK because the command was not updated in this ICON cycle
    auto value = interface->Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(),
                HasSubstr("command_cycle"));
  }
  // Updates the interface so that access is valid.
  interface->UpdatedAt(intrinsic::Now());
  {
    auto value = interface->Value();
    EXPECT_TRUE(value.has_value()) << value.error();
  }
}

TEST_F(RegistryTestFixture, MutableStrictInterfaceMutatesValues) {
  auto interface =
    registry_->AdvertiseMutableStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", kNumDof);
  ASSERT_TRUE(interface.has_value()) << interface.error();
  {
    // Read access is not OK because ICON was never ticked.
    auto value = interface->Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(),
                HasSubstr("inconsistent"));
  }
  // Mutable access is OK.
  interface->MutableValue()->mutable_max_position()->Mutate(0, 5);

  // Updates the interface so that access is valid.
  IncrementIconCycle();
  interface->UpdatedAt(intrinsic::Now());

  EXPECT_EQ(interface->MutableValue()->max_position()->Get(0), 5);
}

TEST_F(RegistryTestFixture, AdvertiseInterfaceNames) {
  auto expected_interface = registry_->AdvertiseInterface<intrinsic_fbs::JointLimits>(
      "joint_limits", kNumDof);
  EXPECT_TRUE(expected_interface.has_value()) << expected_interface.error();

  // Checks interface names need to be unique.
  {
    auto interface = registry_->AdvertiseInterface<intrinsic_fbs::JointLimits>("joint_limits",
                                                                               kNumDof);
    ASSERT_FALSE(interface.has_value());
    EXPECT_THAT(
        interface.error(),
        FieldsAre(StatusCode::kAlreadyExists, HasSubstr("joint_limits")));
  }
  {
    auto interface = registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>("joint_limits",
                                                                                     kNumDof);
    ASSERT_FALSE(interface.has_value());
    EXPECT_THAT(
        interface.error(),
        FieldsAre(StatusCode::kAlreadyExists, HasSubstr("joint_limits")));
  }
  {
    auto interface =
      registry_->AdvertiseMutableInterface<intrinsic_fbs::JointLimits>("joint_limits",
                                                                                      kNumDof);
    ASSERT_FALSE(interface.has_value());
    EXPECT_THAT(
        interface.error(),
        FieldsAre(StatusCode::kAlreadyExists, HasSubstr("joint_limits")));
  }
  {
    auto interface =
      registry_->AdvertiseMutableStrictInterface<intrinsic_fbs::JointLimits>("joint_limits",
                                                                                            kNumDof);
    ASSERT_FALSE(interface.has_value());
    EXPECT_THAT(
        interface.error(),
        FieldsAre(StatusCode::kAlreadyExists, HasSubstr("joint_limits")));
  }
}

TEST_F(RegistryTestFixture, AdvertiseInterface) {
  auto expected_interface = registry_->AdvertiseInterface<intrinsic_fbs::JointLimits>(
      "joint_limits", kNumDof);
  EXPECT_TRUE(expected_interface.has_value()) << expected_interface.error();
}

TEST_F(RegistryTestFixture, RegistersCorrectSizeForNonTrivialType) {
  auto expected_interface = registry_->AdvertiseInterface<intrinsic_fbs::JointLimits>(
      "joint_limits", kNumDof);
  EXPECT_TRUE(expected_interface.has_value()) << expected_interface.error();
  flatbuffers::DetachedBuffer buffer = intrinsic_fbs::BuildJointLimits(kNumDof);
  const size_t expected_size = sizeof(SegmentHeader) + buffer.size();

  auto fd = shm_manager_->SegmentNameToFileDescriptorMap().at("joint_limits");
  struct stat shared_memory_stats;
  ASSERT_EQ(fstat(fd, &shared_memory_stats), 0);
  EXPECT_EQ(shared_memory_stats.st_size, expected_size);
}

TEST_F(RegistryTestFixture, AdvertiseMutableInterface) {
  auto interface =
    registry_->AdvertiseMutableInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", kNumDof);
  ASSERT_TRUE(interface.has_value()) << interface.error();
  EXPECT_NE(interface.value()->max_position()->Get(0), 0.5);
  interface.value()->mutable_max_position()->Mutate(0, 5);
  EXPECT_EQ(interface.value()->max_position()->Get(0), 5);
}

TEST_F(RegistryTestFixture, GetSharedMemoryNamespace) {
  EXPECT_EQ(registry_->SharedMemoryNamespace(), memory_namespace_);
}

TEST_F(RegistryTestFixture, GetModuleName) {
  EXPECT_EQ(registry_->ModuleName(), "my_test_hardware_module");
}

TEST_F(RegistryTestFixture, GetInterfaceHandleWorks) {
  auto interface =
    registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", kNumDof);
  ASSERT_TRUE(interface.has_value()) << interface.error();

  auto handle = registry_->GetInterfaceHandle<intrinsic_fbs::JointLimits>(
      "joint_limits");
  EXPECT_TRUE(handle.has_value()) << handle.error();

  auto invalid_handle = registry_->GetInterfaceHandle<intrinsic_fbs::JointLimits>(
      "doesn't_exist");
  ASSERT_FALSE(invalid_handle.has_value());
  EXPECT_EQ(invalid_handle.error().code,
            StatusCode::kNotFound);
}

TEST_F(RegistryTestFixture, GetMutableInterfaceHandleWorks) {
      auto interface =
    registry_->AdvertiseMutableStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", kNumDof);
      ASSERT_TRUE(interface.has_value()) << interface.error();

      auto limits =
    registry_->GetMutableInterfaceHandle<intrinsic_fbs::JointLimits>(
          "joint_limits");
      ASSERT_TRUE(limits.has_value()) << limits.error();

  // Mutates.
      EXPECT_NE(limits.value()->max_position()->Get(0), 42);
      limits.value()->mutable_max_position()->Mutate(0, 42);
      EXPECT_EQ(limits.value()->max_position()->Get(0), 42);

  auto wrong_name_handle = registry_->GetMutableInterfaceHandle<intrinsic_fbs::JointLimits>(
      "doesn't_exist");
  ASSERT_FALSE(wrong_name_handle.has_value());
  EXPECT_EQ(wrong_name_handle.error().code, StatusCode::kNotFound) << wrong_name_handle.error();
}

TEST_F(RegistryTestFixture,
       GetInterfaceHandleValidatesDynamicSizedFlatbuffers) {
  auto interface =
    registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", kNumDof);
  ASSERT_TRUE(interface.has_value()) << interface.error();

  // Shrinks the segment to an invalid size, that passes the basic size checks
  // for trivially_copyable types.
  ASSERT_EQ(
      ftruncate(
          shm_manager_->SegmentNameToFileDescriptorMap().at("joint_limits"),
          sizeof(SegmentHeader) + sizeof(intrinsic_fbs::JointLimits)),
      0);

  auto invalid_handle = registry_->GetInterfaceHandle<intrinsic_fbs::JointLimits>("joint_limits");
  ASSERT_FALSE(invalid_handle.has_value());
  EXPECT_THAT(
      invalid_handle.error(),
      FieldsAre(StatusCode::kInvalidArgument,
               AllOf(HasSubstr("verification"), HasSubstr("joint_limits"))));
}

TEST_F(RegistryTestFixture,
       GetMutableInterfaceHandleValidatesDynamicSizedFlatbuffers) {
      auto interface =
    registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", kNumDof);
      ASSERT_TRUE(interface.has_value()) << interface.error();

  // Shrinks the segment to an invalid size, that passes the basic size checks
  // for trivially_copyable types.
  ASSERT_EQ(
      ftruncate(
          shm_manager_->SegmentNameToFileDescriptorMap().at("joint_limits"),
          sizeof(SegmentHeader) + sizeof(intrinsic_fbs::JointLimits)),
      0);

  auto invalid_handle = registry_->GetMutableInterfaceHandle<intrinsic_fbs::JointLimits>(
      "joint_limits");
  ASSERT_FALSE(invalid_handle.has_value());
  EXPECT_THAT(
      invalid_handle.error(),
      FieldsAre(StatusCode::kInvalidArgument,
               AllOf(HasSubstr("verification"), HasSubstr("joint_limits"))));
}

}  // namespace
}  // namespace intrinsic::hal


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
