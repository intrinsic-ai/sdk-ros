#include "icon/hal/hardware_interface_registry.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
#include "icon/hal/get_hardware_interface.h"
#include "icon/hal/hardware_interface_handle.h"
#include "icon/hal/hardware_interface_traits.h"
#include "icon/hal/icon_state_register.h"  // IWYU pragma: keep
#include "icon/hal/interfaces/joint_limits_utils.h"
#include "icon/interprocess/shared_memory_manager/segment_header.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"
#include "icon/utils/current_cycle.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_test_macros.h"
#include "icon/utils/time.h"

namespace intrinsic::icon {

namespace hardware_interface_traits {

// Registers the JointLimits interface for the registry.
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointLimits,
                                 intrinsic_fbs::BuildJointLimits,
                                 "intrinsic_fbs.JointLimits")

}  // namespace hardware_interface_traits

namespace {

using ::testing::FieldsAre;
using ::testing::HasSubstr;

constexpr size_t kNumDof = 2;
constexpr char kHwmName[] = "my_test_hardware_module";

class RegistryTestFixture : public ::testing::Test {
 public:
  void SetUp() override {
    memory_namespace_ = UniqueMemoryNamespace();
    INTR_ASSERT_OK_AND_ASSIGN(
        shm_manager_,
        SharedMemoryManager::Create(memory_namespace_, kHwmName, nullptr));

    registry_ = intrinsic::icon::HardwareInterfaceRegistry(*shm_manager_);

    INTR_ASSERT_OK_AND_ASSIGN(
        icon_state_interface_,
        registry_->AdvertiseMutableInterface<intrinsic_fbs::IconState>(
            kIconStateInterfaceName, /*logger=*/nullptr));
  }

  void IncrementIconCycle() {
    Cycle::IncrementCurrentCycle();
    icon_state_interface_.UpdatedAt(intrinsic::Now());
    icon_state_interface_->mutate_current_cycle(Cycle::GetCurrentCycle());
  }

 protected:
  std::unique_ptr<SharedMemoryManager> shm_manager_;
  std::optional<intrinsic::icon::HardwareInterfaceRegistry> registry_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::IconState>
      icon_state_interface_;
  std::string memory_namespace_;
};

TEST_F(RegistryTestFixture, StrictInterfaceVerifiesCycle) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto interface,
      registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr, kNumDof));
  {  // Access is not OK because ICON was never ticked.
    auto value = interface.Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(), HasSubstr("inconsistent"));
  }
  IncrementIconCycle();
  {
    // Access is not OK because the command was not updated in this ICON cycle
    auto value = interface.Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(), HasSubstr("command_cycle"));
  }
  INTR_ASSERT_OK_AND_ASSIGN(
      auto mutable_interface,
      GetMutableStrictInterfaceHandle<intrinsic_fbs::JointLimits>(
          *shm_manager_, "joint_limits", nullptr));

  // Updates the interface so that access is valid.
  mutable_interface.UpdatedAt(intrinsic::Now());
  INTR_EXPECT_OK(interface.Value());
}

TEST_F(RegistryTestFixture, MutableStrictInterfaceVerifiesCycle) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto interface,
      registry_->AdvertiseMutableStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr, kNumDof));
  {
    // Access is not OK because ICON was never ticked.
    auto value = interface.Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(), HasSubstr("inconsistent"));
  }
  IncrementIconCycle();
  {
    // Access is not OK because the command was not updated in this ICON cycle
    auto value = interface.Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(), HasSubstr("command_cycle"));
  }
  // Updates the interface so that access is valid.
  interface.UpdatedAt(intrinsic::Now());
  INTR_EXPECT_OK(interface.Value());
}

TEST_F(RegistryTestFixture, MutableStrictInterfaceMutatesValues) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto interface,
      registry_->AdvertiseMutableStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr, kNumDof));
  {
    // Read access is not OK because ICON was never ticked.
    auto value = interface.Value();
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, StatusCode::kFailedPrecondition);
    EXPECT_THAT(value.error().GetMessage(), HasSubstr("inconsistent"));
  }
  // Mutable access is OK.
  interface.MutableValue()->mutable_max_position()->Mutate(0, 5);

  // Updates the interface so that access is valid.
  IncrementIconCycle();
  interface.UpdatedAt(intrinsic::Now());

  EXPECT_EQ(interface.MutableValue()->max_position()->Get(0), 5);
}

TEST_F(RegistryTestFixture, AdvertiseInterfaceNames) {
  INTR_EXPECT_OK(registry_->AdvertiseInterface<intrinsic_fbs::JointLimits>(
      "joint_limits", /*logger=*/nullptr, kNumDof));

  // Checks interface names need to be unique.
  {
    auto interface = registry_->AdvertiseInterface<intrinsic_fbs::JointLimits>(
        "joint_limits",
        /*logger=*/
        nullptr, kNumDof);
    ASSERT_FALSE(interface.has_value());
    EXPECT_THAT(interface.error(), FieldsAre(StatusCode::kAlreadyExists,
                                             HasSubstr("joint_limits")));
  }
  {
    auto interface =
        registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
            "joint_limits",
            /*logger=*/
            nullptr, kNumDof);
    ASSERT_FALSE(interface.has_value());
    EXPECT_THAT(interface.error(), FieldsAre(StatusCode::kAlreadyExists,
                                             HasSubstr("joint_limits")));
  }
  {
    auto interface =
        registry_->AdvertiseMutableInterface<intrinsic_fbs::JointLimits>(
            "joint_limits",
            /*logger=*/
            nullptr, kNumDof);
    ASSERT_FALSE(interface.has_value());
    EXPECT_THAT(interface.error(), FieldsAre(StatusCode::kAlreadyExists,
                                             HasSubstr("joint_limits")));
  }
  {
    auto interface =
        registry_->AdvertiseMutableStrictInterface<intrinsic_fbs::JointLimits>(
            "joint_limits",
            /*logger=*/
            nullptr, kNumDof);
    ASSERT_FALSE(interface.has_value());
    EXPECT_THAT(interface.error(), FieldsAre(StatusCode::kAlreadyExists,
                                             HasSubstr("joint_limits")));
  }
}

TEST_F(RegistryTestFixture, AdvertiseInterface) {
  INTR_EXPECT_OK(registry_->AdvertiseInterface<intrinsic_fbs::JointLimits>(
      "joint_limits", /*logger=*/nullptr, kNumDof));
}

TEST_F(RegistryTestFixture, RegistersCorrectSizeForNonTrivialType) {
  INTR_EXPECT_OK(registry_->AdvertiseInterface<intrinsic_fbs::JointLimits>(
      "joint_limits", /*logger=*/nullptr, kNumDof));
  flatbuffers::DetachedBuffer buffer = intrinsic_fbs::BuildJointLimits(kNumDof);
  const size_t expected_size = sizeof(SegmentHeader) + buffer.size();

  auto fd = shm_manager_->SegmentNameToFileDescriptorMap().at("joint_limits");
  struct stat shared_memory_stats;
  ASSERT_EQ(fstat(fd, &shared_memory_stats), 0);
  EXPECT_EQ(shared_memory_stats.st_size, expected_size);
}

TEST_F(RegistryTestFixture, AdvertiseMutableInterface) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto interface,
      registry_->AdvertiseMutableInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr, kNumDof));
  EXPECT_NE(interface->max_position()->Get(0), 0.5);
  interface->mutable_max_position()->Mutate(0, 5);
  EXPECT_EQ(interface->max_position()->Get(0), 5);
}

TEST_F(RegistryTestFixture, GetSharedMemoryNamespace) {
  EXPECT_EQ(registry_->SharedMemoryNamespace(), memory_namespace_);
}

TEST_F(RegistryTestFixture, GetModuleName) {
  EXPECT_EQ(registry_->ModuleName(), "my_test_hardware_module");
}

TEST_F(RegistryTestFixture, GetInterfaceHandleWorks) {
  INTR_EXPECT_OK(
      registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr, kNumDof));

  INTR_EXPECT_OK(registry_->GetInterfaceHandle<intrinsic_fbs::JointLimits>(
      "joint_limits", /*logger=*/nullptr));

  auto invalid_handle =
      registry_->GetInterfaceHandle<intrinsic_fbs::JointLimits>(
          "doesn't_exist", /*logger=*/nullptr);
  ASSERT_FALSE(invalid_handle.has_value());
  EXPECT_EQ(invalid_handle.error().code, StatusCode::kNotFound);
}

TEST_F(RegistryTestFixture, GetMutableInterfaceHandleWorks) {
  INTR_EXPECT_OK(
      registry_->AdvertiseMutableStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr, kNumDof));

  INTR_ASSERT_OK_AND_ASSIGN(
      auto limits,
      registry_->GetMutableInterfaceHandle<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr));

  // Mutates.
  EXPECT_NE(limits->max_position()->Get(0), 42);
  limits->mutable_max_position()->Mutate(0, 42);
  EXPECT_EQ(limits->max_position()->Get(0), 42);

  auto wrong_name_handle =
      registry_->GetMutableInterfaceHandle<intrinsic_fbs::JointLimits>(
          "doesn't_exist", /*logger=*/nullptr);
  ASSERT_FALSE(wrong_name_handle.has_value());
  EXPECT_EQ(wrong_name_handle.error().code, StatusCode::kNotFound)
      << ToString(wrong_name_handle.error());
}

TEST_F(RegistryTestFixture,
       GetInterfaceHandleValidatesDynamicSizedFlatbuffers) {
  INTR_EXPECT_OK(
      registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr, kNumDof));

  // Shrinks the segment to an invalid size, that passes the basic size checks
  // for trivially_copyable types.
  ASSERT_EQ(
      ftruncate(
          shm_manager_->SegmentNameToFileDescriptorMap().at("joint_limits"),
          sizeof(SegmentHeader) + sizeof(intrinsic_fbs::JointLimits)),
      0);

  auto invalid_handle =
      registry_->GetInterfaceHandle<intrinsic_fbs::JointLimits>("joint_limits",
                                                                /*logger=*/
                                                                nullptr);
  ASSERT_FALSE(invalid_handle.has_value());
  EXPECT_THAT(
      invalid_handle.error(),
      FieldsAre(StatusCode::kInvalidArgument,
                AllOf(HasSubstr("verification"), HasSubstr("joint_limits"))));
}

TEST_F(RegistryTestFixture,
       GetMutableInterfaceHandleValidatesDynamicSizedFlatbuffers) {
  INTR_EXPECT_OK(
      registry_->AdvertiseStrictInterface<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr, kNumDof));

  // Shrinks the segment to an invalid size, that passes the basic size checks
  // for trivially_copyable types.
  ASSERT_EQ(
      ftruncate(
          shm_manager_->SegmentNameToFileDescriptorMap().at("joint_limits"),
          sizeof(SegmentHeader) + sizeof(intrinsic_fbs::JointLimits)),
      0);

  auto invalid_handle =
      registry_->GetMutableInterfaceHandle<intrinsic_fbs::JointLimits>(
          "joint_limits", /*logger=*/nullptr);
  ASSERT_FALSE(invalid_handle.has_value());
  EXPECT_THAT(
      invalid_handle.error(),
      FieldsAre(StatusCode::kInvalidArgument,
                AllOf(HasSubstr("verification"), HasSubstr("joint_limits"))));
}

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
