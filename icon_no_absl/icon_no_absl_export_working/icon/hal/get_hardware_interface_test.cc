#include "icon/hal/get_hardware_interface.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
#include "flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "icon/flatbuffers/flatbuffer_utils.h"
#include "icon/hal/icon_state_register.h"  // IWYU pragma: keep
#include "icon/hal/interfaces/icon_state_utils.h"
#include "icon/hal/interfaces/joint_limits_utils.h"
#include "icon/interprocess/shared_memory_manager/segment_header.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_test_macros.h"

namespace intrinsic::icon {

// Friend class to SegmentHeader, for testing purposes only. Refer to
// go/totw/135 for more info on the "test peer" design pattern.
class SegmentHeaderTestPeer {
 public:
  static void SetUpdateCounter(int64_t value, SegmentHeader& header) {
    header.update_counter_ = value;
  }
  static void SetVersion(size_t version, SegmentHeader& header) {
    size_t* version_ptr = const_cast<size_t*>(&header.kVersion);
    *version_ptr = version;
  }

  static void SetUpdatedAtCycle(uint64_t value, SegmentHeader& header) {
    header.updated_at_cycle_ = value;
  }
};

namespace {

using intrinsic::icon::SegmentHeader;
using ::intrinsic_fbs::FlatbufferArrayNumElements;
using ::testing::AllOf;
using ::testing::ElementsAreArray;
using ::testing::HasSubstr;

// Needs to match
// intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs
inline constexpr size_t kMaxSegmentNames =
    FlatbufferArrayNumElements(&intrinsic_fbs::SegmentInfo::names);

// Returns a list of realistic segment names.
std::vector<std::string> MemoryNames() {
  std::vector<std::string> expected_names;
  expected_names.reserve(kMaxSegmentNames);
  for (size_t i = 0; i < kMaxSegmentNames; ++i) {
    // A name longer than SegmentName::value will be truncated by
    // CreateSegmentInfo().
    expected_names.push_back("some_op_" + std::to_string(i));
  }
  return expected_names;
}

// Creates a SegmentInfo struct with the maximum number of segments.
// Calls MemoryNames() to get the list of segments.
intrinsic_fbs::SegmentInfo CreateSegmentInfo() {
  intrinsic_fbs::SegmentInfo segment_info;

  std::vector<std::string> full_segment_names = MemoryNames();

  for (size_t i = 0; i < kMaxSegmentNames; ++i) {
    intrinsic_fbs::SegmentName segment_name;

    // Copies the name including the null terminator of std::string.
    char* name_ptr =
        reinterpret_cast<char*>(segment_name.mutable_value()->Data());
    std::snprintf(name_ptr, segment_name.value()->size(), "%s",
                  full_segment_names[i].c_str());

    segment_info.mutable_names()->Mutate(i, segment_name);
  }
  segment_info.mutate_size(kMaxSegmentNames);
  return segment_info;
}

TEST(GetHardwareInterface, GetInterfacesFromModuleInfo) {
  intrinsic_fbs::SegmentInfo segment_info = CreateSegmentInfo();
  INTR_ASSERT_OK_AND_ASSIGN(auto interfaces,
                            GetInterfacesFromModuleInfo(segment_info));
  EXPECT_THAT(interfaces, ElementsAreArray(MemoryNames()));
}

TEST(GetHardwareInterface, GetRequiredInterfacesFromModuleInfo) {
  intrinsic_fbs::SegmentInfo segment_info = CreateSegmentInfo();

  for (size_t i = 0; i < kMaxSegmentNames / 2; ++i) {
    segment_info.mutable_names()->GetMutablePointer(i)->mutate_must_be_used(
        true);
  }
  std::vector<std::string> names = MemoryNames();
  names.resize(kMaxSegmentNames / 2);

  INTR_ASSERT_OK_AND_ASSIGN(auto interfaces,
                            GetRequiredInterfacesFromModuleInfo(segment_info));
  EXPECT_THAT(interfaces, ElementsAreArray(names));
}

TEST(SegmentHeaderIsValid, FailsOnWrongType) {
  SegmentHeader my_header("wrong_type", nullptr);
  const Status s = SegmentHeaderIsValid<intrinsic_fbs::IconState>(
      my_header, "interface_name");
  EXPECT_EQ(s.code, StatusCode::kInvalidArgument);
  EXPECT_THAT(s.message, HasSubstr("type"));
}

TEST(SegmentHeaderIsValid, FailsOnWrongVersion) {
  SegmentHeader my_header("intrinsic_fbs.IconState", nullptr);

  SegmentHeaderTestPeer::SetVersion(SegmentHeader::ExpectedVersion() + 1,
                                    my_header);
  const Status s = SegmentHeaderIsValid<intrinsic_fbs::IconState>(
      my_header, "interface_name");
  EXPECT_EQ(s.code, StatusCode::kInvalidArgument);
  EXPECT_THAT(s.message, HasSubstr("version"));
}

TEST(SegmentHeaderIsValid, Succeeds) {
  SegmentHeader my_header("intrinsic_fbs.IconState", nullptr);

  INTR_EXPECT_OK(SegmentHeaderIsValid<intrinsic_fbs::IconState>(
      my_header, "interface_name"));
}

TEST(FlatbufferIsValid, SucceedsForDetachedBufferOfStruct) {
  flatbuffers::DetachedBuffer buffer = intrinsic_fbs::BuildIconState();

  INTR_EXPECT_OK(FlatbufferIsValid<intrinsic_fbs::IconState>(
      buffer.data(), buffer.size(), "some_name"));
}

TEST(FlatbufferIsValid, SucceedsForStruct) {
  intrinsic_fbs::IconState icon_state;

  uint8_t* icon_state_ptr = reinterpret_cast<uint8_t*>(&icon_state);

  INTR_EXPECT_OK(FlatbufferIsValid<intrinsic_fbs::IconState>(
      icon_state_ptr, sizeof(icon_state), "some_name"));
}

TEST(FlatbufferIsValid, SucceedsForTable) {
  const size_t kNumDof = 6;
  flatbuffers::DetachedBuffer buffer = intrinsic_fbs::BuildJointLimits(kNumDof);

  INTR_EXPECT_OK(FlatbufferIsValid<intrinsic_fbs::JointLimits>(
      buffer.data(), buffer.size(), "some_name"));
}

// The validator only checks that all pointers of the flatbuffer are inside the
// buffer. It doesn't check the content of the flatbuffer.
TEST(FlatbufferIsValid, SucceedsForLargerSizeThanTable) {
  const size_t kNumDof = 6;
  flatbuffers::DetachedBuffer buffer = intrinsic_fbs::BuildJointLimits(kNumDof);

  INTR_EXPECT_OK(FlatbufferIsValid<intrinsic_fbs::JointLimits>(
      buffer.data(), buffer.size() + 1, "some_name"));
}

TEST(FlatbufferIsValid, FailsOnWrongSizeForTable) {
  const size_t kNumDof = 6;
  flatbuffers::DetachedBuffer buffer = intrinsic_fbs::BuildJointLimits(kNumDof);

  const Status s = FlatbufferIsValid<intrinsic_fbs::JointLimits>(
      buffer.data(), buffer.size() - 1, "some_name");
  EXPECT_EQ(s.code, StatusCode::kInvalidArgument);
  EXPECT_THAT(s.message,
              AllOf(HasSubstr("verification"), HasSubstr("some_name")));
}

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
