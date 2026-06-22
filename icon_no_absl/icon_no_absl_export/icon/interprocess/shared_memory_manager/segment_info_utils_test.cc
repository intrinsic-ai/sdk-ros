#include "icon/interprocess/shared_memory_manager/segment_info_utils.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "icon/flatbuffers/flatbuffer_utils.h"

namespace intrinsic::icon {
namespace {

using ::intrinsic_fbs::FlatbufferArrayNumElements;
using ::testing::ElementsAreArray;

// Needs to match
// intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs
inline constexpr size_t kMaxSegmentNames =
    FlatbufferArrayNumElements(&intrinsic_fbs::SegmentInfo::names);

inline constexpr size_t kMaxFileDescriptorNames =
    FlatbufferArrayNumElements(&intrinsic_fbs::FileDescriptorNames::names);

// Returns a list of segment names that are valid for the SegmentInfo
// flatbuffer.
std::vector<std::string> ExpectedSegmentNames(size_t num_names) {
  std::vector<std::string> expected_names;
  expected_names.reserve(num_names);
  for (size_t i = 0; i < num_names; ++i) {
    // Needs to be short enough to fit into SegmentName::value.
    expected_names.push_back("name_" + std::to_string(i));
  }
  return expected_names;
}

intrinsic_fbs::SegmentInfo CreateSegmentInfo() {
  intrinsic_fbs::SegmentInfo segment_info;

  std::vector<std::string> expected_names =
      ExpectedSegmentNames(kMaxSegmentNames);

  for (size_t i = 0; i < kMaxSegmentNames; ++i) {
    intrinsic_fbs::SegmentName segment_name;

    // Copies the name including the null terminator of std::string.
    char* name_ptr =
        reinterpret_cast<char*>(segment_name.mutable_value()->Data());
    (void)std::snprintf(name_ptr, segment_name.value()->size(), "%s",
                        expected_names[i].c_str());

    segment_info.mutable_names()->Mutate(i, segment_name);
  }
  segment_info.mutate_size(kMaxSegmentNames);
  return segment_info;
}

// Generates `kMaxFileDescriptorNames` file descriptor names.
//
// We do allow more than `kMaxFileDescriptorNames` (namely, `kMaxSegmentNames`)
// overall segments.
//
// However, a single `FileDescriptorNames` message must not contain more than
// `kMaxFileDescriptorNames` segments. See the documentation in
// intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs
// for the reasoning behind this.
intrinsic_fbs::FileDescriptorNames CreateFileDescriptorNames() {
  intrinsic_fbs::FileDescriptorNames file_descriptor_names;

  std::vector<std::string> expected_names =
      ExpectedSegmentNames(kMaxFileDescriptorNames);

  for (size_t i = 0; i < kMaxFileDescriptorNames; ++i) {
    intrinsic_fbs::SegmentName segment_name;

    // Copies the name including the null terminator of std::string.
    char* name_ptr =
        reinterpret_cast<char*>(segment_name.mutable_value()->Data());
    (void)std::snprintf(name_ptr, segment_name.value()->size(), "%s",
                        expected_names[i].c_str());

    file_descriptor_names.mutable_names()->Mutate(i, segment_name);
  }
  file_descriptor_names.mutate_size(kMaxFileDescriptorNames);
  return file_descriptor_names;
}

TEST(SegmentInfo, GetNamesFromFileDescriptorNamesWorks) {
  const intrinsic_fbs::FileDescriptorNames file_descriptor_names =
      CreateFileDescriptorNames();

  auto names = GetNamesFromFileDescriptorNames(file_descriptor_names);
  ASSERT_TRUE(names.has_value()) << ToString(names.error());
  EXPECT_THAT(names.value(),
              ElementsAreArray(ExpectedSegmentNames(kMaxFileDescriptorNames)));
}

TEST(SegmentInfo, GetNamesFromSegmentInfoWorks) {
  const intrinsic_fbs::SegmentInfo segment_info = CreateSegmentInfo();
  auto names = GetNamesFromSegmentInfo(segment_info);
  ASSERT_TRUE(names.has_value()) << ToString(names.error());
  EXPECT_THAT(names.value(),
              ElementsAreArray(ExpectedSegmentNames(kMaxSegmentNames)));
}

TEST(SegmentInfo, GetRequiredInterfaceNamesFromSegmentInfoWorks) {
  intrinsic_fbs::SegmentInfo segment_info = CreateSegmentInfo();

  for (size_t i = 0; i < kMaxSegmentNames / 2; ++i) {
    segment_info.mutable_names()->GetMutablePointer(i)->mutate_must_be_used(
        true);
  }

  std::vector<std::string> expected_names =
      ExpectedSegmentNames(kMaxSegmentNames);
  expected_names.resize(kMaxSegmentNames / 2);
  auto names = GetRequiredInterfaceNamesFromSegmentInfo(segment_info);
  ASSERT_TRUE(names.has_value()) << ToString(names.error());
  EXPECT_THAT(names.value(), ElementsAreArray(expected_names));
}

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
