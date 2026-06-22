#include "icon/interprocess/shared_memory_manager/domain_socket_utils.h"

#include <fcntl.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/un.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>

#include "icon/utils/log.h"
#include "icon/utils/mock_log_sink.h"
#include "icon/utils/status.h"
#include "icon/utils/time.h"

namespace intrinsic::icon {

namespace {

using ::testing::FieldsAre;
using ::testing::HasSubstr;
using ::testing::TempDir;

// domain_socket_internal
TEST(CreateSocketDirectory, ErrorsOnRelativePath) {
  EXPECT_THAT(domain_socket_internal::CreateSocketDirectory("tmp/some_dir"),
              FieldsAre(StatusCode::kInvalidArgument, HasSubstr("absolute")));
}

// domain_socket_internal
TEST(CreateSocketDirectory, Works) {
  auto socket_dir = std::filesystem::path(TempDir()) / "some_dir";

  EXPECT_EQ(domain_socket_internal::CreateSocketDirectory(socket_dir).code,
            StatusCode::kOk);

  std::error_code ec;
  EXPECT_TRUE(std::filesystem::exists(socket_dir, ec)) << ec;
  EXPECT_TRUE(std::filesystem::is_directory(socket_dir, ec)) << ec;
}

// domain_socket_internal
TEST(AbsoluteSocketPath, ErrorsOnRelativePath) {
  auto result = domain_socket_internal::AbsoluteSocketPath("tmp/some_path",
                                                           "some_module");
  ASSERT_FALSE(result.has_value());
  EXPECT_THAT(result.error(),
              FieldsAre(StatusCode::kInvalidArgument, HasSubstr("absolute")));
}

// domain_socket_internal
TEST(AbsoluteSocketPath, Works) {
  auto result = domain_socket_internal::AbsoluteSocketPath("/tmp/some_dir",
                                                           "some_module");
  ASSERT_TRUE(result.has_value()) << ToString(result.error());
  EXPECT_EQ(result.value(), "/tmp/some_dir/some_module.sock");
}

// domain_socket_internal
TEST(AddressFromAbsolutePath, ChecksLength) {
  std::string too_long_path(sizeof(sockaddr_un::sun_path) + 1, 'a');

  auto result = domain_socket_internal::AddressFromAbsolutePath(too_long_path);
  ASSERT_FALSE(result.has_value());
  EXPECT_THAT(result.error(),
              FieldsAre(StatusCode::kInvalidArgument, HasSubstr("too long")));
}

// domain_socket_internal
TEST(AddressFromAbsolutePath, Works) {
  std::filesystem::path socket_path =
      (std::filesystem::path(TempDir()) / "some_socket")
          .concat(domain_socket_internal::kSocketSuffix);

  auto addr = domain_socket_internal::AddressFromAbsolutePath(socket_path);
  ASSERT_TRUE(addr.has_value()) << ToString(addr.error());

  EXPECT_EQ(addr->sun_family, AF_UNIX);
  EXPECT_EQ(addr->sun_path, socket_path);
  // Checks that the null terminator is set.
  EXPECT_EQ(addr->sun_path[sizeof(addr->sun_path) - 1], '\0');
}

TEST(SocketDirectoryFromNamespace, Works) {
  EXPECT_EQ(SocketDirectoryFromNamespace(""), "/tmp/intrinsic_icon");

  EXPECT_EQ(SocketDirectoryFromNamespace("some_namespace"),
            "/tmp/intrinsic_icon/some_namespace");

  EXPECT_EQ(SocketDirectoryFromNamespace("/some_namespace"),
            "/tmp/intrinsic_icon/some_namespace");
}

class GetSegmentNameToFileDescriptorMapTest : public ::testing::Test {
 public:
  GetSegmentNameToFileDescriptorMapTest()
      : logger_(log::Logger::Severity::kInfo, log::MockSink(log_entries_)) {}

  log::Logger* logger() { return &logger_; }

  std::span<const log::LogEntryWithStorage> log_entries() {
    return log_entries_;
  }

 private:
  std::vector<log::LogEntryWithStorage> log_entries_;
  log::Logger logger_;
};

TEST_F(GetSegmentNameToFileDescriptorMapTest, CreatesPath) {
  auto socket_dir = std::filesystem::path(TempDir()) / "some_dir";
  (void)GetSegmentNameToFileDescriptorMap(socket_dir, "some_module",
                                          std::chrono::seconds(0), logger());
  std::error_code ec;
  EXPECT_TRUE(std::filesystem::exists(socket_dir, ec)) << ec;
  EXPECT_TRUE(std::filesystem::is_directory(socket_dir, ec)) << ec;
}

TEST_F(GetSegmentNameToFileDescriptorMapTest, TimesOut) {
  auto result = GetSegmentNameToFileDescriptorMap(
      TempDir(), "some_module", std::chrono::seconds(0), logger());
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, StatusCode::kDeadlineExceeded);
}

// GetSegmentNameToFileDescriptorMap receiving descriptors and the version check
// is validated in
// intrinsic/icon/interprocess/shared_memory_manager/domain_socket_server_test.cc

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
