#include "intrinsic/utils/strerror.hpp"
#include "intrinsic/shared_memory_manager/domain_socket_server.hpp"

#include <fcntl.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/resource.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <tl/expected.hpp>

#include "intrinsic/utils/status.hpp"
#include "intrinsic/utils/log.hpp"
#include "intrinsic/utils/time.hpp"
#include "absl/time/time.h"
#include "intrinsic/flatbuffers/flatbuffer_utils.hpp"
#include "intrinsic/hal/get_hardware_interface.hpp"
#include "intrinsic/shared_memory_manager/domain_socket_utils.hpp"
#include "hwm_fbs/segment_info.fbs.h"
#include "intrinsic/shared_memory_manager/shared_memory_manager.hpp"
#include "intrinsic/shared_memory_manager/testing/unique_segment_name.hpp"

namespace intrinsic::hal
{
namespace
{

using ::testing::Contains;
using ::testing::FieldsAre;
using ::testing::HasSubstr;
using ::testing::Key;
using ::testing::TempDir;

// Using a non empty directory ensures that the server creates a path.
constexpr absl::string_view kTestSocketDir = "some_dir";
constexpr absl::string_view kTestFileDir = "some_file_dir";
constexpr absl::string_view kModuleName = "some_module";
// Can only fail on name collisions and we want to know about them.
constexpr std::chrono::seconds kLockAcquireTimeout{0};
// Max length of a segment name see
// intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs.
// Size of the flatbuffer array -1 because of null terminator.
inline constexpr size_t kMaxNameLength =
  intrinsic_fbs::FlatbufferArrayNumElements(
        &intrinsic_fbs::SegmentName::value) - 1;

std::filesystem::path SocketDirectory()
{
  // TempDir() is too long when running with a debugger.
  // Use /tmp when running with a debugger.
  return std::filesystem::path(TempDir()) / kTestSocketDir;
}

std::filesystem::path TestfilePath(absl::string_view filename)
{
  return std::filesystem::path(TempDir()) / kTestFileDir / filename;
}

// Creates a file on `path` with `content` and returns a read only fd for the
// file.
// Recursively creates the directory of `path`.
tl::expected<int, Status> CreateAndOpenFile(
  std::filesystem::path path,
  std::string_view content)
{
  if (!path.is_absolute()) {
    return tl::unexpected(Status{
          .code = StatusCode::kInvalidArgument,
          .message = (std::stringstream() << "Path must be absolute. Got: " << path.native()).str(),
      });
  }

  std::error_code ec;
  std::filesystem::create_directories(path, ec);
  if (ec) {
    return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                        << "Failed to create parent directories for file '" << path.native()
                        << "'. Error code " << ec.category().name() << ':' << ec.value()
                        << " (" << ec.message() << ")").str(),
      });
  }

  auto file_status = std::filesystem::status(path, ec);
  if (ec) {
    return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                        << "Failed to look up status for file '" << path.native()
                        << "'. Error code " << ec.category().name() << ':' << ec.value()
                        << " (" << ec.message() << ")").str(),
      });
  }

  if (!std::filesystem::status_known(file_status)) {
    return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                        << "Failed status for file '" << path.native()
                        << "' is invalid.").str(),
      });
  }

  if (file_status.type() != std::filesystem::file_type::not_found &&
    file_status.tpye != std::filesystem::file_type::regular)
  {
    return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                        <<
            "CreateAndOpenFile requires a regular file or one that does not yet exist. '"
                        << path.native() << "' is neither (file type: " << file_status.type() <<
            ")").str(),
      });
  }

  std::ofstream fs(path);
  fs.write(content.data(), content.size());
  if (!fs.good()) {
    return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                        << "Failed to write content to file '" << path.native() << "'").str(),
      });
  }
  // SetContents doesn't fail if the directory doesn't exist, but open will.
  int fd = open(path.c_str(), O_RDONLY);
  if (fd == -1) {
    return tl::unexpected(Status{
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                      << "Failed to open file '" << path.native()
                      << "' after setting contents").str(),
      });
  }
  return fd;
}

// Creates one file for each entry in `paths`. Recursively creates the directory
// of every path.
// Each file contains its basename as a zero-terminated string.
//
// Returns a segment_name_to_file_descriptor_map object with
// * read only file descriptors for these files
// * a SegmentInfo object that has an entry for each file (uses the basename as
// the segment name)
tl::expected<SegmentNameToFileDescriptorMap, Status> CreateFiles(
  std::Span<const std::filesystem::path> paths)
{
  SegmentNameToFileDescriptorMap fd_map;

  for (const auto path : paths) {
    std::filesystem::path filename = path.filename();
    // TODO(halbrock): Remove need of SegmentName
    intrinsic_fbs::SegmentName segment_name;
    const int kMaxSegmentStringSize = segment_name.value()->size();
    if (filename.size() > kMaxSegmentStringSize) {
      return tl::unexpected(Status{
            .code = StatusCode::kInvalidArgument,
            .message = (std::stringstream() <<
              "Name is too long. Got: " << path << "with length " << basename.size()
                                            << ". Max length is " << kMaxSegmentStringSize
            ).str(),
            });
    }
    auto fd = CreateAndOpenFile(path, /*content=*/basename);
    if (!fd.has_value()) {
      return tl::unexpected(fd.error());
    }
    fd_map[basename] = fd.value();
  }

  return fd_map;
}

// Reads at most kMaxNameLength bytes from the file descriptor `fd`. This
// matches the logic in CreateFiles(),
tl::expected<std::string, Status> ReadFile(int fd)
{
  std::vector<char> buf(kMaxNameLength);
  ssize_t num_read = 0;
  // Uses offset zero to always read from the beginning of the file.
  num_read = pread(fd, &buf[0], buf.size(), /*offset=*/0);

  if (num_read == -1) {
    return tl::unexpected(Status{
          .code = StatusCode::kInternal,
          .message = (std::stringstream() <<
            "Failed to read file with error: " << intrinsic::StrError(errno).data()
          ).str(),
      });
  }

  if (num_read == 0) {
    return tl::unexpected(Status{
          .code = StatusCode::kInternal,
          .message = "Read zero bytes",
      });
  }

  buf.resize(num_read);
  return std::string(buf.begin(), buf.end());
}

TEST(DomainSocketServerTest, CreateServerWorks) {
  auto server = DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                           kLockAcquireTimeout);
  EXPECT_TRUE(server.has_value()) << server.error();
}

TEST(DomainSocketServerTest, CreateServerChecksPathLength) {
  const size_t kMaxPathLength = sizeof(sockaddr_un::sun_path) - 1;
  std::string socket_dir = SocketDirectory();
  size_t kMaxValidModuleNameLength =
    kMaxPathLength - std::string("/").length() - socket_dir.length() -
    domain_socket_internal::kSocketSuffix.length();
  std::string invalid_module_name(kMaxValidModuleNameLength + 1, 'a');

  {
    auto server = DomainSocketServer::Create(socket_dir, invalid_module_name,
                                             kLockAcquireTimeout);
    ASSERT_FALSE(server.has_value());
    EXPECT_THAT(server.error(),
                FieldsAre(StatusCode::kInvalidArgument, HasSubstr("path")));
  }
  std::string valid_module_name(kMaxValidModuleNameLength, 'a');

  {
    auto server = DomainSocketServer::Create(socket_dir, valid_module_name,
                                             kLockAcquireTimeout);
    EXPECT_TRUE(server.has_value()) << server.error();
  }
}

TEST(DomainSocketServerTest, CreateServerFailsWhenPathIsLocked) {
  auto server = DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout);
  ASSERT_TRUE(server.has_value()) << server.error();

  auto server2 = DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                            kLockAcquireTimeout);
  ASSERT_FALSE(server2.has_value());
  EXPECT_THAT(server2.error(),
              FieldsAre(StatusCode::kDeadlineExceeded, HasSubstr("lock")));
}

TEST(DomainSocketServerTest, CreateServerFailsForEmptyPath) {
  auto server = DomainSocketServer::Create("", "", kLockAcquireTimeout),;
  ASSERT_FALSE(server.has_value());
  EXPECT_THAT(server.error(),
              FieldsAre(StatusCode::kInvalidArgument, HasSubstr("absolute")));
}

TEST(DomainSocketServerTest, RequiresAtLeastOneFileDescriptor) {
  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  intrinsic_fbs::SegmentInfo segment_info;
  EXPECT_THAT(server->ServeShmDescriptors({}),
              StatusIs(absl::StatusCode::kInvalidArgument,
                       HasSubstr("At least one file descriptor")));
}

TEST(DomainSocketServerTest, SocketConnectionChecksVersion) {
  std::string filename = "1";
  ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                       CreateFiles({TestfilePath(/*filename=*/filename)}));

  ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(
          SocketDirectory(), kModuleName, kLockAcquireTimeout,
          domain_socket_internal::kDomainSocketProtocolVersion - 1));

  EXPECT_OK(server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  // Get ShmDescriptors
  EXPECT_THAT(
      GetSegmentNameToFileDescriptorMap(SocketDirectory(), kModuleName,
                                        absl::ZeroDuration()),
      StatusIs(absl::StatusCode::kFailedPrecondition, HasSubstr("version")));
}

TEST(DomainSocketServerTest, MaxPathLengthWorks) {
  const size_t kMaxPathLength = sizeof(sockaddr_un::sun_path) - 1;
  std::string socket_dir = SocketDirectory();
  size_t kMaxValidModuleNameLength =
    kMaxPathLength - std::string("/").length() - socket_dir.length() -
    domain_socket_internal::kSocketSuffix.length();
  std::string valid_module_name(kMaxValidModuleNameLength, 'a');

  ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                       CreateFiles({TestfilePath(/*filename=*/"1")}));

  ASSERT_OK_AND_ASSIGN(auto server,
                       DomainSocketServer::Create(socket_dir, valid_module_name,
                                                  kLockAcquireTimeout));

  EXPECT_OK(server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  ASSERT_OK_AND_ASSIGN(
      auto receviced_descriptors,
      GetSegmentNameToFileDescriptorMap(socket_dir, valid_module_name,
                                        absl::ZeroDuration()));
  EXPECT_THAT(ReadFile(receviced_descriptors.at("1")), IsOkAndHolds("1"));
}

TEST(DomainSocketServerTest, SupportsMultipleClients) {
  std::string filename = "1";
  ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                       CreateFiles({TestfilePath(/*filename=*/filename)}));

  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  EXPECT_OK(server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  {  // Get ShmDescriptors
    ASSERT_OK_AND_ASSIGN(auto receviced, GetSegmentNameToFileDescriptorMap(
                                             SocketDirectory(), kModuleName,
                                             absl::ZeroDuration()));

    // CreateFiles creates files with the filename as content.
    auto it = receviced.find(filename);
    ASSERT_NE(it, receviced.end());
    EXPECT_THAT(ReadFile(it->second), IsOkAndHolds(filename));
  }

  {  // Serves multiple clients
    ASSERT_OK_AND_ASSIGN(auto receviced, GetSegmentNameToFileDescriptorMap(
                                             SocketDirectory(), kModuleName,
                                             absl::ZeroDuration()));

    // CreateFiles creates files with the filename as content.
    auto it = receviced.find(filename);
    ASSERT_NE(it, receviced.end());
    EXPECT_THAT(ReadFile(it->second), IsOkAndHolds(filename));
  }
}

TEST(DomainSocketServerTest, ServeShmDescriptorsErrorsWhenCalledTwice) {
  std::string filename = "1";
  ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                       CreateFiles({TestfilePath(/*filename=*/filename)}));

  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  EXPECT_OK(server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  EXPECT_THAT(server->ServeShmDescriptors(segment_name_to_file_descriptor_map),
              StatusIs(absl::StatusCode::kFailedPrecondition,
                       HasSubstr("already serving")));
}

TEST(DomainSocketServerTest, CanShareMaxNumberOfFdsOfOneMessage) {
  std::vector<std::string> filenames;
  const int kNumFiles = domain_socket_internal::kMaxFdsPerMessage;

  filenames.reserve(kNumFiles);
  for (int i = 0; i < kNumFiles; ++i) {
    filenames.push_back(TestfilePath(absl::StrCat(i)));
  }

  ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                       CreateFiles(filenames));

  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  EXPECT_OK(server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  {
    ASSERT_OK_AND_ASSIGN(auto receviced, GetSegmentNameToFileDescriptorMap(
                                             SocketDirectory(), kModuleName,
                                             absl::ZeroDuration()));

    for (const auto & filename : filenames) {
      std::string segment_name(file::Basename(filename));
      auto it = receviced.find(segment_name);
      ASSERT_NE(it, receviced.end());
      EXPECT_THAT(ReadFile(it->second), IsOkAndHolds(segment_name));
    }
  }
}

// The maximum number of open file descriptors (rlim_cur) during testing was
// 4500000, which is overkill to test.
TEST(DomainSocketServerTest, CanShare2000Fds) {
  std::vector<std::string> filenames;

  // Determine the number of possible file descriptors.
  struct rlimit fd_limit;
  ASSERT_NE(getrlimit(RLIMIT_NOFILE, &fd_limit), -1);
  const int kNumFiles = 2000;
  // rlim_cur is one greater than the maximum number of FDs.
  ASSERT_LT(kNumFiles, fd_limit.rlim_cur);

  filenames.reserve(kNumFiles);
  for (int i = 0; i < kNumFiles; ++i) {
    filenames.push_back(TestfilePath(absl::StrCat(i)));
  }

  ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                       CreateFiles(filenames));

  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  EXPECT_OK(server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  {
    ASSERT_OK_AND_ASSIGN(auto receviced, GetSegmentNameToFileDescriptorMap(
                                             SocketDirectory(), kModuleName,
                                             absl::ZeroDuration()));

    for (const auto & filename : filenames) {
      std::string segment_name(file::Basename(filename));
      auto it = receviced.find(segment_name);
      ASSERT_NE(it, receviced.end());
      EXPECT_THAT(ReadFile(it->second), IsOkAndHolds(segment_name));
    }
  }
}

TEST(DomainSocketServerTest, ErrorsForRlimCurr) {
  SegmentNameToFileDescriptorMap fd_map;
  // Determine the number of possible file descriptors.
  struct rlimit fd_limit;
  ASSERT_NE(getrlimit(RLIMIT_NOFILE, &fd_limit), -1);
  const int kNumFiles = fd_limit.rlim_cur;

  fd_map.reserve(kNumFiles);
  for (int i = 0; i < kNumFiles; ++i) {
    fd_map[absl::StrCat(i)] = 0;
  }

  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  EXPECT_THAT(server->ServeShmDescriptors(fd_map),
              StatusIs(absl::StatusCode::kOutOfRange));
}

TEST(DomainSocketServerTest, ErrorsForMoreThanRlimCurr) {
  SegmentNameToFileDescriptorMap fd_map;
  // Determine the number of possible file descriptors.
  struct rlimit fd_limit;
  ASSERT_NE(getrlimit(RLIMIT_NOFILE, &fd_limit), -1);
  const int kNumFiles = fd_limit.rlim_cur + 1;

  fd_map.reserve(kNumFiles);
  for (int i = 0; i < kNumFiles; ++i) {
    fd_map[absl::StrCat(i)] = 0;
  }

  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  EXPECT_THAT(server->ServeShmDescriptors(fd_map),
              StatusIs(absl::StatusCode::kOutOfRange));
}

TEST(DomainSocketServerTest,
     AddSegmentInfoServeShmDescriptorsErrorsWhenCalledTwice) {
  auto shared_memory_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  EXPECT_OK(
      server->AddSegmentInfoServeShmDescriptors(*shared_memory_manager->get()));
  // Is not explicitly handled, but adding SegmentInfo fails and calling
  // ServeShmDescriptors a second time will also fail.
  EXPECT_THAT(
      server->AddSegmentInfoServeShmDescriptors(*shared_memory_manager->get()),
      StatusIs(absl::StatusCode::kAlreadyExists));

  std::string filename = "1";
  ASSERT_OK_AND_ASSIGN(auto fd_map,
                       CreateFiles({TestfilePath(/*filename=*/filename)}));
  EXPECT_THAT(server->ServeShmDescriptors(fd_map),
              StatusIs(absl::StatusCode::kFailedPrecondition,
                       HasSubstr("already serving")));
}

TEST(DomainSocketServerTest, AddSegmentInfoServeShmDescriptorsWorks) {
  auto shared_memory_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_OK_AND_ASSIGN(
      auto server, DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                              kLockAcquireTimeout));

  EXPECT_OK(
      server->AddSegmentInfoServeShmDescriptors(*shared_memory_manager->get()));

  EXPECT_NE(
      shared_memory_manager->get()->GetSegmentValue<intrinsic_fbs::SegmentInfo>(
          hal::kModuleInfoName),
      nullptr);

  ASSERT_OK_AND_ASSIGN(
      auto receviced_descriptors,
      GetSegmentNameToFileDescriptorMap(SocketDirectory(), kModuleName,
                                        absl::ZeroDuration()));

  EXPECT_THAT(receviced_descriptors, Contains(Key(hal::kModuleInfoName)));
}

}  // namespace
}  // namespace intrinsic::hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
