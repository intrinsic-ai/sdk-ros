#include "icon/interprocess/shared_memory_manager/domain_socket_server.h"

#include <fcntl.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/resource.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <tl/expected.hpp>
#include <vector>

#include "flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "icon/flatbuffers/flatbuffer_utils.h"
#include "icon/hal/get_hardware_interface.h"
#include "icon/interprocess/shared_memory_manager/domain_socket_utils.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"
#include "icon/utils/cleanup.h"
#include "icon/utils/log.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_test_macros.h"
#include "icon/utils/strerror.h"

namespace intrinsic::icon {
namespace {

using ::testing::Contains;
using ::testing::FieldsAre;
using ::testing::HasSubstr;
using ::testing::Key;
using ::testing::TempDir;

// Using a non empty directory ensures that the server creates a path.
constexpr std::string_view kTestSocketDir = "some_dir";
constexpr std::string_view kTestFileDir = "some_file_dir";
constexpr std::string_view kModuleName = "some_module";
// Can only fail on name collisions and we want to know about them.
constexpr std::chrono::seconds kLockAcquireTimeout{0};
// Max length of a segment name see
// intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs.
// Size of the flatbuffer array -1 because of null terminator.
inline constexpr size_t kMaxNameLength =
    intrinsic_fbs::FlatbufferArrayNumElements(
        &intrinsic_fbs::SegmentName::value) -
    1;

// Creates a file on `path` with `content` and returns a read only fd for the
// file.
// Recursively creates the directory of `path`.
tl::expected<int, Status> CreateAndOpenFile(std::filesystem::path path,
                                            std::string_view content) {
  if (!path.is_absolute()) {
    return tl::unexpected(Status{
        .code = StatusCode::kInvalidArgument,
        .message = (std::stringstream()
                    << "Path must be absolute. Got: " << path.native())
                       .str(),
    });
  }

  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (ec) {
    return tl::unexpected(Status{
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                    << "Failed to create parent directories for file '"
                    << path.native() << "'. Error code " << ec.category().name()
                    << ':' << ec.value() << " (" << ec.message() << ")")
                       .str(),
    });
  }

  auto file_status = std::filesystem::status(path, ec);

  if (!std::filesystem::status_known(file_status)) {
    if (ec) {
      return tl::unexpected(Status{
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                      << "Failed to look up status for file '" << path.native()
                      << "'. Error code " << ec.category().name() << ':'
                      << ec.value() << " (" << ec.message() << ")")
                         .str(),
      });
    }
    return tl::unexpected(Status{
        .code = StatusCode::kInternal,
        .message = (std::stringstream() << "Failed status for file '"
                                        << path.native() << "' is invalid.")
                       .str(),
    });
  }

  if (file_status.type() != std::filesystem::file_type::not_found &&
      file_status.type() != std::filesystem::file_type::regular) {
    return tl::unexpected(Status{
        .code = StatusCode::kInternal,
        .message =
            (std::stringstream() << "CreateAndOpenFile requires a regular file "
                                    "or one that does not yet exist. '"
                                 << path.native() << "' is neither ("
                                 << static_cast<int>(file_status.type()) << ")")
                .str(),
    });
  }

  std::ofstream fs(path);
  fs.write(content.data(), content.size());
  if (!fs.good()) {
    return tl::unexpected(Status{
        .code = StatusCode::kInternal,
        .message = (std::stringstream() << "Failed to write content to file '"
                                        << path.native() << "'")
                       .str(),
    });
  }
  // SetContents doesn't fail if the directory doesn't exist, but open will.
  int fd = open(path.c_str(), O_RDONLY);
  if (fd == -1) {
    return tl::unexpected(Status{
        .code = StatusCode::kInternal,
        .message =
            (std::stringstream() << "Failed to open file '" << path.native()
                                 << "' after setting contents: "
                                 << intrinsic::StrError(errno).data())
                .str(),
    });
  }
  return fd;
}

// Creates one file for each entry in `paths`. Recursively creates the directory
// of every path.
// Each file contains its filename as a zero-terminated string.
//
// Returns a segment_name_to_file_descriptor_map object with
// * read only file descriptors for these files
// * a SegmentInfo object that has an entry for each file (uses the filename as
// the segment name)
tl::expected<SegmentNameToFileDescriptorMap, Status> CreateFiles(
    const std::vector<std::filesystem::path>& paths) {
  SegmentNameToFileDescriptorMap fd_map;

  for (const auto& path : paths) {
    std::filesystem::path filename = path.filename();
    // TODO(halbrock): Remove need of SegmentName
    intrinsic_fbs::SegmentName segment_name;
    const int kMaxSegmentStringSize = segment_name.value()->size();
    if (filename.native().size() > kMaxSegmentStringSize) {
      return tl::unexpected(Status{
          .code = StatusCode::kInvalidArgument,
          .message = (std::stringstream()
                      << "Name is too long. Got: " << path << "with length "
                      << filename.native().size() << ". Max length is "
                      << kMaxSegmentStringSize)
                         .str(),
      });
    }
    auto fd = CreateAndOpenFile(path, /*content=*/filename.native());
    if (!fd.has_value()) {
      return tl::unexpected(fd.error());
    }
    fd_map[filename] = fd.value();
  }

  return fd_map;
}

// Reads at most kMaxNameLength bytes from the file descriptor `fd`. This
// matches the logic in CreateFiles(),
tl::expected<std::string, Status> ReadFile(int fd) {
  std::vector<char> buf(kMaxNameLength);
  ssize_t num_read = 0;
  // Uses offset zero to always read from the beginning of the file.
  num_read = pread(fd, &buf[0], buf.size(), /*offset=*/0);

  if (num_read == -1) {
    return tl::unexpected(Status{
        .code = StatusCode::kInternal,
        .message = (std::stringstream() << "Failed to read file with error: "
                                        << intrinsic::StrError(errno).data())
                       .str(),
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

log::Logger::SinkCallback StdErrSink() {
  return [](const log::Logger::LogEntry& entry) {
    std::string severity_name = "";
    switch (entry.severity) {
      case log::Logger::Severity::kDebug:
        severity_name = "DEBUG";
        break;
      case log::Logger::Severity::kInfo:
        severity_name = " INFO";
        break;
      case log::Logger::Severity::kWarning:
        severity_name = " WARN";
        break;
      case log::Logger::Severity::kError:
        severity_name = "ERROR";
        break;
      case log::Logger::Severity::kFatal:
        severity_name = "FATAL";
        break;
      default:
        severity_name = "?????";
        break;
    }
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
    std::cerr << severity_name << "[" << seconds << "] "
              << entry.loc.file_name() << ":" << entry.loc.line() << " | "
              << entry.msg << std::endl;
  };
}

class DomainSocketServerTest : public ::testing::Test {
 public:
  void SetUp() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(1, 99999);
    std::filesystem::path base = TempDir();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
    temp_dir_ =
        base / (std::to_string(seconds) + "_" + std::to_string(distrib(gen)));
    bool temp_dir_is_new = std::filesystem::create_directory(temp_dir_);
    ASSERT_TRUE(temp_dir_is_new) << "Failed to create a unique test directory.";
  }

  std::filesystem::path SocketDirectory() {
    // TempDir() is too long when running with a debugger.
    // Use /tmp when running with a debugger.
    return temp_dir_ / kTestSocketDir;
  }

  std::filesystem::path TestfilePath(std::string_view filename) {
    return temp_dir_ / kTestFileDir / filename;
  }

 private:
  std::filesystem::path temp_dir_;
};

TEST_F(DomainSocketServerTest, CreateServerWorks) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  INTR_EXPECT_OK(DomainSocketServer::Create(
      SocketDirectory(), kModuleName, kLockAcquireTimeout, /*logger=*/&logger));
}

TEST_F(DomainSocketServerTest, CreateServerChecksPathLength) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  const size_t kMaxPathLength = sizeof(sockaddr_un::sun_path) - 1;
  std::string socket_dir = SocketDirectory();
  size_t kMaxValidModuleNameLength =
      kMaxPathLength - std::string("/").length() - socket_dir.length() -
      domain_socket_internal::kSocketSuffix.length();
  std::string invalid_module_name(kMaxValidModuleNameLength + 1, 'a');

  {
    auto server =
        DomainSocketServer::Create(socket_dir, invalid_module_name,
                                   kLockAcquireTimeout, /*logger=*/&logger);
    ASSERT_FALSE(server.has_value());
    EXPECT_THAT(server.error(),
                FieldsAre(StatusCode::kInvalidArgument, HasSubstr("path")));
  }
  std::string valid_module_name(kMaxValidModuleNameLength, 'a');

  INTR_EXPECT_OK(DomainSocketServer::Create(
      socket_dir, valid_module_name, kLockAcquireTimeout, /*logger=*/&logger));
}

TEST_F(DomainSocketServerTest, CreateServerFailsWhenPathIsLocked) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));

  auto server2 = DomainSocketServer::Create(
      SocketDirectory(), kModuleName, kLockAcquireTimeout, /*logger=*/&logger);
  ASSERT_FALSE(server2.has_value());
  EXPECT_THAT(server2.error(),
              FieldsAre(StatusCode::kDeadlineExceeded, HasSubstr("lock")));
}

TEST_F(DomainSocketServerTest, CreateServerFailsForEmptyPath) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  auto server = DomainSocketServer::Create("", "", kLockAcquireTimeout,
                                           /*logger=*/&logger);
  ASSERT_FALSE(server.has_value());
  EXPECT_THAT(server.error(),
              FieldsAre(StatusCode::kInvalidArgument, HasSubstr("absolute")));
}

TEST_F(DomainSocketServerTest, RequiresAtLeastOneFileDescriptor) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));

  intrinsic_fbs::SegmentInfo segment_info;
  Status s = server->ServeShmDescriptors({});
  EXPECT_EQ(s.code, StatusCode::kInvalidArgument) << ToString(s);
  EXPECT_THAT(s.message, HasSubstr("At least one file descriptor"))
      << ToString(s);
}

TEST_F(DomainSocketServerTest, SocketConnectionChecksVersion) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  std::string filename = "1";
  INTR_ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                            CreateFiles({TestfilePath(/*filename=*/filename)}));
  Cleanup close_fds([&]() noexcept {
    for (const auto& [_, fd] : segment_name_to_file_descriptor_map) {
      if (int r = close(fd); r != 0) {
        std::cerr << "Failed to close FD " << fd << ": "
                  << std::string(StrError(errno).data()) << std::endl;
      }
    }
  });

  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(
          SocketDirectory(), kModuleName, kLockAcquireTimeout,
          /*logger=*/&logger,
          domain_socket_internal::kDomainSocketProtocolVersion - 1));

  INTR_EXPECT_OK(
      server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  // Get ShmDescriptors
  auto received_fd_map = GetSegmentNameToFileDescriptorMap(
      SocketDirectory(), kModuleName, kLockAcquireTimeout, /*logger=*/&logger);
  ASSERT_FALSE(received_fd_map.has_value());
  EXPECT_EQ(received_fd_map.error().code, StatusCode::kFailedPrecondition);
  EXPECT_THAT(received_fd_map.error().message, HasSubstr("version"));
}

TEST_F(DomainSocketServerTest, MaxPathLengthWorks) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  const size_t kMaxPathLength = sizeof(sockaddr_un::sun_path) - 1;
  std::string socket_dir = SocketDirectory();
  size_t kMaxValidModuleNameLength =
      kMaxPathLength - std::string("/").length() - socket_dir.length() -
      domain_socket_internal::kSocketSuffix.length();
  std::string valid_module_name(kMaxValidModuleNameLength, 'a');

  INTR_ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                            CreateFiles({TestfilePath(/*filename=*/"1")}));
  Cleanup close_fds([&]() noexcept {
    for (const auto& [_, fd] : segment_name_to_file_descriptor_map) {
      if (int r = close(fd); r != 0) {
        std::cerr << "Failed to close FD " << fd << ": "
                  << std::string(StrError(errno).data()) << std::endl;
      }
    }
  });
  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), valid_module_name,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  INTR_EXPECT_OK(
      server->ServeShmDescriptors(segment_name_to_file_descriptor_map));
  INTR_ASSERT_OK_AND_ASSIGN(auto received_descriptors,
                            GetSegmentNameToFileDescriptorMap(
                                socket_dir, valid_module_name,
                                kLockAcquireTimeout, /*logger=*/&logger));
  INTR_ASSERT_OK_AND_ASSIGN(auto read_result,
                            ReadFile(received_descriptors.at("1")));
  EXPECT_EQ(read_result, "1");
}

TEST_F(DomainSocketServerTest, SupportsMultipleClients) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  std::string filename = "1";

  INTR_ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                            CreateFiles({TestfilePath(/*filename=*/filename)}));
  Cleanup close_fds([&]() noexcept {
    for (const auto& [_, fd] : segment_name_to_file_descriptor_map) {
      if (int r = close(fd); r != 0) {
        std::cerr << "Failed to close FD " << fd << ": "
                  << std::string(StrError(errno).data()) << std::endl;
      }
    }
  });
  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  INTR_EXPECT_OK(
      server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  {  // Get ShmDescriptors
    INTR_ASSERT_OK_AND_ASSIGN(
        auto received, GetSegmentNameToFileDescriptorMap(
                           SocketDirectory(), kModuleName, kLockAcquireTimeout,
                           /*logger=*/&logger));

    // CreateFiles creates files with the filename as content.
    auto it = received.find(filename);
    ASSERT_NE(it, received.end());
    INTR_ASSERT_OK_AND_ASSIGN(auto read_result, ReadFile(it->second));
    EXPECT_EQ(read_result, filename);
  }

  {  // Serves multiple clients
    INTR_ASSERT_OK_AND_ASSIGN(
        auto received, GetSegmentNameToFileDescriptorMap(
                           SocketDirectory(), kModuleName, kLockAcquireTimeout,
                           /*logger=*/&logger));

    // CreateFiles creates files with the filename as content.
    auto it = received.find(filename);
    ASSERT_NE(it, received.end());
    INTR_ASSERT_OK_AND_ASSIGN(auto read_result, ReadFile(it->second));
    EXPECT_EQ(read_result, filename);
  }
}

TEST_F(DomainSocketServerTest, ServeShmDescriptorsErrorsWhenCalledTwice) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  std::string filename = "1";
  INTR_ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                            CreateFiles({TestfilePath(/*filename=*/filename)}));
  Cleanup close_fds([&]() noexcept {
    for (const auto& [_, fd] : segment_name_to_file_descriptor_map) {
      if (int r = close(fd); r != 0) {
        std::cerr << "Failed to close FD " << fd << ": "
                  << std::string(StrError(errno).data()) << std::endl;
      }
    }
  });

  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  INTR_EXPECT_OK(
      server->ServeShmDescriptors(segment_name_to_file_descriptor_map));
  {
    Status s = server->ServeShmDescriptors(segment_name_to_file_descriptor_map);
    EXPECT_EQ(s.code, StatusCode::kFailedPrecondition) << ToString(s);
    EXPECT_THAT(s.message, HasSubstr("already serving"));
  }
}

TEST_F(DomainSocketServerTest, CanShareMaxNumberOfFdsOfOneMessage) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  std::vector<std::filesystem::path> paths;
  const int kNumFiles = domain_socket_internal::kMaxFdsPerMessage;

  paths.reserve(kNumFiles);
  for (int i = 0; i < kNumFiles; ++i) {
    paths.push_back(TestfilePath(std::to_string(i)));
  }

  INTR_ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                            CreateFiles(paths));
  Cleanup close_fds([&]() noexcept {
    for (const auto& [_, fd] : segment_name_to_file_descriptor_map) {
      if (int r = close(fd); r != 0) {
        std::cerr << "Failed to close FD " << fd << ": "
                  << std::string(StrError(errno).data()) << std::endl;
      }
    }
  });

  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  INTR_EXPECT_OK(
      server->ServeShmDescriptors(segment_name_to_file_descriptor_map));
  {
    INTR_ASSERT_OK_AND_ASSIGN(
        auto received, GetSegmentNameToFileDescriptorMap(
                           SocketDirectory(), kModuleName, kLockAcquireTimeout,
                           /*logger=*/&logger));

    for (const auto& path : paths) {
      std::string segment_name(path.filename().native());
      auto it = received.find(segment_name);
      ASSERT_NE(it, received.end());
      INTR_ASSERT_OK_AND_ASSIGN(auto read_result, ReadFile(it->second));
      EXPECT_EQ(read_result, segment_name);
    }
  }
}

// The maximum number of open file descriptors (rlim_cur) during testing was
// 4500000, which is overkill to test.
TEST_F(DomainSocketServerTest, CanShare2000Fds) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  std::vector<std::filesystem::path> paths;

  // Determine the number of possible file descriptors.
  struct rlimit fd_limit;
  ASSERT_NE(getrlimit(RLIMIT_NOFILE, &fd_limit), -1)
      << std::string(intrinsic::StrError(errno).data());
  struct rlimit original_limit = fd_limit;
  Cleanup reset_rlimit(
      [&]() noexcept { setrlimit(RLIMIT_NOFILE, &original_limit); });

  fd_limit.rlim_cur = fd_limit.rlim_max;
  ASSERT_NE(setrlimit(RLIMIT_NOFILE, &fd_limit), -1)
      << std::string(intrinsic::StrError(errno).data());

  // rlim_cur is one greater than the maximum number of FDs, but leave some room
  // for non-domain socket FDs.
  const unsigned long kNumFiles =
      std::min(std::max(1ul, fd_limit.rlim_cur / 2), 2000ul);
  paths.reserve(kNumFiles);
  for (unsigned long i = 0; i < kNumFiles; ++i) {
    paths.push_back(TestfilePath(std::to_string(i)));
  }

  INTR_ASSERT_OK_AND_ASSIGN(auto segment_name_to_file_descriptor_map,
                            CreateFiles(paths));
  Cleanup close_fds([&]() noexcept {
    for (const auto& [_, fd] : segment_name_to_file_descriptor_map) {
      if (int r = close(fd); r != 0) {
        std::cerr << "Failed to close FD " << fd << ": "
                  << std::string(StrError(errno).data()) << std::endl;
      }
    }
  });

  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  INTR_EXPECT_OK(
      server->ServeShmDescriptors(segment_name_to_file_descriptor_map));

  INTR_ASSERT_OK_AND_ASSIGN(
      auto received, GetSegmentNameToFileDescriptorMap(
                         SocketDirectory(), kModuleName, kLockAcquireTimeout,
                         /*logger=*/&logger));

  for (const auto& path : paths) {
    std::string segment_name(path.filename().native());
    auto it = received.find(segment_name);
    ASSERT_NE(it, received.end());
    INTR_ASSERT_OK_AND_ASSIGN(auto read_result, ReadFile(it->second));
    EXPECT_EQ(read_result, segment_name);
  }
}

TEST_F(DomainSocketServerTest, ErrorsForRlimCurr) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  SegmentNameToFileDescriptorMap fd_map;
  // Determine the number of possible file descriptors.
  struct rlimit fd_limit;
  ASSERT_NE(getrlimit(RLIMIT_NOFILE, &fd_limit), -1);
  const int kNumFiles = fd_limit.rlim_cur;

  fd_map.reserve(kNumFiles);
  for (int i = 0; i < kNumFiles; ++i) {
    fd_map[std::to_string(i)] = 0;
  }

  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  EXPECT_EQ(server->ServeShmDescriptors(fd_map).code, StatusCode::kOutOfRange);
}

TEST_F(DomainSocketServerTest, ErrorsForMoreThanRlimCurr) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  SegmentNameToFileDescriptorMap fd_map;
  // Determine the number of possible file descriptors.
  struct rlimit fd_limit;
  ASSERT_NE(getrlimit(RLIMIT_NOFILE, &fd_limit), -1);
  const int kNumFiles = fd_limit.rlim_cur + 1;

  fd_map.reserve(kNumFiles);
  for (int i = 0; i < kNumFiles; ++i) {
    fd_map[std::to_string(i)] = 0;
  }

  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  EXPECT_EQ(server->ServeShmDescriptors(fd_map).code, StatusCode::kOutOfRange);
}

TEST_F(DomainSocketServerTest,
       AddSegmentInfoServeShmDescriptorsErrorsWhenCalledTwice) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shared_memory_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module",
                                  /*logger=*/&logger));
  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  INTR_EXPECT_OK(
      server->AddSegmentInfoServeShmDescriptors(*shared_memory_manager));
  // Is not explicitly handled, but adding SegmentInfo fails and calling
  // ServeShmDescriptors a second time will also fail.
  {
    Status s =
        server->AddSegmentInfoServeShmDescriptors(*shared_memory_manager);
    EXPECT_EQ(s.code, StatusCode::kAlreadyExists) << ToString(s);
  }

  std::string filename = "1";
  INTR_ASSERT_OK_AND_ASSIGN(auto fd_map,
                            CreateFiles({TestfilePath(/*filename=*/filename)}));
  Cleanup close_fds([&]() noexcept {
    for (const auto& [_, fd] : fd_map) {
      if (int r = close(fd); r != 0) {
        std::cerr << "Failed to close FD " << fd << ": "
                  << std::string(StrError(errno).data()) << std::endl;
      }
    }
  });
  {
    Status s = server->ServeShmDescriptors(fd_map);
    EXPECT_EQ(s.code, StatusCode::kFailedPrecondition) << ToString(s);
    EXPECT_THAT(s.message, HasSubstr("already serving")) << ToString(s);
  }
}

TEST_F(DomainSocketServerTest, AddSegmentInfoServeShmDescriptorsWorks) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shared_memory_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module",
                                  /*logger=*/&logger));
  INTR_ASSERT_OK_AND_ASSIGN(
      auto server,
      DomainSocketServer::Create(SocketDirectory(), kModuleName,
                                 kLockAcquireTimeout, /*logger=*/&logger));
  INTR_EXPECT_OK(
      server->AddSegmentInfoServeShmDescriptors(*shared_memory_manager));

  EXPECT_NE(shared_memory_manager->GetSegmentValue<intrinsic_fbs::SegmentInfo>(
                icon::kModuleInfoName),
            nullptr);

  INTR_ASSERT_OK_AND_ASSIGN(
      auto received_descriptors,
      GetSegmentNameToFileDescriptorMap(SocketDirectory(), kModuleName,
                                        kLockAcquireTimeout,
                                        /*logger=*/&logger));

  EXPECT_THAT(received_descriptors, Contains(Key(icon::kModuleInfoName)));
}

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
