#include "intrinsic/shared_memory_manager/domain_socket_utils.hpp"

#include <sys/time.h>
#include <sys/file.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>
#include <chrono>
#include <thread>
#include <tl/expected.hpp>
#include <filesystem>
#include <sstream>

#include "intrinsic/utils/cleanup.hpp"
#include "intrinsic/utils/time.hpp"
#include "intrinsic/utils/log.hpp"
#include "intrinsic/utils/status.hpp"
#include "intrinsic/shared_memory_manager/segment_info_utils.hpp"
#include "hwm_fbs/segment_info.fbs.h"

namespace intrinsic::hal
{

namespace
{

constexpr std::string_view kDomainSocketDirectory = "/tmp/intrinsic_icon";

// Validates that the socket path fits into `sockaddr_un`.
Status PathLengthIsValidForSockaddrUn(std::filesystem::path socket_path)
{
  // sockaddr_un::sun_path requires a null terminator.
  const size_t kMaxSocketPathLength = sizeof(sockaddr_un::sun_path) - 1;
  if (socket_path.native().length() > kMaxSocketPathLength) {
    return Status {
      .code = StatusCode::kInvalidArgument,
      .message = (std::stringstream()
                   << "Socket path is too long. Got: " << socket_path << " with length "
                   << socket_path.native().length() << ". Max length is " <<
        kMaxSocketPathLength).str(),
    };
  }
  return OkStatus();
}

// Closes socket on error.
Status ConnectToServer(
  int to_server_sock,
  std::filesystem::path absolute_socket_path,
  Time deadline)
{
  auto addr =
    domain_socket_internal::AddressFromAbsolutePath(absolute_socket_path);
  if (!addr) {
    return addr.error();
  }

  bool logged_connection_retry = false;
  do {
    if (connect(to_server_sock, (sockaddr *)&(addr.value()), sizeof(sockaddr_un)) == 0) {
      LOG(INFO) << "Connected to server.";
      return OkStatus();
    }

    if (!logged_connection_retry) {
      LOG(WARNING)
            << "Failed to connect to socket '" << absolute_socket_path.native()
            << "' with error: " << strerror(errno) << " Retrying until "
            << deadline;
      logged_connection_retry = true;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } while (Now() < deadline);

  LOG(ERROR)
        << "Failed to connect to socket before " << deadline
        << ". Cleaning up.";
  if (close(to_server_sock) == -1) {
    LOG(WARNING)
          << "Failed to close socket fd for '" << absolute_socket_path.native()
          << "' with error: " << strerror(errno) << ".";
  }

  return Status {
    .code = StatusCode::kDeadlineExceeded,
    .message = (std::stringstream()
                 << "Failed to connect to socket '" << absolute_socket_path.native()
                 << "' before " << deadline << ".").str(),
  };
}

}   // namespace

namespace domain_socket_internal
{

tl::expected<std::filesystem::path, Status> AbsoluteSocketPath(
  std::filesystem::path absolute_path,
  std::string_view module_name)
{
  if (!absolute_path.is_absolute()) {
    return tl::unexpected(Status {
          .code = StatusCode::kInvalidArgument,
          .message = (std::stringstream()
                      << "The path `socket_directory` must be absolute. Got: "
                      << absolute_path).str(),
      });
  }

  std::filesystem::path full_socket_path =
    (absolute_path / module_name).concat(domain_socket_internal::kSocketSuffix);

  if (auto status = PathLengthIsValidForSockaddrUn(full_socket_path); !status.ok()) {
    return tl::unexpected(status);
  }
  return full_socket_path;
}

Status CreateSocketDirectory(std::filesystem::path absolute_path)
{
  if (!absolute_path.is_absolute()) {
    return Status {
      .code = StatusCode::kInvalidArgument,
      .message = (std::stringstream()
                   << "The path `socket_directory` must be absolute. Got: "
                   << absolute_path).str(),
    };
  }

  {
    std::error_code ec;
    std::filesystem::create_directories(absolute_path, ec);
    if (ec) {
      return Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                     << "Failed to create socket directory '" << absolute_path
                     << "'. Error code " << ec.category().name() << ':' << ec.value()
                     << " (" << ec.message() << ")").str(),
      };
    }
  }

  return OkStatus();
}

tl::expected<sockaddr_un, Status> AddressFromAbsolutePath(
  std::filesystem::path absolute_socket_path)
{
  sockaddr_un addr;
  memset(&addr, 0, sizeof(struct sockaddr_un));
  addr.sun_family = AF_UNIX;

  if(auto status = PathLengthIsValidForSockaddrUn(absolute_socket_path); !status.ok()) {
    return tl::unexpected(status);
  }
  // snprintf always includes the null terminator as required by
  // sockaddr_un::sun_path.
  if (std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s",
                    absolute_socket_path.c_str()) < 0)
  {
    return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                      << "Failed to copy socket path '" << absolute_socket_path
                      << "' to sockaddr_un struct with error: " << strerror(errno)).str(),
      });
  }
  return addr;
}

}   // namespace domain_socket_internal

// Receives a single message from the open socket `to_server_sock`.
// Returns InternalError if a received file descriptor is not valid.
// Returns InternalError on parsing errors.
// Returns InternalError when the size of the received message is wrong.
// Returns FailedPreconditionError when the socket protocol version of the
// message doesn't match domain_socket_internal::kDomainSocketProtocolVersion.
tl::expected<domain_socket_internal::ShmDescriptors, Status> GetSingleMessage(
  int to_server_sock)
{
  domain_socket_internal::ShmDescriptors descriptors;
  descriptors.file_descriptors_in_order.reserve(
      domain_socket_internal::kMaxFdsPerMessage);

  constexpr size_t kExpectedBytes = sizeof(descriptors.transfer_data);

  // Allocates size for the max number of fds.
  std::array<char, CMSG_SPACE(domain_socket_internal::kMaxFdsPerMessage *
                              sizeof(int))>
  cmsg_buf {0};

  iovec iov {.iov_base = (char *)(&descriptors.transfer_data),
    .iov_len = kExpectedBytes};

  msghdr msgh {
    .msg_name = nullptr,
    .msg_namelen = 0,
    .msg_iov = &iov,
    .msg_iovlen = 1,
    // ancillary data = file descriptors.
    .msg_control = cmsg_buf.data(),
    .msg_controllen = cmsg_buf.size(),
    .msg_flags = 0,
  };


  // Copy of the first message header, with the pointer to the ancillary data
  // that contains the file descriptors.
  msghdr first_msghdr = msgh;
  size_t received_bytes_sum = 0;

  // Receiving a single message can require multiple calls to recvmsg
  // https://gist.github.com/kentonv/bc7592af98c68ba2738f4436920868dc
  while (received_bytes_sum < kExpectedBytes) {
    if (received_bytes_sum > 0) {
      // recvmsg() transmits the control message (which contains the file
      // descriptors) on the first call. For subsequent calls, we set the
      // msg_control and msg_controllen fields of msgh to zero so recvmsg() does
      // not overwrite the control message.
      //
      // Because we made a copy of msgh before the loop, we can still access the
      // control message later.
      msgh.msg_control = nullptr;
      msgh.msg_controllen = 0;
    }

    // Returns the number of bytes received.
    ssize_t received_bytes = recvmsg(to_server_sock, &msgh, 0);
    if (received_bytes == -1) {
      return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream() <<
            "Failed to receive data with error: " << strerror(errno) << ".").str(),
        });
    }

    if (received_bytes == 0) {
      return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = "No data received. Likely due to shutdown.",
        });
    }

    received_bytes_sum += received_bytes;
    if ((received_bytes_sum) > kExpectedBytes) {
      return tl::unexpected(Status {
          .code = StatusCode::kOutOfRange,
          .message = (std::stringstream()
                        << "Received " << (received_bytes_sum)
                        << " bytes << but only expected " << kExpectedBytes << " bytes"
          ).str(),
        });
    }

    msgh.msg_iov->iov_base =
      (char *)(&descriptors.transfer_data) + received_bytes_sum;
    msgh.msg_iov->iov_len = kExpectedBytes - received_bytes_sum;
  }

  // // Returns error instead of retrying because the data is transmitted as a
  // // single message.
  if (received_bytes_sum != kExpectedBytes) {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                      << "Received " << received_bytes_sum
                      << " bytes. Expected " << kExpectedBytes
                      << " bytes").str(),
      });
  }

  if (descriptors.transfer_data.domain_socket_protocol_version !=
    domain_socket_internal::kDomainSocketProtocolVersion)
  {
    return tl::unexpected(Status {
        .code = StatusCode::kFailedPrecondition,
        .message = (std::stringstream()
                      << "Incompatible domain socket protocol version. Got: "
                      << descriptors.transfer_data.domain_socket_protocol_version << " expected: "
                      << domain_socket_internal::kDomainSocketProtocolVersion << "."
        ).str(),
      });
  }

  const int kNumNames = descriptors.transfer_data.file_descriptor_names.size();
  const auto segment_names =
    GetNamesFromFileDescriptorNames(
          descriptors.transfer_data.file_descriptor_names);
  if (!segment_names) {
    return tl::unexpected(segment_names.error());
  }

  // Parses fd data.
  struct cmsghdr * cmsg = CMSG_FIRSTHDR(&first_msghdr);
  if (!cmsg) {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = "No control message received",
      });
  }

  if (cmsg->cmsg_len != CMSG_LEN(kNumNames * sizeof(int))) {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                      << "Unexpected size of control message. Expected "
                      << CMSG_LEN(kNumNames * sizeof(int)) << " bytes got "
                      << cmsg->cmsg_len << " bytes"
        ).str(),
      });
  }

  if (cmsg->cmsg_level != SOL_SOCKET) {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                      << "Unexpected level of control message. Expected "
                      << SOL_SOCKET << " got " << cmsg->cmsg_level
        ).str(),
      });
  }
  if (cmsg->cmsg_type != SCM_RIGHTS) {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                      << "Unexpected type of control message. Expected '"
                      << SCM_RIGHTS << "' got '" << cmsg->cmsg_type << "'"
        ).str(),
      });
  }
  descriptors.file_descriptors_in_order.resize(kNumNames);
  // https://man7.org/linux/man-pages/man7/unix.7.html
  // If the number of file descriptors received in the ancillary data cause
  // the process to exceed its RLIMIT_NOFILE resource limit, the excess file
  // descriptors are automatically closed in the receiving process. One cannot
  // split the list over multiple recvmsg calls.
  for (int i = 0; i < kNumNames; ++i) {
    int fd = reinterpret_cast<int *>(CMSG_DATA(cmsg))[i];
    if (fcntl(fd, F_GETFD) == -1) {
      return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                        << "File descriptor for segment '" << (*segment_names)[i]
                        << "' is not valid. Either the receiving process can't open any more "
                        << "files, or the server sent a closed file descriptor. Error: "
                        << strerror(errno) << "."
          ).str(),
        });
    }

    descriptors.file_descriptors_in_order[i] = fd;
  }
  LOG(INFO)
      << "Received " << kNumNames << " valid file descriptors in message "
      << descriptors.transfer_data.message_index << " of "
      << descriptors.transfer_data.num_messages << ".";

  return descriptors;
}

tl::expected<SegmentNameToFileDescriptorMap, Status>
GetSegmentNameToFileDescriptorMap(
  std::filesystem::path socket_directory,
  std::string_view module_name,
  std::chrono::seconds connection_timeout)
{
  Time deadline = Now() + connection_timeout;
  int to_server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
  if (to_server_sock == -1) {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                      << "Failed to create GetShmDescriptors client socket with error: "
                      << strerror(errno) << "."
        ).str(),
      });
  }

  if (auto status = domain_socket_internal::CreateSocketDirectory(socket_directory);
    !status.ok())
  {
    return tl::unexpected(status);
  }

  auto absolute_socket_path = domain_socket_internal::AbsoluteSocketPath(
      socket_directory, module_name);
  if (!absolute_socket_path) {
    return tl::unexpected(absolute_socket_path.error());
  }

  if (auto status = ConnectToServer(to_server_sock, *absolute_socket_path, deadline);
    !status.ok())
  {
    return tl::unexpected(status);
  }

  // Closes the socket on exit
  Cleanup close_socket(
    [to_server_sock]() {
      if (close(to_server_sock) == -1) {
        LOG(ERROR)
          << "Failed to close GetShmDescriptors client socket with error: "
          << strerror(errno) << ".";
      }
    });

  // The socket blocks at most for one Second if no data is received.
  // This stops a misbehaving server from doing damage.
  struct timeval socket_receive_timeout
  {
    .tv_sec = 1,
    .tv_usec = 0,
  };
  if (setsockopt(to_server_sock, SOL_SOCKET, SO_RCVTIMEO,
    (const char *)&socket_receive_timeout,
                 sizeof socket_receive_timeout) == -1)
  {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                      << "Failed to set socket timeout with error: " << strerror(errno) << "."
        ).str(),
      });
  }

  SegmentNameToFileDescriptorMap segment_name_to_file_descriptor_map;
  // Reserves enough data for at least one message.
  // Reserves more data below once we know how many messages are expected.
  segment_name_to_file_descriptor_map.reserve(
      domain_socket_internal::kMaxFdsPerMessage);

  size_t remaining_messages = 0;
  size_t expected_message_index = 1;
  size_t expected_num_messages = 0;
  do {
    const auto message = GetSingleMessage(to_server_sock);
    if (!message) {
      return tl::unexpected(message.error());
    }

    if (expected_message_index != message->transfer_data.message_index) {
      return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                        << "Expected message with index " << expected_message_index
                        << " got " << message->transfer_data.message_index
          ).str(),
        });
    }
    // Reserve the correct space after receiving the first message.
    if (expected_message_index == 1) {
      expected_num_messages = message->transfer_data.num_messages;

      segment_name_to_file_descriptor_map.reserve(
          domain_socket_internal::kMaxFdsPerMessage * expected_num_messages);
    }

    if (expected_num_messages != message->transfer_data.num_messages) {
      return tl::unexpected(Status {
          .code = StatusCode::kFailedPrecondition,
          .message = (std::stringstream()
                        << "Expected " << expected_num_messages << " messages, but message ("
                        << message->transfer_data.message_index << ") declares "
                        << message->transfer_data.num_messages << "."
          ).str(),
        });
    }
    remaining_messages = expected_num_messages - expected_message_index;
    expected_message_index++;

    auto names = GetNamesFromFileDescriptorNames(
        message->transfer_data.file_descriptor_names);
    if (!names) {
      return tl::unexpected(names.error());
    }

    if (names->size() != message->file_descriptors_in_order.size()) {
      return tl::unexpected(Status {
          .code = StatusCode::kFailedPrecondition,
          .message = (std::stringstream()
                        << "Names and file descriptors have different sizes. Got: "
                        << names->size() << " and "
                        << message->file_descriptors_in_order.size() << "."
          ).str(),
        });
    }

    for (size_t i = 0; i < names->size(); ++i) {
      segment_name_to_file_descriptor_map[names->at(i)] =
        message->file_descriptors_in_order[i];
    }
  } while (remaining_messages > 0);

  return segment_name_to_file_descriptor_map;
}

std::filesystem::path SocketDirectoryFromNamespace(
  std::string_view shared_memory_namespace)
{
  auto socket_base_path = std::filesystem::path(kDomainSocketDirectory);
  if (shared_memory_namespace.empty()) {
    return socket_base_path;
  }
    // Ensure that we **don't** replace the base path if
    // `shared_memory_namespace` happens to be an absolute path (i.e. start with
    // a directory separator).
    //
    // But at the same time, we do want a separator if `shared_memory_namespace`
    // *doesn't* start with one. Appending the empty string to
    // `socket_base_path` ensures that there's a separator before
    // `shared_memory_namespace`.
  return (socket_base_path / "").concat(shared_memory_namespace);
}

}  // namespace intrinsic::hal
