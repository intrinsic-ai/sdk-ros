#include "intrinsic/utils/strerror.hpp"
#include "intrinsic/shared_memory_manager/domain_socket_server.hpp"

#include <sys/file.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/socket.h>

#include <mutex>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <thread>
#include <tl/expected.hpp>

#include "intrinsic/utils/log.hpp"
#include "intrinsic/utils/status.hpp"
#include "intrinsic/utils/cleanup.hpp"
#include "intrinsic/shared_memory_manager/domain_socket_utils.hpp"
#include "intrinsic/hal/get_hardware_interface.hpp"
#include "intrinsic/shared_memory_manager/shared_memory_manager.hpp"
#include "intrinsic/utils/time.hpp"
#include "intrinsic/flatbuffers/fixed_string.hpp"

#include "hwm_fbs/segment_info.fbs.h"


namespace intrinsic::hal
{

namespace
{

constexpr std::string_view kLockSuffix = ".lock";

// Returns the path of the lockfile.
std::filesystem::path LockName(
  std::filesystem::path socket_directory,
  std::string_view module_name)
{
  return (socket_directory / module_name).concat(kLockSuffix);
}

// Uses flock (https://linux.die.net/man/2/flock) to lock `absolute_lock_path`.
// The operating system automatically releases the Lock when the process exits.
// An alternative is using https://linux.die.net/man/2/fcntl on the
// socket fd.
// Tries once when `timeout` is zero or negative.
tl::expected<int, Status> TryLockPath(
  std::filesystem::path absolute_lock_path,
  std::chrono::seconds timeout)
{
  Time deadline = Now() + timeout;
  if (absolute_lock_path.empty()) {
    return tl::unexpected(Status {
          .code = StatusCode::kInvalidArgument,
          .message = "path cannot be empty",
      });
  }

  if (!absolute_lock_path.is_absolute()) {
    return tl::unexpected(Status {
          .code = StatusCode::kInvalidArgument,
          .message = (std::stringstream()
                  << "Path must be absolute. Got: " << absolute_lock_path.native()).str(),
         });
  }
  // Don't delete the lock file, even on shutdown.
  // Otherwise there is a potential race between deletion and re-creation of the
  // file.
  // Example of the race where a new process is started while the old is
  // shutting down:
  //
  // 1. The newer process checks for existence of the lock file, finds the
  //    file, and waits to acquire the lock.
  // 2. The older process then executes the DomainSocketServer destructor
  //    as part of its shutdown, and releases the lock.
  // 3. The newer process acquires the lock.
  // 4. The older process deletes the lock file.
  // 5. Another process may acquire the lock at any time, because the file
  //    doesn't exist on the filesystem anymore.
  int lock_fd = open(absolute_lock_path.c_str(), O_WRONLY | O_CREAT,
                     S_IRWXU | S_IRWXG | S_IRWXO);

  if (lock_fd == -1) {
    return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                      << "Unable to open lock '" << absolute_lock_path.native()
                      << "' with error: " << intrinsic::StrError(errno).data()).str(),
      });
  }

  Cleanup close_lock_fd(
    [&lock_fd, absolute_lock_path]() {
      if (close(lock_fd) == -1) {
        LOG(WARNING)   << "Failed to close lock fd for '" << absolute_lock_path.native()
                       << "' with error: " << intrinsic::StrError(errno).data() << ".";
      }
    });
  // Try once even if the deadline has passed.
  bool logged_error = false;
  do {
    // Uses LOCK_NB to make flock nonblocking and avoid blocking the process.
    if (int err = flock(lock_fd, LOCK_EX | LOCK_NB); err == 0) {
      // TODO(nilsb): proper logging
      std::cerr << "INFO: Acquired exclusive lock '" << absolute_lock_path.native() << "'." <<
        std::endl;
      std::move(close_lock_fd).Cancel();
      return lock_fd;
    } else {
      if (errno == EWOULDBLOCK) {
        if (!logged_error) {
          std::cerr
                << "WARN: File '" << absolute_lock_path.native()
                << "' is locked by another process, will retry until " << deadline
                << "."
                << std::endl;
          logged_error = true;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      } else {
        return tl::unexpected(Status {
              .code = StatusCode::kInternal,
              .message = (std::stringstream()
                          << "Unable to lock '" << absolute_lock_path.native()
                          << "' with error: " << intrinsic::StrError(errno).data()).str(),
          });
      }
    }
  } while (Now() < deadline);

  return tl::unexpected(Status {
        .code = StatusCode::kDeadlineExceeded,
        .message = (std::stringstream()
                    << "Timed out waiting for lock '" << absolute_lock_path.native()
                    << "' to become exclusive").str(),
    });
}

// Releases the flock held by the server.
// It's important that the lock file is not deleted on shutdown.
// Otherwise there is a potential race between deletion and re-creation.
Status UnlockPathCloseLockfile(int lockfile_fd)
{
  if (lockfile_fd == -1) {
    return Status {
      .code = StatusCode::kInvalidArgument,
      .message = "Lockfile is invalid",
    };
  }

  if (int err = flock(lockfile_fd, LOCK_UN); err != 0) {
    return Status {
      .code = StatusCode::kInternal,
      .message = (std::stringstream() << "Unable to unlock with error: " <<
        intrinsic::StrError(errno).data()).str(),
    };
  }

  if (close(lockfile_fd) == -1) {
    // TODO(nilsb): proper logging
    std::cerr     << "WARN: Failed to close lockfile fd with error: "
                  << intrinsic::StrError(errno).data() << "." << std::endl;
  }
  return OkStatus();
}

}   // namespace

// Prepares a single message.
tl::expected<std::unique_ptr<const DomainSocketServer::Message>, Status>
DomainSocketServer::PrepareMessage(
  size_t message_index, size_t num_messages,
  const domain_socket_internal::ShmDescriptors & descriptors)
{
  if (message_index == 0) {
    return tl::unexpected(Status {
        .code = StatusCode::kOutOfRange,
        .message = "message_index starts at one",
      });
  }
  if (num_messages == 0) {
    return tl::unexpected(Status {
        .code = StatusCode::kOutOfRange,
        .message = "num_messages starts at one",
      });
  }
  if (message_index > num_messages) {
    return tl::unexpected(Status {
        .code = StatusCode::kOutOfRange,
        .message = (std::stringstream()
                      << "message_index (" << message_index <<
          ") cannot be greater than num_messages (" << num_messages << ")").str(),
      });
  }

  auto message = std::make_unique<Message>();
  message->descriptors = descriptors;

  message->descriptors.transfer_data.message_index = message_index;
  message->descriptors.transfer_data.num_messages = num_messages;
  if (protocol_version_ !=
    domain_socket_internal::kDomainSocketProtocolVersion)
  {
    // TODO(nilsb): proper logging
    std::cerr   << "WARN:" << "Overriding protocol version from "
                << domain_socket_internal::kDomainSocketProtocolVersion
                << " to " << protocol_version_
                << ". This should only happen in tests." << std::endl;

    message->descriptors.transfer_data.domain_socket_protocol_version =
      protocol_version_;
  }

  message->iov = {.iov_base = &message->descriptors.transfer_data,
    .iov_len = sizeof(message->descriptors.transfer_data)};

  // char has a size of 1 byte.
  message->cmsg_buf = std::vector<char>(
      CMSG_SPACE(message->descriptors.file_descriptors_in_order.size() *
                 sizeof(int)),
      0);
  message->msgh = {.msg_name = nullptr,
    .msg_namelen = 0,
    .msg_iov = &message->iov,
    .msg_iovlen = 1,
    .msg_control = message->cmsg_buf.data(),
    .msg_controllen = message->cmsg_buf.size(),
    .msg_flags = 0, };


  message->msgh.msg_control = message->cmsg_buf.data();
  message->msgh.msg_controllen = message->cmsg_buf.size();
  struct cmsghdr * cmsgp = CMSG_FIRSTHDR(&message->msgh);
  if (!cmsgp) {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = "Failed to get first control message header",
      });
  }
  cmsgp->cmsg_level = SOL_SOCKET;
  cmsgp->cmsg_type = SCM_RIGHTS;
  cmsgp->cmsg_len = CMSG_LEN(
      message->descriptors.file_descriptors_in_order.size() * sizeof(int));
  for (size_t i = 0; i < message->descriptors.file_descriptors_in_order.size();
    ++i)
  {
    reinterpret_cast<int *>(CMSG_DATA(cmsgp))[i] =
      message->descriptors.file_descriptors_in_order[i];
  }

  return message;
}

tl::expected<std::vector<std::unique_ptr<const DomainSocketServer::Message>>, Status>
DomainSocketServer::PrepareMessages(
  const SegmentNameToFileDescriptorMap & segment_name_to_file_descriptor_map)
{
  std::vector<std::unique_ptr<const DomainSocketServer::Message>> messages;

  if (segment_name_to_file_descriptor_map.empty()) {
    return tl::unexpected(Status {
        .code = StatusCode::kInvalidArgument,
        .message = "segment_name_to_file_descriptor_map is empty",
    });
  }
  // Computes the number of required messages starting at 1.
  // Example:
  // Assuming kMaxFdsPerMessage = 10
  // kNumMessages =  1 for [  1 -  10] fds.
  // kNumMessages =  2 for [ 11 -  20] fds.
  // ...
  // kNumMessages =  9 for [ 81 -  90] fds.
  // kNumMessages = 10 for [ 91 - 100] fds.
  // kNumMessages = 11 for [101 - 110] fds.
  const size_t kNumMessages =
    1 + ((segment_name_to_file_descriptor_map.size() - 1) /
    (domain_socket_internal::kMaxFdsPerMessage));
  size_t descriptor_count = 0;
  auto map_it = segment_name_to_file_descriptor_map.begin();
  // One indexed, so message_index == kNumMessages is OK.
  for (size_t message_index = 1; message_index <= kNumMessages;
    ++message_index)
  {
    // TODO(nilsb): proper logging
    std::cerr   << "INFO: " << "Preparing message " << message_index << " of "
                << kNumMessages << std::endl;
    // Fill descriptors for this message.
    domain_socket_internal::ShmDescriptors descriptors;
    auto & fd_names = descriptors.transfer_data.file_descriptor_names;
    for (fd_names.mutate_size(0);
         // Should not send more than kMaxFdsPerMessage FDs
      (fd_names.size() < domain_socket_internal::kMaxFdsPerMessage) &&
         // Cannot add more names than the fixed-size `names`
         // array supports
      (fd_names.size() < fd_names.names()->size()) &&
         // Cannot add more names than we got as inputs
      (map_it != segment_name_to_file_descriptor_map.end());
      ++map_it, fd_names.mutate_size(fd_names.size() + 1))
    {
      intrinsic_fbs::SegmentName segment_name;
      if (auto status = intrinsic_fbs::StringCopy(
              segment_name.mutable_value(), map_it->first); !status.ok())
      {
        return tl::unexpected(ToStatus(status));
      }

      descriptors.transfer_data.file_descriptor_names.mutable_names()->Mutate(
          fd_names.size(), segment_name);

      descriptors.file_descriptors_in_order.push_back(map_it->second);
    }
    descriptor_count += fd_names.size();
    auto message =
      PrepareMessage(
            /*message_index=*/message_index,
            /*num_messages=*/kNumMessages, descriptors);
    if (!message) {
      return tl::unexpected(message.error());
    }
    messages.push_back(std::move(*message));
  }
  if (descriptor_count != segment_name_to_file_descriptor_map.size()) {
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                      << "Expected " << segment_name_to_file_descriptor_map.size() <<
          " descriptors, but only got " << descriptor_count).str(),
      });
  }

  return messages;
}

Status DomainSocketServer::AddSegmentInfoServeShmDescriptors(
  SharedMemoryManager & shared_memory_manager)
{
  // SegmentInfo is expected by
  // intrinsic/icon/hal/hardware_module_proxy.h
  // It can in theory be replaced by the SegmentInfo shared in the data of the
  // domain socket message.
  if(auto status =
    shared_memory_manager.AddSegment<intrinsic_fbs::SegmentInfo>(
          /*name=*/hal::kModuleInfoName, /**must_be_used=*/false,
        shared_memory_manager.GetSegmentInfo(),
          /*type_id=*/hal::kModuleInfoName); !status.ok())
  {
    return status;
  }

  return ServeShmDescriptors(
      shared_memory_manager.SegmentNameToFileDescriptorMap());
}

Status DomainSocketServer::ServeShmDescriptors(
  const SegmentNameToFileDescriptorMap & segment_name_to_file_descriptor_map)
{
  if (request_handler_) {
    return Status {
      .code = StatusCode::kFailedPrecondition,
      .message = (std::stringstream() <<
        "Server is already serving descriptors on socket " <<
        absolute_socket_path_).str(),
    };
  }
  const size_t kNumDescriptors = segment_name_to_file_descriptor_map.size();

  // Determine the number of possible file descriptors.
  struct rlimit fd_limit;
  if (getrlimit(RLIMIT_NOFILE, &fd_limit) == -1) {
    return Status {
      .code = StatusCode::kFailedPrecondition,
      .message = (std::stringstream() <<
        "Failed to get limit of in-flight file descriptors "
        "(RLIMIT_NOFILE) with error: " <<
        intrinsic::StrError(errno).data()).str(),
    };
  }
  // rlim_cur is one greater than the maximum number of FDs.
  if (kNumDescriptors >= fd_limit.rlim_cur) {
    return Status {
      .code = StatusCode::kOutOfRange,
      .message = (std::stringstream() << "Number of file descriptors (" <<
        kNumDescriptors <<
        ") >= current soft limit (rlim_cur: " << fd_limit.rlim_cur <<
        "). Max limit (rlim_max) of RLIMIT_NOFILE is " << fd_limit.rlim_max).str(),
    };
  }

  // On Linux, at least one byte of “real data” is required to successfully
  // send ancillary data over a Unix domain stream socket.
  if (kNumDescriptors < 1) {
    Status {
      .code = StatusCode::kInvalidArgument,
      .message = "At least one file descriptor and name must be provided",
    };
  }

  {
    auto expected_messages =
      PrepareMessages(segment_name_to_file_descriptor_map);
    if (!expected_messages) {
      return expected_messages.error();
    }
    messages_ = std::move(*expected_messages);
  }
  if (listen(socket_fd_, /*backlog=*/1) == -1) {
    return Status {
      .code = StatusCode::kInternal,
      .message = (std::stringstream()
                 << "Failed to listen to socket with error: " << intrinsic::StrError(errno).data()
      ).str(),
    };
  }
  // TODO(nilsb): proper logging
  std::cerr   << "INFO: " << "Socket '" << absolute_socket_path_.native() << "' is serving  "
              << kNumDescriptors << " descriptors." << std::endl;

  {
    std::lock_guard l(handler_started_mtx_);
    handler_started_ = false;
  }
  // Accepts a connection, sends the prepared message and closes the connection.
  request_handler_ = std::make_unique<intrinsic::Thread>(
    [this](StopToken stop_token)->void {
      {
        std::lock_guard l(handler_started_mtx_);
        handler_started_ = true;
      }
      handler_started_cv_.notify_one();
      while (!stop_token.stop_requested()) {
          // TODO(nilsb): proper logging
        std::cerr << "INFO: " << "Waiting for new connection." << std::endl;

          // Accept all new connections.
          // A blocking socket is fine, because closing the socket unblocks.
        int to_client_sock = accept(socket_fd_, nullptr, nullptr);
        if (to_client_sock == -1) {
            // TODO(nilsb): proper logging
          std::cerr   << "ERROR: " << "Error while calling accept on '"
                      << absolute_socket_path_.native() << "': " << intrinsic::StrError(errno).data()
                      << ". Exiting. This is expected during shutdown." << std::endl;
          return;
        }
          // TODO(nilsb): proper logging
        std::cerr   << "INFO: " << "Accepted new connection on '"
                    << absolute_socket_path_.native() << "'." << std::endl;
          // Timeout stops rogue clients from blocking the server.
          // One second is more than enough time to send a message.
        struct timeval tv
        {
          .tv_sec = 1,
          .tv_usec = 0,
        };
        if (setsockopt(to_client_sock, SOL_SOCKET, SO_SNDTIMEO,
        (const char *)&tv, sizeof tv) == -1)
        {
            // TODO(nilsb): proper logging
          std::cerr   << "ERROR: " << "Failed to set socket timeout with error: "
                      << intrinsic::StrError(errno).data() << std::endl;
        }

        for (const auto & message : messages_) {
            // Only sending a single message without an explicit protocol.
          ssize_t bytes_sent = sendmsg(to_client_sock, &message->msgh, 0);
          if (bytes_sent == -1) {
              // TODO(nilsb): proper logging
            std::cerr   << "ERROR: " << "sendmsg failed with error: " << intrinsic::StrError(errno).data()
                        << "." << std::endl;
            continue;
          }

          if (size_t expected_bytes =
          sizeof(message->descriptors.transfer_data);
          bytes_sent != expected_bytes)
          {
              // TODO(nilsb): proper logging
            std::cerr   << "ERROR: "
                        << "Expected to send " << expected_bytes
                        << "bytes, but only sent " << bytes_sent << "bytes." << std::endl;
          }

            // TODO(nilsb): proper logging
          std::cerr   << "INFO: "
                      << "Sent "
                      << message->descriptors.file_descriptors_in_order.size()
                      << " file descriptors and " << bytes_sent
                      << " bytes of TransferData in message "
                      << message->descriptors.transfer_data.message_index
                      << " of " << messages_.size() << "." << std::endl;
        }

          // TODO(nilsb): proper logging
        std::cerr   << "INFO: "
                    << "Finished sending " << messages_.size()
                    << " messages." << std::endl;

          // Close the socket to the client.
        if (close(to_client_sock) == -1) {
            // TODO(nilsb): proper logging
          std::cerr   << "ERROR: "
                      << "Failed to close socket to client with error: "
                      << intrinsic::StrError(errno).data() << "." << std::endl;
          continue;
        }
      }
    });

  {
    std::unique_lock l(handler_started_mtx_);
    if (!handler_started_cv_.wait_for(l,
                                      std::chrono::seconds(10),
      [&]() {return handler_started_;}))
    {
      // Request the thread to stop, but don't join here. If it's stuck at the
      // very start of the thread body, we'd likely also get stuck here.
      (void) request_handler_->request_stop();
      return Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream() <<
          "Failed to start request handler thread for '" <<
          absolute_socket_path_ << "'").str(),
      };
    }
  }

  return OkStatus();
}

DomainSocketServer::~DomainSocketServer()
{
  // TODO(nilsb): proper logging
  std::cerr   << "INFO: "
              << "Shutting down DomainSocketServer on " << absolute_socket_path_.native() <<
    std::endl;
  if (socket_fd_ != -1) {
    if (shutdown(socket_fd_, SHUT_RDWR) == -1) {
      // TODO(nilsb): proper logging
      std::cerr   << "ERROR: "
                  << "Failed to shutdown socket with error: " << intrinsic::StrError(errno).data() << "." <<
        std::endl;
      std::exit(1);
    }
    // Closing the socket stops the server loop.
    if (close(socket_fd_) == -1) {
      // TODO(nilsb): proper logging
      std::cerr   << "WARN: "
                  << "Failed to close socket fd for '" << absolute_socket_path_.native()
                  << "'. with error: " << intrinsic::StrError(errno).data() << "." << std::endl;
    }
  }
  if (!absolute_socket_path_.empty()) {
    if (unlink(absolute_socket_path_.c_str()) == -1) {
      // TODO(nilsb): proper logging
      std::cerr   << "WARN: "
                  << "Failed to unlink socket file '" << absolute_socket_path_.native()
                  << "'. with error: " << intrinsic::StrError(errno).data() << "." << std::endl;
    }
  }

  // TODO: It might be important to shutdown the thread first, before closing
  // the file.
  request_handler_.reset();

  if (const auto & status = UnlockPathCloseLockfile(flock_fd_); !status.ok()) {
    // TODO(nilsb): proper logging
    std::cerr   << "ERROR: "
                << "Failed to unlock lock '" << absolute_lock_path_.native()
                << "' for socket' " << absolute_socket_path_.native()
                << "': " << status << std::endl;
  }
}

// static
tl::expected<std::unique_ptr<DomainSocketServer>, Status> DomainSocketServer::Create(
  std::filesystem::path socket_directory, std::string_view module_name,
  std::chrono::seconds lock_acquire_timeout, size_t protocol_version)
{
  if (auto status =
    domain_socket_internal::CreateSocketDirectory(socket_directory); !status.ok())
  {
    return tl::unexpected(status);
  }

  std::filesystem::path absolute_lock_path = LockName(socket_directory, module_name);
  auto flock_fd = TryLockPath(absolute_lock_path, lock_acquire_timeout);
  if (!flock_fd) {
    return tl::unexpected(flock_fd.error());
  }

  Cleanup clear_flock(
    [flock_fd = *flock_fd]() {
      if (const auto & status = UnlockPathCloseLockfile(flock_fd); !status.ok()) {
      // TODO(nilsb): proper logging
        std::cerr << "ERROR: "
                  << "Failed to unlock flock with error: " << status << std::endl;
      }
    });

  auto absolute_socket_path =
    domain_socket_internal::AbsoluteSocketPath(
          socket_directory, module_name);
  if(!absolute_socket_path) {
    return tl::unexpected(absolute_socket_path.error());
  }

  // A preexisting socket file needs to be unlinked before binding. This also
  // closes all connections of hypothetical clients.
  // https://gavv.net/articles/unix-socket-reuse/
  // Since we hold the lock, we know that nobody else is using this socket.
  if (std::filesystem::exists(*absolute_socket_path)) {
    // TODO(nilsb): proper logging
    std::cerr   << "INFO: "
                << "Socket file '" << absolute_socket_path->native()
                << "' already exists. Unlinking." << std::endl;
    if (unlink(absolute_socket_path->c_str()) == -1) {
      return tl::unexpected(Status {
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                        << "Failed to unlink socket file '" << absolute_socket_path->native()
                        << "'. with error: " << intrinsic::StrError(errno).data()).str(),
        });
    }
  }

  // Uses SOCK_STREAM (TCP) for connections, because it already supports
  // sessions. Otherwise we'd need to implement a simple handshake protocol.
  int socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);

  auto addr =
    domain_socket_internal::AddressFromAbsolutePath(*absolute_socket_path);
  if (!addr) {
    return tl::unexpected(addr.error());
  }

  if (bind(socket_fd, (sockaddr *)&(addr.value()), sizeof(sockaddr_un)) == -1) {
    int savedErrno = errno;
    // TODO(nilsb): proper logging
    std::cerr   << "ERROR: "
                << "Failed to bind to socket with error: "
                << intrinsic::StrError(savedErrno).data()
                << ". Cleaning up, then returning error." << std::endl;
    if (close(socket_fd) == -1) {
    // TODO(nilsb): proper logging
      std::cerr   << "WARN: "
                  << "Failed to close socket fd for '" << absolute_socket_path->native()
                  << "'. with error: " << intrinsic::StrError(errno).data() << "." << std::endl;
    }
    return tl::unexpected(Status {
        .code = StatusCode::kInternal,
        .message = (std::stringstream()
                      << "Failed to bind socket file '" << absolute_socket_path->native()
                      << "'. with error: " << intrinsic::StrError(savedErrno).data()).str(),
      });
  }
  std::move(clear_flock).Cancel();

  return std::unique_ptr<DomainSocketServer>(new DomainSocketServer(
      *absolute_socket_path, absolute_lock_path,
      /*socket_fd=*/socket_fd,
      /*flock_fd=*/*flock_fd,
      /*protocol_version=*/protocol_version));
}

}  // namespace intrinsic::hal
