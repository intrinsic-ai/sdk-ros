#pragma once

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <chrono>

#include <tl/expected.hpp>
#include "status.hpp"
#include "flatbuffer_utils.hpp"

//#include "intrinsic/icon/flatbuffers/flatbuffer_utils.h"
//#include "intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "hwm_fbs/segment_info.fbs.h"

namespace intrinsic::hal {

namespace domain_socket_internal {

using ::intrinsic_fbs::FlatbufferArrayNumElements;

// The maximum number of file descriptors that can be shared in a single
// message. The FileDescriptorNames flatbuffer definition
// (intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs)
// dictates this constant.
// The absolute maximum is defined by the kernel constant SCM_MAX_FD, which is
// 253, or 255 in kernels before 2.6.38.
inline constexpr size_t kMaxFdsPerMessage =
    FlatbufferArrayNumElements(&intrinsic_fbs::FileDescriptorNames::names);

// LINT.IfChange(expected_version)
inline constexpr size_t kDomainSocketProtocolVersion = 2;
// LINT.ThenChange()

inline constexpr std::string_view kSocketSuffix = ".sock";

struct TransferData {
  size_t domain_socket_protocol_version =
      domain_socket_internal::kDomainSocketProtocolVersion;
  // The server may have to break up the list of file descriptors into multiple
  // messages. The server reports the total number of messages in this member on
  // every message it sends, so that clients know how many messages to expect.
  size_t num_messages = 0;
  // One-based Index of the current message.
  // A DomainSocket client uses this to make sure it has received all
  // (num_messages) messages the server is going to send.
  // Note that the server will always transmit messages in order. That is, the
  // client can stop receiving when num_messages == message_index
  size_t message_index = 0;
  intrinsic_fbs::FileDescriptorNames file_descriptor_names;
};

struct ShmDescriptors {
  TransferData transfer_data;
  std::vector<int> file_descriptors_in_order;
};

// Recursively creates the directory on `absolute_path` if it doesn't exist.
// NoOp if the directory already exists.
// Returns InvalidArgumentError if `absolute_path` is not absolute.
// Forwards errors of creating the respective directories.
Status CreateSocketDirectory(std::string_view absolute_path);

// Returns the absolute path of the socket file for the given `absolute_path`
// and `module_name`.
// `absolute_path` is the base directory for the socket file.
// InvalidArgumentError if the generated path is too long.
tl::expected<std::string, Status> AbsoluteSocketPath(std::string_view absolute_path,
                                                      std::string_view module_name);

// Writes `absolute_socket_path` into a valid sockaddr_un.
// Returns InvalidArgumentError if `absolute_socket_path` is too long.
tl::expected<sockaddr_un, Status> AddressFromAbsolutePath(
    std::string_view absolute_socket_path);

}  // namespace domain_socket_internal

// Returns the socket directory for the given `shared_memory_namespace`.
// Returns the default socket directory when `shared_memory_namespace` is empty.
std::string SocketDirectoryFromNamespace(
    std::string_view shared_memory_namespace);

using SegmentNameToFileDescriptorMap = std::unordered_map<std::string, int>;

// Receives a map of segment names to valid file descriptors from a
// DomainSocketServer.
// The received file descriptors are copied to the file descriptor table of the
// receiving process. They remain open when the server process closes theirs and
// vice versa.
//
// Generates socket location from `socket_directory` and `module_name`.
// Creates the socket directory if it doesn't exist.
// Tries once when `connection_timeout` is absl::ZeroDuration.
// Returns DeadlineExceededError if no server is available within
// `connection_timeout`, or the server doesn't start sending data before
// `connection_timeout`.
// Transmitting the file descriptors can take longer than
// `connection_timeout`.
tl::expected<SegmentNameToFileDescriptorMap, Status>
GetSegmentNameToFileDescriptorMap(std::string_view socket_directory,
                                  std::string_view module_name,
                                  std::chrono::seconds connection_timeout);

}  // namespace intrinsic::icon
