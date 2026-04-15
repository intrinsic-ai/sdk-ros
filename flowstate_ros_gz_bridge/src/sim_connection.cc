// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim_connection.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "gz/transport/Node.hh"
#include "gz/transport/NodeOptions.hh"
#include "gz/transport/TopicUtils.hh"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace simulation {

namespace {

inline constexpr char kGzNodePartition[] = "intrinsic_sim";

absl::StatusOr<gz::transport::NodeOptions> DefaultNodeOptions() {
  gz::transport::NodeOptions node_opts;
  node_opts.SetPartition(kGzNodePartition);

  // Sanity check to make sure the values are valid
  if (!gz::transport::TopicUtils::IsValidNamespace(node_opts.NameSpace()) ||
      !gz::transport::TopicUtils::IsValidPartition(node_opts.Partition())) {
    return absl::InternalError(
        "Node option contains invalid namespace or partition");
  }

  return node_opts;
}

absl::StatusOr<std::string> ResolveSimAddress(
    std::string_view sim_server_address) {
  // Copy sim_server_address so we can make a null-terminated string to pass
  // to C APIs.
  const std::string hostname(sim_server_address);

  struct addrinfo hints;
  memset(&hints, 0, sizeof(struct addrinfo));
  hints.ai_family = AF_INET;

  struct addrinfo* res;
  // TODO(luca) it seems the second parameter here is the port
  if (getaddrinfo(hostname.c_str(), "", &hints, &res) != 0) {
    return absl::UnavailableError(absl::StrFormat(
        "getAddrinfo('%s') failed: %s", hostname, strerror(errno)));
  }
  absl::Cleanup addrinfo_closer = [res] { freeaddrinfo(res); };

  std::array<char, INET6_ADDRSTRLEN> buffer;
  void* src = nullptr;
  switch (res->ai_family) {
    case AF_INET:
      src = &(reinterpret_cast<struct sockaddr_in*>(res->ai_addr)->sin_addr);
      break;
    case AF_INET6:
      src = &(reinterpret_cast<struct sockaddr_in6*>(res->ai_addr)->sin6_addr);
      break;
    default:
      return absl::InternalError(
          absl::StrFormat("Unhandled address family %i", res->ai_family));
  }
  const char* ip_str =
      inet_ntop(res->ai_family, src, buffer.data(), buffer.size());
  if (ip_str == nullptr) {
    return absl::InternalError(
        absl::StrFormat("inet_ntop failed: %s", strerror(errno)));
  }

  return ip_str;
}
}  // namespace

absl::StatusOr<std::shared_ptr<SimConnection>> SimConnection::Create(
    std::string_view sim_server_address) {
  if (sim_server_address.empty()) {
    return absl::InvalidArgumentError("Sim server address cannot be empty.");
  }
  INTR_ASSIGN_OR_RETURN(std::string sim_ip,
                        ResolveSimAddress(sim_server_address));

  INTR_ASSIGN_OR_RETURN(gz::transport::NodeOptions node_opts,
                        DefaultNodeOptions());

  auto sim_connection =
      std::shared_ptr<SimConnection>(new SimConnection(std::move(node_opts)));
  // By default, gz-transport does not support communication of nodes on
  // different local networks, e.g. when the sim connection client and the
  // sim server have different subnets. We need to explicitly tell the
  // gz-tranport node to relay traffic to the specified sim server ip.
  sim_connection->node_->AddGlobalRelay(sim_ip);
  return sim_connection;
}

SimConnection::SimConnection(gz::transport::NodeOptions node_options)
    : node_(std::make_shared<gz::transport::Node>(std::move(node_options))) {}

std::shared_ptr<gz::transport::Node> SimConnection::Node() { return node_; }

}  // namespace simulation
}  // namespace intrinsic
