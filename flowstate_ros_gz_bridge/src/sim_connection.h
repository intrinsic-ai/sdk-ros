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

#ifndef FLOWSTATE_ROS_GZ_BRIDGE__SRC__SIM_CONNECTION_H_
#define FLOWSTATE_ROS_GZ_BRIDGE__SRC__SIM_CONNECTION_H_

#include <memory>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "gz/transport/Node.hh"

namespace intrinsic {
namespace simulation {

// SimConnection manages a connection to Gazebo simulation.
// The connection utilizes a gz-tranport node to communicate with
// the Gazebo server over pub/sub.
class SimConnection {
 public:
  // Create a connection to simulation at the specified simulation
  // server address. If simulation connection creation is unsuccessful,
  // the following status errors are returned:
  //   * kUnavailable: The simulation server is unavailable.
  //   * kInternal: Error resolving sim_server_address.
  static absl::StatusOr<std::shared_ptr<SimConnection>> Create(
      std::string_view sim_server_address);

  // Get a shared_ptr to the transport node that manages pub/sub connections
  // to simulation.
  std::shared_ptr<gz::transport::Node> Node();

 private:
  explicit SimConnection(gz::transport::NodeOptions node_options);
  // TODO(luca) changed from upstream where it was not wrapped in a shared_ptr
  // ros_gz_bridge required it to be a shared_ptr
  std::shared_ptr<gz::transport::Node> node_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // FLOWSTATE_GZ_RMF_BRIDGE__SRC__SIM_CONNECTION_H_
