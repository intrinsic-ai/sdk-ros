// Copyright 2025 Intrinsic Innovation LLC
//
// You are hereby granted a non-exclusive, worldwide, royalty-free license to
// use, copy, modify, and distribute this Intrinsic SDK in source code or binary
// form for use in connection with the services and APIs provided by Intrinsic
// Innovation LLC (“Intrinsic”).
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// If you use this Intrinsic SDK with any Intrinsic services, your use is
// subject to the Intrinsi Platform Terms of Service
// [https://intrinsic.ai/platform-terms].  If you create works that call
// Intrinsic APIs, you must agree to the terms of service for those APIs
// separately. This license does not grant you any special rights to use the
// services.
//
// This copyright notice shall be included in all copies or substantial portions
// of the software.

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
