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

#ifndef FLOWSTATE_ROS_BRIDGE__BRIDGE_INTERFACE_HPP_
#define FLOWSTATE_ROS_BRIDGE__BRIDGE_INTERFACE_HPP_

#include <memory>

#include "executive.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"
#include "world.hpp"

namespace flowstate_ros_bridge {

///=============================================================================
/// @brief  We rely on NodeInterfaces to create a abstract away the type of ROS
/// node
///   (eg. LifecycleNode vs regular) and avoid circular references with the node
///   that will initialize implementations of BridgeInterface.
using ROSNodeInterfaces =
    rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>;

///=============================================================================
class BridgeInterface {
 public:
  /// @brief A static method that will be called exactly only over the lifetime
  ///   of the implementation to declare any ROS parameters.
  virtual void declare_ros_parameters(
      ROSNodeInterfaces ros_node_interfaces) = 0;

  /// @brief Initialize the Bridge.
  /// @param ros_node_interfaces ROS node interfaces.
  /// @param executive_client A client to the Flowstate Executive service.
  virtual bool initialize(ROSNodeInterfaces ros_node_interfaces,
                          std::shared_ptr<Executive> executive_client,
                          std::shared_ptr<World> world_client) = 0;

  /// @brief Virtual destructor.
  virtual ~BridgeInterface() = default;
};

}  // namespace flowstate_ros_bridge.

#endif  // FLOWSTATE_ROS_BRIDGE__BRIDGE_INTERFACE_HPP_
