// Copyright 2025 Intrinsic Innovation LLC
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

#ifndef FLOWSTATE_ROS_BRIDGE__BRIDGE_INTERFACE_HPP_
#define FLOWSTATE_ROS_BRIDGE__BRIDGE_INTERFACE_HPP_

#include <memory>

#include "executive.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"
#include "world.hpp"
#include "diagnostics.hpp"

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
  /// @param world_client A client to the Flowstate World service.
  /// @param diagnostics_client A client to the Flowstate Diagnostics service.
  virtual bool initialize(ROSNodeInterfaces ros_node_interfaces,
                          std::shared_ptr<Executive> executive_client,
                          std::shared_ptr<World> world_client,
                          std::shared_ptr<Diagnostics> diagnostics_client) = 0;

  /// @brief Virtual destructor.
  virtual ~BridgeInterface() = default;
};

}  // namespace flowstate_ros_bridge.

#endif  // FLOWSTATE_ROS_BRIDGE__BRIDGE_INTERFACE_HPP_
