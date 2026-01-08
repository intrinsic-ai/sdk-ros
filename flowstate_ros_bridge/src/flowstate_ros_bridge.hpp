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

#ifndef FLOWSTATE_ROS_BRIDGE_HPP_
#define FLOWSTATE_ROS_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "flowstate_ros_bridge/executive.hpp"
#include "flowstate_ros_bridge/world.hpp"
#include "flowstate_ros_bridge/diagnostics.hpp"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/zenoh_publisher_data.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace flowstate_ros_bridge {
///=============================================================================
class FlowstateROSBridge : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Constructor.
  explicit FlowstateROSBridge(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// Lifecycle methods.
  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  pluginlib::ClassLoader<BridgeInterface> bridge_loader_;
  std::vector<pluginlib::UniquePtr<BridgeInterface>> bridges_;
  std::shared_ptr<Executive> executive_;
  std::shared_ptr<World> world_;
  std::shared_ptr<Diagnostics> diagnostics_;
  std::shared_ptr<intrinsic::PubSub> pubsub_;
  std::vector<std::string> bridge_ids_;
};
}  // namespace flowstate_ros_bridge.

#endif  // FLOWSTATE_ROS_BRIDGE_HPP_
