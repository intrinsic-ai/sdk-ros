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

#ifndef LOCAL_FLOWSTATE_ROS_BRIDGE_HPP_
#define LOCAL_FLOWSTATE_ROS_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "flowstate_ros_bridge/executive.hpp"
#include "flowstate_ros_bridge/world.hpp"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace local_flowstate_ros_bridge {

class LocalFlowstateROSBridge : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Constructor.
  explicit LocalFlowstateROSBridge(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /// Lifecycle methods.
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

private:
  pluginlib::ClassLoader<flowstate_ros_bridge::BridgeInterface> bridge_loader_;
  std::vector<pluginlib::UniquePtr<flowstate_ros_bridge::BridgeInterface>>
      bridges_;
  std::shared_ptr<flowstate_ros_bridge::Executive> executive_;
  std::shared_ptr<flowstate_ros_bridge::World> world_;
  std::shared_ptr<intrinsic::PubSub> pubsub_;
  std::vector<std::string> bridge_ids_;
};
} // namespace local_flowstate_ros_bridge

#endif // FLOWSTATE_ROS_BRIDGE_HPP_
