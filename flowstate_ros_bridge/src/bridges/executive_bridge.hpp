// Copyright 2024 Intrinsic Innovation LLC
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

#ifndef BRIDGES__EXECUTIVE_BRIDGE_HPP_
#define BRIDGES__EXECUTIVE_BRIDGE_HPP_

#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "flowstate_interfaces/action/start_process.hpp"
#include "flowstate_interfaces/srv/list_behavior_trees.hpp"
#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace flowstate_ros_bridge {
///=============================================================================
// The Node will retrieve information required for configuration from
// ROS Parameters.
// TODO(Yadunund): Define a standalone executable which overrides parameters
// based on a config proto.
class ExecutiveBridge : public BridgeInterface {
 public:
  using ListBehaviorTrees = flowstate_interfaces::srv::ListBehaviorTrees;
  using StartProcess = flowstate_interfaces::action::StartProcess;
  using StartProcessGoalHandle = rclcpp_action::ServerGoalHandle<StartProcess>;

  /// Documentation inherited.
  void declare_ros_parameters(ROSNodeInterfaces ros_node_interfaces) final;

  /// Documentation inherited.
  bool initialize(ROSNodeInterfaces ros_node_interfaces,
                  std::shared_ptr<Executive> executive_client,
                  std::shared_ptr<World> world_client) final;

 private:
  struct Data : public std::enable_shared_from_this<Data> {
    ROSNodeInterfaces node_interfaces_;
    std::thread task_thread_;
    std::mutex mutex_;
    std::shared_ptr<Executive> executive_;
    std::shared_ptr<rclcpp::Service<ListBehaviorTrees>> list_bts_srv_;
    std::shared_ptr<rclcpp_action::Server<StartProcess>> start_process_srv_;
    Executive::ProcessHandlePtr process_handle_;
    ~Data();
  };
  std::shared_ptr<Data> data_;
};
}  // namespace flowstate_ros_bridge.

#endif  // BRIDGES__EXECUTIVE_BRIDGE_HPP_
