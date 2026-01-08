// Copyright 2024 Intrinsic Innovation LLC
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
                  std::shared_ptr<World> world_client,
                  std::shared_ptr<Diagnostics> diagnostics_client) final;

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
