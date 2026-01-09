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

#include "executive_bridge.hpp"

#include <string>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "flowstate_interfaces/msg/behavior_tree.hpp"
#include "flowstate_interfaces/msg/execution_mode.hpp"
#include "flowstate_interfaces/msg/simulation_mode.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp_action/create_server.hpp"

namespace flowstate_ros_bridge {
constexpr const char* kListBTParamName = "list_behavior_trees_srv_name";
constexpr const char* kStartProcessParamName = "start_process_action_name";

///=============================================================================
void ExecutiveBridge::declare_ros_parameters(
    ROSNodeInterfaces ros_node_interfaces) {
  const auto& param_interface =
      ros_node_interfaces
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  param_interface->declare_parameter(
      kListBTParamName, rclcpp::ParameterValue{"list_behavior_trees"});

  param_interface->declare_parameter(
      kStartProcessParamName,
      rclcpp::ParameterValue{"start_flowstate_process"});
}

///=============================================================================
bool ExecutiveBridge::initialize(ROSNodeInterfaces ros_node_interfaces, std::shared_ptr<Executive> executive_client,
                                 std::shared_ptr<World> /*world_client*/)
{
  data_ = std::make_shared<Data>();
  data_->node_interfaces_ = std::move(ros_node_interfaces);
  data_->executive_ = std::move(executive_client);

  const auto& param_interface =
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  data_->list_bts_srv_ = rclcpp::create_service<ListBehaviorTrees>(
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>(),
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeServicesInterface>(),
      param_interface->get_parameter(kListBTParamName).get_value<std::string>(),
      [data_ = this->data_](
          const std::shared_ptr<ListBehaviorTrees::Request> /*request*/,
          std::shared_ptr<ListBehaviorTrees::Response> response) {
        response->success = false;
        RCLCPP_INFO(data_->node_interfaces_
                        .get<rclcpp::node_interfaces::NodeLoggingInterface>()
                        ->get_logger(),
                    "Getting behavior trees..");
        auto result = data_->executive_->behavior_trees();
        if (result.ok()) {
          for (const auto& bt : result.value()) {
            flowstate_interfaces::msg::BehaviorTree bt_msg;
            bt_msg.name = bt.name();
            bt_msg.tree_id = bt.tree_id();
            response->behavior_trees.push_back(std::move(bt_msg));
          }
          response->success = true;
        }
      },
      rclcpp::ServicesQoS(), nullptr);

  data_->start_process_srv_ = rclcpp_action::create_server<StartProcess>(
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>(),
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeClockInterface>(),
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeLoggingInterface>(),
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeWaitablesInterface>(),
      param_interface->get_parameter(kStartProcessParamName)
          .get_value<std::string>(),
      [data_ = this->data_](const rclcpp_action::GoalUUID& uuid,
                            std::shared_ptr<const StartProcess::Goal> goal) {
        // Handle goal.
        RCLCPP_INFO(data_->node_interfaces_
                        .get<rclcpp::node_interfaces::NodeLoggingInterface>()
                        ->get_logger(),
                    "Received request to start process %s",
                    goal->behavior_tree.name.c_str());
        (void)uuid;
        data_->process_handle_.reset();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [data_ = this->data_](
          const std::shared_ptr<StartProcessGoalHandle> goal_handle) {
        // Handle cancel.
        RCLCPP_INFO(data_->node_interfaces_
                        .get<rclcpp::node_interfaces::NodeLoggingInterface>()
                        ->get_logger(),
                    "Received request to cancel goal");
        (void)goal_handle;
        if (data_->process_handle_ != nullptr) {
          data_->process_handle_->cancel();
        }
        auto result = std::make_shared<StartProcess::Result>();
        result->success = false;
        result->message = "Cancelled by user.";
        goal_handle->canceled(std::move(result));
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [data_ = this->data_](
          const std::shared_ptr<StartProcessGoalHandle> goal_handle) {
        // Handle accept.
        const auto goal = goal_handle->get_goal();
        auto result =
            data_->executive_->behavior_tree(goal->behavior_tree.name);
        if (result.ok()) {
          intrinsic_proto::executive::ExecutionMode execution_mode;
          switch (goal->execution_mode.mode) {
            // Default to Normal if unspecified.
            case flowstate_interfaces::msg::ExecutionMode::
                EXECUTION_MODE_UNSPECIFIED:
            case flowstate_interfaces::msg::ExecutionMode::
                EXECUTION_MODE_NORMAL:
              execution_mode = intrinsic_proto::executive::ExecutionMode::
                  EXECUTION_MODE_NORMAL;
              break;
            case flowstate_interfaces::msg::ExecutionMode::
                EXECUTION_MODE_STEP_WISE:
              execution_mode = intrinsic_proto::executive::ExecutionMode::
                  EXECUTION_MODE_STEP_WISE;
              break;
            default:
              auto result = std::make_shared<StartProcess::Result>();
              result->success = false;
              result->message = "Invalid ExecutionMode requested.";
              goal_handle->abort(std::move(result));
              return;
          }
          intrinsic_proto::executive::SimulationMode simulation_mode;
          switch (goal->simulation_mode.mode) {
            // Default to Normal if unspecified.
            case flowstate_interfaces::msg::SimulationMode::
                SIMULATION_MODE_UNSPECIFIED:
            case flowstate_interfaces::msg::SimulationMode::
                SIMULATION_MODE_REALITY:
              simulation_mode = intrinsic_proto::executive::SimulationMode::
                  SIMULATION_MODE_REALITY;
              break;
            case flowstate_interfaces::msg::SimulationMode::
                SIMULATION_MODE_DRAFT:
              simulation_mode = intrinsic_proto::executive::SimulationMode::
                  SIMULATION_MODE_DRAFT;
              break;
            default:
              auto result = std::make_shared<StartProcess::Result>();
              result->success = false;
              result->message = "Invalid SimulationMode requested.";
              goal_handle->abort(std::move(result));
              return;
          }
          // Get process parameters.
          nlohmann::json process_params;
          try {
            if (!goal->parameters.empty()) {
              process_params = nlohmann::json::parse(goal->parameters);
            }
          } catch (const nlohmann::json::exception& e) {
            auto result = std::make_shared<StartProcess::Result>();
            result->success = false;
            result->message =
                "Failed to parse parameters in the goal into a JSON object.";
            goal_handle->abort(std::move(result));
            return;
          }
          // Set scene_id if provided.
          std::optional<std::string> scene_id_opt = std::nullopt;
          if (!goal->scene_id.empty()) {
            scene_id_opt = goal->scene_id;
          }
          // Start the process.
          auto start_result = data_->executive_->start(
              result.value(), execution_mode, simulation_mode,
              std::move(process_params),
              [goal_handle](bool done, const absl::Status& status) -> void {
                auto fb = std::make_shared<StartProcess::Feedback>();
                fb->done = done;
                if (!status.ok()) {
                  fb->error_message = status.ToString();
                }
                goal_handle->publish_feedback(std::move(fb));
              },
              [goal_handle](const bool success,
                            const std::string& message) -> void {
                auto result = std::make_shared<StartProcess::Result>();
                result->success = success;
                result->message = message;
                if (result->success) {
                  goal_handle->succeed(std::move(result));
                } else {
                  goal_handle->abort(std::move(result));
                }
              },
              std::move(scene_id_opt));
          if (start_result.ok()) {
            LOG(INFO) << "Process started successfully.";
            data_->process_handle_ = start_result.value();
          } else {
            LOG(ERROR) << "Failed to run process.";
            auto result = std::make_shared<StartProcess::Result>();
            result->success = false;
            result->message = start_result.status().ToString();
            goal_handle->abort(std::move(result));
          }
        }
      });

  data_->process_handle_ = nullptr;
  return true;
}

///=============================================================================
ExecutiveBridge::Data::~Data() {
  if (task_thread_.joinable()) {
    task_thread_.join();
  }
}
}  // namespace flowstate_ros_bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::ExecutiveBridge,
                       flowstate_ros_bridge::BridgeInterface)
