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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "aquarium_gateway/aquarium_gateway.hpp"
#include "aquarium_gateway/serialization.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace {

using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
using GoalHandleParallelGripper =
    rclcpp_action::ServerGoalHandle<ParallelGripperCommand>;

class ActionGatewayTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override { aquarium_gateway::ShutdownGateway(); }

  static void TearDownTestSuite() {
    aquarium_gateway::ShutdownGateway();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ActionGatewayTest, DynamicallyTranslateParallelGripperCommandAction) {
  // 1. CREATE DUMMY GRIPPER ACTION SERVER
  auto server_node =
      rclcpp::Node::make_shared("dummy_gripper_action_server_node");

  std::atomic<bool> server_received_goal{false};
  std::vector<std::string> received_joint_names;
  std::vector<double> received_positions;
  std::mutex goal_mutex;

  auto handle_goal =
      [](const rclcpp_action::GoalUUID& uuid,
         std::shared_ptr<const ParallelGripperCommand::Goal> goal) {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

  auto handle_cancel =
      [](const std::shared_ptr<GoalHandleParallelGripper> goal_handle) {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      };

  auto handle_accepted =
      [&server_received_goal, &received_joint_names, &received_positions,
       &goal_mutex](
          const std::shared_ptr<GoalHandleParallelGripper> goal_handle) {
        const auto goal = goal_handle->get_goal();
        {
          std::lock_guard<std::mutex> lock(goal_mutex);
          received_joint_names = goal->command.name;
          received_positions = goal->command.position;
          server_received_goal = true;
        }

        // Simulates gripper action execution for a bit of time (50ms)
        // and succeeds with ParallelGripperCommand::Result
        std::thread([goal_handle]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          auto result = std::make_shared<ParallelGripperCommand::Result>();
          result->state.header.frame_id = "gripper_base_link";
          result->state.name = {"left_finger_joint", "right_finger_joint"};
          result->state.position = {0.035, 0.035};
          result->state.velocity = {0.0, 0.0};
          result->state.effort = {12.5, 12.5};
          result->stalled = false;
          result->reached_goal = true;
          goal_handle->succeed(result);
        }).detach();
      };

  auto action_server = rclcpp_action::create_server<ParallelGripperCommand>(
      server_node, "/test_parallel_gripper_action", handle_goal, handle_cancel,
      handle_accepted);

  // 2. INITIALIZE GATEWAY NODE AND MULTI-THREADED EXECUTOR
  aquarium_gateway::g_node =
      rclcpp::Node::make_shared("test_aquarium_gateway_action_node");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(aquarium_gateway::g_node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  // 3. CREATE TEMPORARY YAML CONFIG PROXYING ParallelGripperCommand
  std::string temp_yaml_path =
      (std::filesystem::temp_directory_path() / "test_gripper_aquarium.yaml")
          .string();
  {
    std::ofstream yaml_out(temp_yaml_path);
    yaml_out << R"(
target:
  type: "node"
  package: "control_msgs"
  name: "dummy_gripper_action_server_node"
types:
  - name: "sensor_msgs/msg/JointState"
    fields:
      - "std_msgs/msg/Header header"
      - "string[] name"
      - "double[] position"
      - "double[] velocity"
      - "double[] effort"
  - name: "std_msgs/msg/Header"
    fields:
      - "builtin_interfaces/msg/Time stamp"
      - "string frame_id"
  - name: "builtin_interfaces/msg/Time"
    fields:
      - "int32 sec"
      - "uint32 nanosec"
interfaces:
  - internal_interface: "ros_action_client"
    internal_name: "/test_parallel_gripper_action"
    internal_type: "control_msgs/action/ParallelGripperCommand"
    external_name: "parallel_gripper_command"
    type_hash: "RIHS01_388b0a9cb34b22db9c6d67b752df85e173f40d6ad1d2a13824f114620f4d3fa1"
    request_fields:
      - "sensor_msgs/msg/JointState command"
    response_fields:
      - "sensor_msgs/msg/JointState state"
      - "bool stalled"
      - "bool reached_goal"
)";
  }

  // 4. LOAD CONFIG AND COMPILE DYNAMIC PROTOBUF SCHEMAS IN MEMORY
  google::protobuf::DescriptorPool pool;
  google::protobuf::DynamicMessageFactory factory(&pool);
  std::map<std::string, aquarium_gateway::ServiceConfig> config_cache;

  bool load_ok =
      aquarium_gateway::load_gateway_config(temp_yaml_path, pool, config_cache);
  ASSERT_TRUE(load_ok);

  const auto* goal_desc = pool.FindMessageTypeByName(
      "control_msgs.action.ParallelGripperCommand_Goal");
  ASSERT_NE(goal_desc, nullptr);
  const auto* res_desc = pool.FindMessageTypeByName(
      "control_msgs.action.ParallelGripperCommand_Result");
  ASSERT_NE(res_desc, nullptr);
  const auto* time_desc = pool.FindMessageTypeByName(
      "control_msgs.action.builtin_interfaces_msg_Time");
  ASSERT_NE(time_desc, nullptr);
  const auto* sec_field = time_desc->FindFieldByName("sec");
  ASSERT_NE(sec_field, nullptr);
  EXPECT_EQ(sec_field->type(), google::protobuf::FieldDescriptor::TYPE_INT32);
  const auto* nanosec_field = time_desc->FindFieldByName("nanosec");
  ASSERT_NE(nanosec_field, nullptr);
  EXPECT_EQ(nanosec_field->type(),
            google::protobuf::FieldDescriptor::TYPE_UINT32);

  // 5. BUILD DYNAMIC PROTOBUF GOAL REQUEST (Intrinsic Skill Input)
  std::unique_ptr<google::protobuf::Message> dynamic_request(
      factory.GetPrototype(goal_desc)->New());
  const auto* req_reflection = dynamic_request->GetReflection();
  const auto* cmd_field = goal_desc->FindFieldByName("command");
  ASSERT_NE(cmd_field, nullptr);
  auto* cmd_msg =
      req_reflection->MutableMessage(dynamic_request.get(), cmd_field);
  const auto* cmd_desc = cmd_msg->GetDescriptor();
  const auto* cmd_reflection = cmd_msg->GetReflection();

  const auto* name_field = cmd_desc->FindFieldByName("name");
  ASSERT_NE(name_field, nullptr);
  cmd_reflection->AddString(cmd_msg, name_field, "left_finger_joint");
  cmd_reflection->AddString(cmd_msg, name_field, "right_finger_joint");

  const auto* pos_field = cmd_desc->FindFieldByName("position");
  ASSERT_NE(pos_field, nullptr);
  cmd_reflection->AddDouble(cmd_msg, pos_field, 0.035);
  cmd_reflection->AddDouble(cmd_msg, pos_field, 0.035);

  std::unique_ptr<google::protobuf::Message> dynamic_response(
      factory.GetPrototype(res_desc)->New());

  // 6. DISPATCH THROUGH DYNAMIC ACTION CLIENT GATEWAY
  std::string error_msg;
  const auto& srv_config = config_cache["parallel_gripper_command"];
  bool dispatch_ok = aquarium_gateway::DynamicActionTransactionDispatcher(
      srv_config.internal_name, srv_config.internal_type, srv_config.type_desc,
      *dynamic_request, *dynamic_response, error_msg);

  EXPECT_TRUE(dispatch_ok) << "Dispatcher failed with error: " << error_msg;

  // 7. VERIFY SERVER RECEIVED THE GOAL AS COMMANDED
  EXPECT_TRUE(server_received_goal);
  {
    std::lock_guard<std::mutex> lock(goal_mutex);
    ASSERT_EQ(received_joint_names.size(), 2u);
    EXPECT_EQ(received_joint_names[0], "left_finger_joint");
    EXPECT_EQ(received_joint_names[1], "right_finger_joint");
    ASSERT_EQ(received_positions.size(), 2u);
    EXPECT_DOUBLE_EQ(received_positions[0], 0.035);
    EXPECT_DOUBLE_EQ(received_positions[1], 0.035);
  }

  // 8. VERIFY ACTION SERVER SUCCESS RESULT WAS PROPAGATED TO PROTOBUF
  const auto* resp_reflection = dynamic_response->GetReflection();
  const auto* status_field = res_desc->FindFieldByName("aquarium_skill_status");
  ASSERT_NE(status_field, nullptr);
  EXPECT_EQ(resp_reflection->GetString(*dynamic_response, status_field),
            "SUCCEEDED");

  const auto* reached_field = res_desc->FindFieldByName("reached_goal");
  ASSERT_NE(reached_field, nullptr);
  EXPECT_TRUE(resp_reflection->GetBool(*dynamic_response, reached_field));

  const auto* stalled_field = res_desc->FindFieldByName("stalled");
  ASSERT_NE(stalled_field, nullptr);
  EXPECT_FALSE(resp_reflection->GetBool(*dynamic_response, stalled_field));

  const auto* state_field = res_desc->FindFieldByName("state");
  ASSERT_NE(state_field, nullptr);
  const auto& state_msg =
      resp_reflection->GetMessage(*dynamic_response, state_field);
  const auto* state_name_field =
      state_msg.GetDescriptor()->FindFieldByName("name");
  ASSERT_NE(state_name_field, nullptr);
  ASSERT_EQ(state_msg.GetReflection()->FieldSize(state_msg, state_name_field),
            2);
  EXPECT_EQ(state_msg.GetReflection()->GetRepeatedString(state_msg,
                                                         state_name_field, 0),
            "left_finger_joint");
  EXPECT_EQ(state_msg.GetReflection()->GetRepeatedString(state_msg,
                                                         state_name_field, 1),
            "right_finger_joint");

  const auto* state_pos_field =
      state_msg.GetDescriptor()->FindFieldByName("position");
  ASSERT_NE(state_pos_field, nullptr);
  ASSERT_EQ(state_msg.GetReflection()->FieldSize(state_msg, state_pos_field),
            2);
  EXPECT_DOUBLE_EQ(state_msg.GetReflection()->GetRepeatedDouble(
                       state_msg, state_pos_field, 0),
                   0.035);
  EXPECT_DOUBLE_EQ(state_msg.GetReflection()->GetRepeatedDouble(
                       state_msg, state_pos_field, 1),
                   0.035);

  // 9. CLEANUP
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  action_server.reset();
  server_node.reset();
  aquarium_gateway::ShutdownGateway();
  std::filesystem::remove(temp_yaml_path);
}

TEST_F(ActionGatewayTest, ZenohQueryableProtoProxyingParallelGripperCommand) {
  // TEST external_interface: zenoh_queryable_proto (package: ai.intrinsic,
  // Params/Result)
  auto server_node =
      rclcpp::Node::make_shared("dummy_zenoh_proto_gripper_server_node");

  std::atomic<bool> server_received_goal{false};
  std::mutex goal_mutex;
  std::vector<std::string> received_joint_names;
  std::vector<double> received_positions;

  auto handle_goal =
      [](const rclcpp_action::GoalUUID& uuid,
         std::shared_ptr<const ParallelGripperCommand::Goal> goal) {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

  auto handle_cancel =
      [](const std::shared_ptr<GoalHandleParallelGripper> goal_handle) {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      };

  auto handle_accepted =
      [&server_received_goal, &received_joint_names, &received_positions,
       &goal_mutex](
          const std::shared_ptr<GoalHandleParallelGripper> goal_handle) {
        const auto goal = goal_handle->get_goal();
        {
          std::lock_guard<std::mutex> lock(goal_mutex);
          received_joint_names = goal->command.name;
          received_positions = goal->command.position;
          server_received_goal = true;
        }

        std::thread([goal_handle]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          auto result = std::make_shared<ParallelGripperCommand::Result>();
          result->state.header.frame_id = "gripper_base_link";
          result->state.name = {"left_finger_joint", "right_finger_joint"};
          result->state.position = {0.015, 0.015};
          result->reached_goal = true;
          result->stalled = false;
          goal_handle->succeed(result);
        }).detach();
      };

  auto action_server = rclcpp_action::create_server<ParallelGripperCommand>(
      server_node, "/zenoh_proto_gripper_action", handle_goal, handle_cancel,
      handle_accepted);

  aquarium_gateway::g_node =
      rclcpp::Node::make_shared("test_aquarium_gateway_zenoh_proto_node");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(aquarium_gateway::g_node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  std::string temp_yaml_path = (std::filesystem::temp_directory_path() /
                                "test_zenoh_proto_aquarium.yaml")
                                   .string();
  {
    std::ofstream yaml_out(temp_yaml_path);
    yaml_out << R"(
target:
  type: "node"
  package: "control_msgs"
  name: "dummy_zenoh_proto_gripper_server_node"
types:
  - name: "sensor_msgs/msg/JointState"
    fields:
      - "std_msgs/msg/Header header"
      - "string[] name"
      - "double[] position"
      - "double[] velocity"
      - "double[] effort"
  - name: "std_msgs/msg/Header"
    fields:
      - "builtin_interfaces/msg/Time stamp"
      - "string frame_id"
  - name: "builtin_interfaces/msg/Time"
    fields:
      - "int32 sec"
      - "uint32 nanosec"
interfaces:
  - internal_interface: "ros_action_client"
    internal_name: "/zenoh_proto_gripper_action"
    internal_type: "control_msgs/action/ParallelGripperCommand"
    external_name: "zenoh_proto_gripper_command"
    external_interface: "zenoh_queryable_proto"
    type_hash: "RIHS01_388b0a9cb34b22db9c6d67b752df85e173f40d6ad1d2a13824f114620f4d3fa1"
    request_fields:
      - "sensor_msgs/msg/JointState command"
    response_fields:
      - "sensor_msgs/msg/JointState state"
      - "bool stalled"
      - "bool reached_goal"
)";
  }

  google::protobuf::DescriptorPool pool;
  google::protobuf::DynamicMessageFactory factory(&pool);
  std::map<std::string, aquarium_gateway::ServiceConfig> config_cache;

  bool load_ok =
      aquarium_gateway::load_gateway_config(temp_yaml_path, pool, config_cache);
  ASSERT_TRUE(load_ok);

  const auto* params_desc =
      pool.FindMessageTypeByName("ai.intrinsic.ParallelGripperCommandParams");
  ASSERT_NE(params_desc, nullptr);
  const auto* result_desc =
      pool.FindMessageTypeByName("ai.intrinsic.ParallelGripperCommandResult");
  ASSERT_NE(result_desc, nullptr);

  std::unique_ptr<google::protobuf::Message> dynamic_request(
      factory.GetPrototype(params_desc)->New());
  const auto* req_reflection = dynamic_request->GetReflection();
  const auto* cmd_field = params_desc->FindFieldByName("command");
  ASSERT_NE(cmd_field, nullptr);
  auto* cmd_msg =
      req_reflection->MutableMessage(dynamic_request.get(), cmd_field);
  const auto* cmd_desc = cmd_msg->GetDescriptor();
  const auto* cmd_reflection = cmd_msg->GetReflection();

  const auto* name_field = cmd_desc->FindFieldByName("name");
  ASSERT_NE(name_field, nullptr);
  cmd_reflection->AddString(cmd_msg, name_field, "left_finger_joint");
  cmd_reflection->AddString(cmd_msg, name_field, "right_finger_joint");

  const auto* pos_field = cmd_desc->FindFieldByName("position");
  ASSERT_NE(pos_field, nullptr);
  cmd_reflection->AddDouble(cmd_msg, pos_field, 0.015);
  cmd_reflection->AddDouble(cmd_msg, pos_field, 0.015);

  std::unique_ptr<google::protobuf::Message> dynamic_response(
      factory.GetPrototype(result_desc)->New());

  std::string error_msg;
  const auto& srv_config = config_cache["zenoh_proto_gripper_command"];
  bool dispatch_ok = aquarium_gateway::DynamicActionTransactionDispatcher(
      srv_config.internal_name, srv_config.internal_type, srv_config.type_desc,
      *dynamic_request, *dynamic_response, error_msg);

  EXPECT_TRUE(dispatch_ok) << "Dispatcher failed with error: " << error_msg;

  EXPECT_TRUE(server_received_goal);
  {
    std::lock_guard<std::mutex> lock(goal_mutex);
    ASSERT_EQ(received_joint_names.size(), 2u);
    EXPECT_EQ(received_joint_names[0], "left_finger_joint");
    EXPECT_EQ(received_joint_names[1], "right_finger_joint");
    ASSERT_EQ(received_positions.size(), 2u);
    EXPECT_DOUBLE_EQ(received_positions[0], 0.015);
    EXPECT_DOUBLE_EQ(received_positions[1], 0.015);
  }

  const auto* resp_reflection = dynamic_response->GetReflection();
  const auto* status_field =
      result_desc->FindFieldByName("aquarium_skill_status");
  ASSERT_NE(status_field, nullptr);
  EXPECT_EQ(resp_reflection->GetString(*dynamic_response, status_field),
            "SUCCEEDED");

  const auto* reached_field = result_desc->FindFieldByName("reached_goal");
  ASSERT_NE(reached_field, nullptr);
  EXPECT_TRUE(resp_reflection->GetBool(*dynamic_response, reached_field));

  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  action_server.reset();
  server_node.reset();
  aquarium_gateway::ShutdownGateway();
  std::filesystem::remove(temp_yaml_path);
}

}  // namespace
