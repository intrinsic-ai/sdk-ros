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

#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "aquarium_setup_assistant/config_writer.hpp"
#include "aquarium_setup_assistant/terminal_ui.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aquarium_setup_assistant {
namespace test {

TEST(TerminalUiTest, VerifyCatalogScreenRenderLayoutAndModes) {
  TerminalUi ui(true);

  std::vector<InterfaceRecord> mock_services = {
      {"/camera/capture_image", "sensor_msgs/srv/Trigger", true, false}};
  std::vector<InterfaceRecord> mock_topics = {
      {"/chatter", "std_msgs/msg/String", false, false}};
  std::vector<InterfaceRecord> mock_actions = {
      {"/gripper_command",
       "control_msgs/action/ParallelGripperCommand",
       false,
       false,
       false,
       "",
       {},
       {},
       true}};

  ui.update_interfaces(mock_services, MODE_SERVICE);
  ui.update_interfaces(mock_topics, MODE_TOPIC);
  ui.update_interfaces(mock_actions, MODE_ACTION);

  ui.render();
  std::string topic_screen = ui.get_clean_screen();
  EXPECT_NE(topic_screen.find("Aquarium Setup Assistant"), std::string::npos);
  EXPECT_NE(topic_screen.find("[T]opics      [S]ervices      [A]ctions"),
            std::string::npos);
  EXPECT_NE(topic_screen.find("[ ] /chatter"), std::string::npos);
  EXPECT_NE(topic_screen.find("Selected: 0 / Discovered: 3 (S:1, T:1, A:1)"),
            std::string::npos);

  ui.process_key(KEY_SWITCH_SERVICE);
  ui.render();
  std::string service_screen = ui.get_clean_screen();
  EXPECT_NE(service_screen.find("[ ] /camera/capture_image"),
            std::string::npos);

  // Selecting /camera/capture_image must return it in get_selected_interfaces.
  ui.process_key(KEY_SPACE);
  auto selected_srvs = ui.get_selected_interfaces();
  ASSERT_EQ(selected_srvs.size(), 1u);
  EXPECT_EQ(selected_srvs[0].name, "/camera/capture_image");
  ui.process_key(KEY_SPACE);

  ui.process_key(KEY_SWITCH_ACTION);
  ui.render();
  std::string action_screen = ui.get_clean_screen();
  EXPECT_NE(action_screen.find("[ ] /gripper_command"), std::string::npos);
}

TEST(TerminalUiTest, VerifyKeyboardSpaceToggleAction) {
  TerminalUi ui(true);

  std::vector<InterfaceRecord> mock_actions = {
      {"/gripper_command",
       "control_msgs/action/ParallelGripperCommand",
       false,
       false,
       false,
       "",
       {},
       {},
       true}};
  ui.update_interfaces(mock_actions, MODE_ACTION);
  ui.process_key(KEY_SWITCH_ACTION);

  ui.queue_test_key(KEY_SPACE);
  ui.queue_test_key(KEY_QUIT);

  while (ui.is_running()) {
    KeyPress key = ui.read_input();
    if (key == KEY_QUIT) {
      ui.process_key(key);
    } else {
      ui.process_key(key);
      ui.render();
    }
  }

  std::string clean_screen = ui.get_clean_screen();
  EXPECT_NE(clean_screen.find("[X] /gripper_command"), std::string::npos);
  EXPECT_NE(clean_screen.find("Selected: 1 / Discovered: 1 (S:0, T:0, A:1)"),
            std::string::npos);
}

TEST(TerminalUiTest, VerifyMasterDetailViewportNavigation) {
  TerminalUi ui(true);

  std::vector<InterfaceRecord> mock_actions = {
      {"/gripper_action",
       "control_msgs/action/ParallelGripperCommand",
       false,
       false,
       false,
       "",
       {},
       {},
       true}};
  ui.update_interfaces(mock_actions, MODE_ACTION);
  ui.process_key(KEY_SWITCH_ACTION);
  EXPECT_EQ(ui.get_active_state(), STATE_CATALOG);

  IntrospectionRecord details;
  details.interface_name = "/gripper_action";
  details.type_name = "control_msgs/action/ParallelGripperCommand";
  details.type_hash =
      "RIHS01_388b0a9cb34b22db9c6d67b752df85e173f40d6ad1d2a13824f114620f4d3fa1";
  details.request_fields = {"sensor_msgs/JointState command"};
  details.response_fields = {"sensor_msgs/JointState state", "bool stalled",
                             "bool reached_goal"};
  ui.update_introspection_details(details);

  ui.process_key(KEY_ENTER);
  EXPECT_EQ(ui.get_active_state(), STATE_DETAIL);

  ui.render();
  std::string detail_screen = ui.get_clean_screen();
  EXPECT_NE(detail_screen.find("Interface Details"), std::string::npos);
  EXPECT_NE(detail_screen.find("Name:  /gripper_action"), std::string::npos);
  EXPECT_NE(
      detail_screen.find("Type:  control_msgs/action/ParallelGripperCommand"),
      std::string::npos);
  EXPECT_NE(
      detail_screen.find(
          "Type ID Hash:  "
          "RIHS01_"
          "388b0a9cb34b22db9c6d67b752df85e173f40d6ad1d2a13824f114620f4d3fa1"),
      std::string::npos);
  EXPECT_NE(detail_screen.find("Request Message Schema Fields:"),
            std::string::npos);
  EXPECT_NE(detail_screen.find("- sensor_msgs/JointState command"),
            std::string::npos);
  EXPECT_NE(detail_screen.find("Response/Message Schema Fields:"),
            std::string::npos);
  EXPECT_NE(detail_screen.find("- sensor_msgs/JointState state"),
            std::string::npos);
  EXPECT_NE(detail_screen.find("- bool stalled"), std::string::npos);
  EXPECT_NE(detail_screen.find("- bool reached_goal"), std::string::npos);
  EXPECT_NE(detail_screen.find("[Escape] Return to list"), std::string::npos);

  ui.process_key(KEY_ESCAPE);
  EXPECT_EQ(ui.get_active_state(), STATE_CATALOG);

  ui.render();
  std::string catalog_screen = ui.get_clean_screen();
  EXPECT_NE(catalog_screen.find("[ ] /gripper_action"), std::string::npos);
}

TEST(TerminalUiTest, VerifyConfigWriterYamlOutput) {
  auto node = std::make_shared<rclcpp::Node>("test_config_writer_node");

  node->declare_parameter<std::string>("target_type", "node");
  node->declare_parameter<std::string>("target_package", "test_package");
  node->declare_parameter<std::string>("target_name", "test_node");
  node->declare_parameter<std::vector<std::string>>(
      "target_arguments", std::vector<std::string>{"--arg1", "val1"});

  std::vector<InterfaceRecord> selected = {
      {"/my_service",
       "my_package/srv/MySrv",
       true,
       true,
       true,
       "RIHS01_service_hash",
       {"int32 req_f"},
       {"int32 resp_f"},
       false},
      {"/my_topic",
       "my_package/msg/MyMsg",
       false,
       true,
       true,
       "RIHS01_topic_hash",
       {},
       {"string topic_f"},
       false},
      {"/my_gripper",
       "control_msgs/action/ParallelGripperCommand",
       false,
       true,
       true,
       "RIHS01_action_hash",
       {"sensor_msgs/JointState command"},
       {"sensor_msgs/JointState state", "bool stalled", "bool reached_goal"},
       true}};

  std::string test_file = "test_aquarium.yaml";
  std::remove(test_file.c_str());

  EXPECT_TRUE(write_config_yaml(node, selected, test_file));

  std::ifstream in(test_file);
  ASSERT_TRUE(in.is_open());
  std::stringstream buffer;
  buffer << in.rdbuf();
  const std::string content = buffer.str();
  in.close();

  EXPECT_NE(content.find("target:"), std::string::npos);
  EXPECT_NE(content.find("  type: \"node\""), std::string::npos);
  EXPECT_NE(content.find("  package: \"test_package\""), std::string::npos);
  EXPECT_NE(content.find("  name: \"test_node\""), std::string::npos);
  EXPECT_NE(content.find("interfaces:"), std::string::npos);

  EXPECT_NE(content.find("  - internal_interface: \"ros_service\""),
            std::string::npos);
  EXPECT_NE(content.find("    internal_name: \"/my_service\""),
            std::string::npos);
  EXPECT_NE(content.find("    internal_type: \"my_package/srv/MySrv\""),
            std::string::npos);
  EXPECT_NE(content.find("    external_type: \"my_package.srv.MySrv\""),
            std::string::npos);
  EXPECT_NE(content.find("    type_hash: \"RIHS01_service_hash\""),
            std::string::npos);
  EXPECT_NE(content.find("      - \"int32 req_f\""), std::string::npos);
  EXPECT_NE(content.find("      - \"int32 resp_f\""), std::string::npos);

  EXPECT_NE(content.find("  - internal_interface: \"ros_topic_subscription\""),
            std::string::npos);
  EXPECT_NE(content.find("    internal_name: \"/my_topic\""),
            std::string::npos);
  EXPECT_NE(content.find("      - \"string topic_f\""), std::string::npos);

  EXPECT_NE(content.find("  - internal_interface: \"ros_action_client\""),
            std::string::npos);
  EXPECT_NE(content.find("    internal_name: \"/my_gripper\""),
            std::string::npos);
  EXPECT_NE(
      content.find(
          "    internal_type: \"control_msgs/action/ParallelGripperCommand\""),
      std::string::npos);
  EXPECT_NE(content.find("      - \"sensor_msgs/JointState command\""),
            std::string::npos);
  EXPECT_NE(content.find("      - \"bool stalled\""), std::string::npos);
  EXPECT_NE(content.find("      - \"bool reached_goal\""), std::string::npos);

  std::remove(test_file.c_str());
}

TEST(TerminalUiTest, VerifyParseCliTargetArgs) {
  TargetSpec spec;

  EXPECT_TRUE(
      parse_cli_target_args({"aquarium_setup_assistant", "demo_gripper_driver",
                             "gripper.launch.xml", "sim:=true"},
                            spec));
  EXPECT_EQ(spec.type, "launch");
  EXPECT_EQ(spec.package, "demo_gripper_driver");
  EXPECT_EQ(spec.name, "gripper.launch.xml");
  ASSERT_EQ(spec.arguments.size(), 1u);
  EXPECT_EQ(spec.arguments[0], "sim:=true");

  EXPECT_TRUE(parse_cli_target_args({"aquarium_setup_assistant", "launch",
                                     "aquarium", "add_two_ints.launch.xml"},
                                    spec));
  EXPECT_EQ(spec.type, "launch");
  EXPECT_EQ(spec.package, "aquarium");
  EXPECT_EQ(spec.name, "add_two_ints.launch.xml");
  EXPECT_TRUE(spec.arguments.empty());

  EXPECT_TRUE(parse_cli_target_args(
      {"aquarium_setup_assistant", "run", "demo_nodes_cpp",
       "add_two_ints_server", "--selection", "selection.yaml", "--arg1"},
      spec));
  EXPECT_EQ(spec.type, "node");
  EXPECT_EQ(spec.package, "demo_nodes_cpp");
  EXPECT_EQ(spec.name, "add_two_ints_server");
  EXPECT_EQ(spec.selection_file, "selection.yaml");
  ASSERT_EQ(spec.arguments.size(), 1u);
  EXPECT_EQ(spec.arguments[0], "--arg1");

  EXPECT_FALSE(parse_cli_target_args({"aquarium_setup_assistant"}, spec));
}

}  // namespace test
}  // namespace aquarium_setup_assistant

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
