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

#include <grpcpp/grpcpp.h>

#include <backward_ros/backward.hpp>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "aquarium_gateway/aquarium_gateway.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  backward::SignalHandling sh;
  setvbuf(stdout, nullptr, _IONBF, 0);
  setvbuf(stderr, nullptr, _IONBF, 0);

  rclcpp::init(argc, argv);
  aquarium_gateway::g_node = rclcpp::Node::make_shared("aquarium_gateway");

  RCLCPP_INFO(aquarium_gateway::g_node->get_logger(),
              "Starting aquarium_gateway...");

  aquarium_gateway::g_node->declare_parameter<std::string>("config_file",
                                                           "aquarium.yaml");
  const std::string config_file =
      aquarium_gateway::g_node->get_parameter("config_file").as_string();

  aquarium_gateway::g_node->declare_parameter<std::string>("resource_name", "");
  aquarium_gateway::g_node->declare_parameter<std::string>("capability", "");
  aquarium_gateway::g_node->declare_parameter<int>("grpc_port", 50051);

  aquarium_gateway::g_resource_name =
      aquarium_gateway::g_node->get_parameter("resource_name").as_string();
  aquarium_gateway::g_capability =
      aquarium_gateway::g_node->get_parameter("capability").as_string();
  aquarium_gateway::g_grpc_port =
      aquarium_gateway::g_node->get_parameter("grpc_port").as_int();

  if (!aquarium_gateway::g_resource_name.empty() &&
      !aquarium_gateway::g_capability.empty()) {
    RCLCPP_INFO(aquarium_gateway::g_node->get_logger(),
                "Resource registration enabled: resource_name='%s', "
                "capability='%s', grpc_port=%d",
                aquarium_gateway::g_resource_name.c_str(),
                aquarium_gateway::g_capability.c_str(),
                aquarium_gateway::g_grpc_port);
  } else {
    RCLCPP_INFO(
        aquarium_gateway::g_node->get_logger(),
        "Resource registration skipped (resource_name or capability empty).");
  }

  // Build dynamic runtime Protobuf schema registry in memory.
  google::protobuf::DescriptorPool pool;
  google::protobuf::DynamicMessageFactory factory(&pool);
  aquarium_gateway::RegisterResourceRegistryDescriptors(pool);

  RCLCPP_INFO(aquarium_gateway::g_node->get_logger(),
              "Loading configuration: %s", config_file.c_str());
  if (!aquarium_gateway::load_gateway_config(
          config_file, pool, aquarium_gateway::g_config_cache)) {
    RCLCPP_FATAL(aquarium_gateway::g_node->get_logger(),
                 "Failed to load gateway configuration. Exiting.");
    return 1;
  }

  aquarium_gateway::setup_subscriptions(aquarium_gateway::g_node);

  // Spin ROS 2 node and generic gRPC server on background threads.
  std::thread ros_spin_thread([]() { rclcpp::spin(aquarium_gateway::g_node); });
  std::thread grpc_thread(
      [&pool, &factory]() { aquarium_gateway::RunGrpcServer(pool, factory); });

  aquarium_gateway::RunZenohServer(pool, factory);

  if (aquarium_gateway::g_grpc_server != nullptr) {
    RCLCPP_INFO(aquarium_gateway::g_node->get_logger(),
                "Shutting down gRPC server...");
    aquarium_gateway::g_grpc_server->Shutdown();
  }
  if (grpc_thread.joinable()) {
    grpc_thread.join();
  }
  aquarium_gateway::g_grpc_server.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  if (ros_spin_thread.joinable()) {
    ros_spin_thread.join();
  }

  aquarium_gateway::ShutdownGateway();
  return 0;
}
