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

#ifndef AQUARIUM_GATEWAY__AQUARIUM_GATEWAY_HPP_
#define AQUARIUM_GATEWAY__AQUARIUM_GATEWAY_HPP_

#include <google/protobuf/descriptor.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/message.h>
#include <grpcpp/grpcpp.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "aquarium_gateway/serialization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "type_description_interfaces/msg/type_description.hpp"
#include "zenoh.hxx"

namespace aquarium_gateway {

enum class InterfaceType {
  ROS_SERVICE,
  ROS_TOPIC_SUBSCRIPTION,
  ROS_ACTION_CLIENT
};

struct ServiceConfig {
  InterfaceType interface_type = InterfaceType::ROS_SERVICE;
  std::string external_name;
  std::string external_interface;
  std::string internal_name;
  std::string internal_type;
  type_description_interfaces::msg::TypeDescription type_desc;
};

extern std::shared_ptr<rclcpp::Node> g_node;
extern std::map<std::string, ServiceConfig> g_config_cache;
extern std::mutex g_cache_mutex;
extern std::unique_ptr<grpc::Server> g_grpc_server;
extern std::string g_resource_name;
extern std::string g_capability;
extern int g_grpc_port;

void RegisterResourceRegistryDescriptors(
    google::protobuf::DescriptorPool& pool);

void setup_subscriptions(std::shared_ptr<rclcpp::Node> node);

void ShutdownGateway();

void RunGrpcServer(google::protobuf::DescriptorPool& pool,
                   google::protobuf::DynamicMessageFactory& factory);

void RunZenohServer(google::protobuf::DescriptorPool& pool,
                    google::protobuf::DynamicMessageFactory& factory);

bool build_dynamic_protobuf_schemas_in_memory(
    InterfaceType interface_type, const std::string& service_package,
    const std::string& service_base_name,
    const type_description_interfaces::msg::TypeDescription& type_desc,
    google::protobuf::DescriptorPool& target_pool,
    const std::string& external_interface = "");

bool load_gateway_config(const std::string& config_file,
                         google::protobuf::DescriptorPool& pool,
                         std::map<std::string, ServiceConfig>& out_cache);

bool DynamicTransactionDispatcher(
    const std::string& target_service_name,
    const std::string& target_service_type,
    const type_description_interfaces::msg::TypeDescription& type_desc,
    const google::protobuf::Message& dynamic_request,
    google::protobuf::Message& dynamic_response, std::string& error_msg);

bool DynamicActionTransactionDispatcher(
    const std::string& target_action_name,
    const std::string& target_action_type,
    const type_description_interfaces::msg::TypeDescription& type_desc,
    const google::protobuf::Message& dynamic_request,
    google::protobuf::Message& dynamic_response, std::string& error_msg);

void HandleZenohQuery(const zenoh::Query& query,
                      const ServiceConfig& srv_config,
                      google::protobuf::DescriptorPool& pool,
                      google::protobuf::DynamicMessageFactory& factory);

}  // namespace aquarium_gateway

#endif  // AQUARIUM_GATEWAY__AQUARIUM_GATEWAY_HPP_
