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

#include "aquarium_gateway/aquarium_gateway.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/dynamic_message.h>
#include <grpcpp/generic/callback_generic_service.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/support/byte_buffer.h>
#include <grpcpp/support/server_callback.h>
#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "aquarium_gateway/serialization.hpp"
#include "rcl_action/action_client.h"
#include "rcl_action/rcl_action.h"
#include "rcl_action/wait.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rmw/rmw.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/string.h"
#include "type_description_interfaces/msg/field_type.hpp"
#include "type_description_interfaces/msg/type_description.hpp"
#include "yaml-cpp/yaml.h"
#include "zenoh.hxx"

namespace aquarium_gateway {

std::shared_ptr<rclcpp::Node> g_node = nullptr;
std::map<std::string, ServiceConfig> g_config_cache;
std::mutex g_cache_mutex;
std::unique_ptr<grpc::Server> g_grpc_server = nullptr;

std::string g_resource_name;
std::string g_capability;
int g_grpc_port = 50051;

namespace {

struct SubscriptionCache {
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<uint8_t> cached_data;
  bool has_data = false;
  rclcpp::Time last_publish_time{0, 0, RCL_ROS_TIME};
  DynamicLayout layout;
};

std::map<std::string, std::shared_ptr<SubscriptionCache>> g_subscription_caches;
std::mutex g_caches_mutex;
std::vector<std::shared_ptr<rclcpp::GenericSubscription>> g_subscriptions;

struct CachedServiceClient {
  rcl_client_t client;
  std::mutex mutex;
  bool is_initialized = false;
};

std::map<std::string, std::shared_ptr<CachedServiceClient>>
    g_cached_service_clients;
std::mutex g_cached_service_clients_mutex;

struct CachedActionClient {
  rcl_action_client_t client;
  std::mutex mutex;
  bool is_initialized = false;
};

std::map<std::string, std::shared_ptr<CachedActionClient>>
    g_cached_action_clients;
std::mutex g_cached_action_clients_mutex;

rclcpp::Logger GetGatewayLogger() {
  if (g_node != nullptr) {
    return g_node->get_logger();
  }
  return rclcpp::get_logger("aquarium_gateway");
}

std::string RosToProtoMessageName(const std::string& ros_type) {
  std::string s = ros_type;
  std::replace(s.begin(), s.end(), '/', '_');
  std::replace(s.begin(), s.end(), ':', '_');
  return s;
}

void ResolveMessageSchemaNames(InterfaceType interface_type,
                               const std::string& package,
                               const std::string& base_name,
                               const std::string& external_interface,
                               std::string& proto_package,
                               std::string& req_message_name,
                               std::string& resp_message_name) {
  if (external_interface == "zenoh_queryable_proto") {
    proto_package = "ai.intrinsic";
    req_message_name = base_name + "Params";
    resp_message_name = base_name + "Result";
    return;
  }

  proto_package =
      package +
      (interface_type == InterfaceType::ROS_ACTION_CLIENT
           ? ".action"
           : (interface_type == InterfaceType::ROS_SERVICE ? ".srv" : ".msg"));
  req_message_name =
      base_name +
      (interface_type == InterfaceType::ROS_ACTION_CLIENT
           ? "_Goal"
           : (interface_type == InterfaceType::ROS_SERVICE ? "_Request"
                                                           : "_Params"));
  resp_message_name =
      base_name + (interface_type == InterfaceType::ROS_ACTION_CLIENT
                       ? "_Result"
                       : "_Response");
}

void ParseYamlFieldList(
    const YAML::Node& field_list_node,
    std::vector<type_description_interfaces::msg::Field>& out_fields) {
  if (!field_list_node) {
    return;
  }
  for (const auto& f_str : field_list_node) {
    const std::string field_line = f_str.as<std::string>();
    const size_t space_pos = field_line.find(' ');
    if (space_pos == std::string::npos) {
      continue;
    }
    const std::string t_str = field_line.substr(0, space_pos);
    const std::string n_str = field_line.substr(space_pos + 1);

    type_description_interfaces::msg::Field field;
    field.name = n_str;
    field.type.type_id = parse_field_type(t_str, field.type.nested_type_name,
                                          field.type.capacity);
    out_fields.push_back(field);
  }
}

void AddReferencedTypes(
    const type_description_interfaces::msg::IndividualTypeDescription& desc,
    const std::map<std::string,
                   type_description_interfaces::msg::IndividualTypeDescription>&
        custom_types,
    type_description_interfaces::msg::TypeDescription& type_desc) {
  for (const auto& f : desc.fields) {
    const DecodedFieldType decoded = DecodeFieldTypeId(f.type.type_id);
    if (decoded.base_type_id ==
        type_description_interfaces::msg::FieldType::FIELD_TYPE_NESTED_TYPE) {
      const std::string& nested_name = f.type.nested_type_name;
      bool exists = false;
      for (const auto& ref : type_desc.referenced_type_descriptions) {
        if (ref.type_name == nested_name) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        auto it = custom_types.find(nested_name);
        if (it != custom_types.end()) {
          type_desc.referenced_type_descriptions.push_back(it->second);
          AddReferencedTypes(it->second, custom_types, type_desc);
        } else {
          RCLCPP_WARN(GetGatewayLogger(),
                      "Referenced custom type not defined in config: %s",
                      nested_name.c_str());
        }
      }
    }
  }
}

void GenerateRandomUuid(uint8_t uuid[16]) {
  thread_local std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<uint32_t> dist(0, 255);
  for (int i = 0; i < 16; ++i) {
    uuid[i] = static_cast<uint8_t>(dist(rng));
  }
  uuid[6] = static_cast<uint8_t>((uuid[6] & 0x0F) | 0x40);
  uuid[8] = static_cast<uint8_t>((uuid[8] & 0x3F) | 0x80);
}

void SetResponseStatusFields(google::protobuf::Message* dynamic_response,
                             bool ok, const std::string& error_msg,
                             bool preserve_existing_skill_status) {
  const auto* resp_desc = dynamic_response->GetDescriptor();
  const auto* resp_reflection = dynamic_response->GetReflection();

  const auto* skill_status_field =
      resp_desc->FindFieldByName("aquarium_skill_status");
  if (skill_status_field != nullptr &&
      skill_status_field->type() ==
          google::protobuf::FieldDescriptor::TYPE_STRING) {
    if (!preserve_existing_skill_status ||
        resp_reflection->GetString(*dynamic_response, skill_status_field)
            .empty()) {
      resp_reflection->SetString(dynamic_response, skill_status_field,
                                 ok ? "SUCCEEDED" : "FAILED: " + error_msg);
    }
  }

  const auto* status_field = resp_desc->FindFieldByName("status");
  if (status_field != nullptr &&
      status_field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    google::protobuf::Message* status_msg =
        resp_reflection->MutableMessage(dynamic_response, status_field);
    const auto* status_desc = status_msg->GetDescriptor();
    const auto* status_reflection = status_msg->GetReflection();
    const auto* code_field = status_desc->FindFieldByName("code");
    const auto* msg_field = status_desc->FindFieldByName("message");
    if (code_field != nullptr) {
      status_reflection->SetInt32(status_msg, code_field, ok ? 0 : 13);
    }
    if (msg_field != nullptr && !ok) {
      status_reflection->SetString(status_msg, msg_field, error_msg);
    }
  }
}

void PopulateResourceInstanceProto(google::protobuf::Message* instance,
                                   const std::string& resource_name,
                                   const std::string& address) {
  const auto* inst_desc = instance->GetDescriptor();
  const auto* inst_refl = instance->GetReflection();
  inst_refl->SetString(instance, inst_desc->FindFieldByName("name"),
                       resource_name);

  google::protobuf::Message* handle = inst_refl->MutableMessage(
      instance, inst_desc->FindFieldByName("resource_handle"));
  const auto* handle_desc = handle->GetDescriptor();
  const auto* handle_refl = handle->GetReflection();
  handle_refl->SetString(handle, handle_desc->FindFieldByName("name"),
                         resource_name);

  google::protobuf::Message* conn = handle_refl->MutableMessage(
      handle, handle_desc->FindFieldByName("connection_info"));
  const auto* conn_desc = conn->GetDescriptor();
  const auto* conn_refl = conn->GetReflection();

  google::protobuf::Message* grpc_msg =
      conn_refl->MutableMessage(conn, conn_desc->FindFieldByName("grpc"));
  const auto* grpc_desc = grpc_msg->GetDescriptor();
  const auto* grpc_refl = grpc_msg->GetReflection();
  grpc_refl->SetString(grpc_msg, grpc_desc->FindFieldByName("address"),
                       address);
}

bool IsRunningInCluster() {
  const char* k8s_host = std::getenv("KUBERNETES_SERVICE_HOST");
  if (k8s_host != nullptr && k8s_host[0] != '\0') {
    return true;
  }
  struct stat sb;
  if (stat("/etc/intrinsic/runtime_config.pb", &sb) == 0) {
    return true;
  }
  std::ifstream resolv("/etc/resolv.conf");
  if (resolv.is_open()) {
    std::string line;
    while (std::getline(resolv, line)) {
      if (line.rfind("search", 0) == 0 &&
          line.find("cluster.local") != std::string::npos) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

void RegisterResourceRegistryDescriptors(
    google::protobuf::DescriptorPool& pool) {
  using google::protobuf::DescriptorProto;
  using google::protobuf::FieldDescriptorProto;
  using google::protobuf::FileDescriptorProto;

  FileDescriptorProto file_proto;
  file_proto.set_name("resource_registry.proto");
  file_proto.set_package("intrinsic_proto.resources");
  file_proto.set_syntax("proto3");

  DescriptorProto* grpc_info = file_proto.add_message_type();
  grpc_info->set_name("ResourceGrpcConnectionInfo");
  FieldDescriptorProto* field = grpc_info->add_field();
  field->set_name("address");
  field->set_number(1);
  field->set_type(FieldDescriptorProto::TYPE_STRING);
  field->set_label(FieldDescriptorProto::LABEL_OPTIONAL);

  DescriptorProto* conn_info = file_proto.add_message_type();
  conn_info->set_name("ResourceConnectionInfo");
  field = conn_info->add_field();
  field->set_name("grpc");
  field->set_number(1);
  field->set_type(FieldDescriptorProto::TYPE_MESSAGE);
  field->set_type_name("intrinsic_proto.resources.ResourceGrpcConnectionInfo");
  field->set_label(FieldDescriptorProto::LABEL_OPTIONAL);

  DescriptorProto* res_handle = file_proto.add_message_type();
  res_handle->set_name("ResourceHandle");
  field = res_handle->add_field();
  field->set_name("name");
  field->set_number(1);
  field->set_type(FieldDescriptorProto::TYPE_STRING);
  field->set_label(FieldDescriptorProto::LABEL_OPTIONAL);

  field = res_handle->add_field();
  field->set_name("connection_info");
  field->set_number(3);
  field->set_type(FieldDescriptorProto::TYPE_MESSAGE);
  field->set_type_name("intrinsic_proto.resources.ResourceConnectionInfo");
  field->set_label(FieldDescriptorProto::LABEL_OPTIONAL);

  DescriptorProto* res_inst = file_proto.add_message_type();
  res_inst->set_name("ResourceInstance");
  field = res_inst->add_field();
  field->set_name("name");
  field->set_number(1);
  field->set_type(FieldDescriptorProto::TYPE_STRING);
  field->set_label(FieldDescriptorProto::LABEL_OPTIONAL);

  field = res_inst->add_field();
  field->set_name("resource_handle");
  field->set_number(11);
  field->set_type(FieldDescriptorProto::TYPE_MESSAGE);
  field->set_type_name("intrinsic_proto.resources.ResourceHandle");
  field->set_label(FieldDescriptorProto::LABEL_OPTIONAL);

  DescriptorProto* list_resp = file_proto.add_message_type();
  list_resp->set_name("ListResourceInstanceResponse");
  field = list_resp->add_field();
  field->set_name("instances");
  field->set_number(1);
  field->set_type(FieldDescriptorProto::TYPE_MESSAGE);
  field->set_type_name("intrinsic_proto.resources.ResourceInstance");
  field->set_label(FieldDescriptorProto::LABEL_REPEATED);

  pool.BuildFile(file_proto);
}

bool build_dynamic_protobuf_schemas_in_memory(
    InterfaceType interface_type, const std::string& service_package,
    const std::string& service_base_name,
    const type_description_interfaces::msg::TypeDescription& type_desc,
    google::protobuf::DescriptorPool& target_pool,
    const std::string& external_interface) {
  google::protobuf::FileDescriptorProto file_proto;
  const bool is_zenoh_proto = (external_interface == "zenoh_queryable_proto");

  std::string proto_package;
  std::string req_message_name;
  std::string resp_message_name;
  ResolveMessageSchemaNames(interface_type, service_package, service_base_name,
                            external_interface, proto_package, req_message_name,
                            resp_message_name);

  file_proto.set_name(proto_package + "_" + service_base_name + ".proto");
  file_proto.set_package(proto_package);
  file_proto.set_syntax("proto3");

  std::string req_type_name;
  std::string resp_type_name;
  for (const auto& f : type_desc.type_description.fields) {
    if (f.name == "request_message" || f.name == "goal") {
      req_type_name = f.type.nested_type_name;
    } else if (f.name == "response_message" || f.name == "result") {
      resp_type_name = f.type.nested_type_name;
    }
  }

  for (const auto& nested : type_desc.referenced_type_descriptions) {
    std::string msg_name = RosToProtoMessageName(nested.type_name);
    if ((!req_type_name.empty() && nested.type_name == req_type_name) ||
        (req_type_name.empty() &&
         ((nested.type_name.size() >= 5 &&
           nested.type_name.rfind("_Goal") == nested.type_name.size() - 5) ||
          (nested.type_name.size() >= 8 && nested.type_name.rfind("_Request") ==
                                               nested.type_name.size() - 8)))) {
      msg_name = req_message_name;
    } else if ((!resp_type_name.empty() &&
                nested.type_name == resp_type_name) ||
               (resp_type_name.empty() &&
                ((nested.type_name.size() >= 7 &&
                  nested.type_name.rfind("_Result") ==
                      nested.type_name.size() - 7) ||
                 (nested.type_name.size() >= 9 &&
                  nested.type_name.rfind("_Response") ==
                      nested.type_name.size() - 9)))) {
      msg_name = resp_message_name;
    }

    const std::string full_proto_msg_name = proto_package + "." + msg_name;
    const google::protobuf::Descriptor* existing =
        target_pool.FindMessageTypeByName(full_proto_msg_name);
    if (existing != nullptr) {
      bool dep_exists = false;
      for (int i = 0; i < file_proto.dependency_size(); ++i) {
        if (file_proto.dependency(i) == existing->file()->name()) {
          dep_exists = true;
          break;
        }
      }
      if (!dep_exists) {
        file_proto.add_dependency(existing->file()->name());
      }
      continue;
    }

    google::protobuf::DescriptorProto* msg_proto =
        file_proto.add_message_type();
    msg_proto->set_name(msg_name);

    int field_number = 1;
    if (msg_name == resp_message_name &&
        (is_zenoh_proto ||
         interface_type == InterfaceType::ROS_ACTION_CLIENT)) {
      google::protobuf::FieldDescriptorProto* fp = msg_proto->add_field();
      fp->set_name("aquarium_skill_status");
      fp->set_number(field_number++);
      fp->set_type(google::protobuf::FieldDescriptorProto::TYPE_STRING);
      fp->set_label(
          google::protobuf::FieldDescriptorProto_Label_LABEL_OPTIONAL);
    }

    for (const auto& f : nested.fields) {
      google::protobuf::FieldDescriptorProto* fp = msg_proto->add_field();
      fp->set_name(f.name);
      fp->set_number(field_number++);

      const DecodedFieldType decoded = DecodeFieldTypeId(f.type.type_id);
      fp->set_label(
          decoded.is_collection()
              ? google::protobuf::FieldDescriptorProto_Label_LABEL_REPEATED
              : google::protobuf::FieldDescriptorProto_Label_LABEL_OPTIONAL);

      if (decoded.base_type_id ==
          type_description_interfaces::msg::FieldType::FIELD_TYPE_NESTED_TYPE) {
        fp->set_type(google::protobuf::FieldDescriptorProto::TYPE_MESSAGE);
        fp->set_type_name("." + proto_package + "." +
                          RosToProtoMessageName(f.type.nested_type_name));
      } else {
        using FieldType = type_description_interfaces::msg::FieldType;
        google::protobuf::FieldDescriptorProto::Type proto_type =
            google::protobuf::FieldDescriptorProto::TYPE_INT64;
        switch (decoded.base_type_id) {
          case FieldType::FIELD_TYPE_INT8:
          case FieldType::FIELD_TYPE_INT16:
          case FieldType::FIELD_TYPE_INT32:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_INT32;
            break;
          case FieldType::FIELD_TYPE_UINT8:
          case FieldType::FIELD_TYPE_BYTE:
          case FieldType::FIELD_TYPE_UINT16:
          case FieldType::FIELD_TYPE_UINT32:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_UINT32;
            break;
          case FieldType::FIELD_TYPE_INT64:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_INT64;
            break;
          case FieldType::FIELD_TYPE_UINT64:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_UINT64;
            break;
          case FieldType::FIELD_TYPE_FLOAT:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_FLOAT;
            break;
          case FieldType::FIELD_TYPE_DOUBLE:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_DOUBLE;
            break;
          case FieldType::FIELD_TYPE_BOOLEAN:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_BOOL;
            break;
          case FieldType::FIELD_TYPE_CHAR:
          case FieldType::FIELD_TYPE_STRING:
          case FieldType::FIELD_TYPE_BOUNDED_STRING:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_STRING;
            break;
          default:
            proto_type = google::protobuf::FieldDescriptorProto::TYPE_INT64;
            break;
        }
        fp->set_type(proto_type);
      }
    }
  }

  google::protobuf::ServiceDescriptorProto* srv_proto =
      file_proto.add_service();
  srv_proto->set_name(service_base_name + "_Service");

  google::protobuf::MethodDescriptorProto* m_proto = srv_proto->add_method();
  m_proto->set_name("Call");
  m_proto->set_input_type("." + proto_package + "." + req_message_name);
  m_proto->set_output_type("." + proto_package + "." + resp_message_name);

  const google::protobuf::FileDescriptor* fd =
      target_pool.BuildFile(file_proto);
  if (fd == nullptr) {
    RCLCPP_ERROR(GetGatewayLogger(),
                 "Failed to compile dynamic FileDescriptorProto in memory: %s",
                 file_proto.DebugString().c_str());
    return false;
  }
  return true;
}

bool load_gateway_config(const std::string& config_file,
                         google::protobuf::DescriptorPool& pool,
                         std::map<std::string, ServiceConfig>& out_cache) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(GetGatewayLogger(), "Failed to load YAML config '%s': %s",
                 config_file.c_str(), e.what());
    return false;
  }

  std::map<std::string,
           type_description_interfaces::msg::IndividualTypeDescription>
      custom_types;
  if (config["types"]) {
    for (const auto& item : config["types"]) {
      const std::string name = item["name"].as<std::string>();
      type_description_interfaces::msg::IndividualTypeDescription type_desc;
      type_desc.type_name = name;
      ParseYamlFieldList(item["fields"], type_desc.fields);
      custom_types[name] = type_desc;
    }
  }

  if (!config["interfaces"]) {
    RCLCPP_ERROR(GetGatewayLogger(), "Config missing 'interfaces' key.");
    return false;
  }

  for (const auto& item : config["interfaces"]) {
    if (!item["internal_interface"]) {
      continue;
    }
    const std::string internal_interface =
        item["internal_interface"].as<std::string>();
    InterfaceType interface_type;
    if (internal_interface == "ros_service") {
      interface_type = InterfaceType::ROS_SERVICE;
    } else if (internal_interface == "ros_topic_subscription") {
      interface_type = InterfaceType::ROS_TOPIC_SUBSCRIPTION;
    } else if (internal_interface == "ros_action_client") {
      interface_type = InterfaceType::ROS_ACTION_CLIENT;
    } else {
      continue;
    }

    const std::string internal_name = item["internal_name"].as<std::string>();
    const std::string internal_type = item["internal_type"].as<std::string>();
    const std::string external_interface =
        item["external_interface"]
            ? item["external_interface"].as<std::string>()
            : "";

    std::string package;
    std::string base_name;
    if (!parse_ros_type(internal_type, package, base_name)) {
      RCLCPP_ERROR(GetGatewayLogger(), "Invalid internal_type: %s",
                   internal_type.c_str());
      return false;
    }

    type_description_interfaces::msg::TypeDescription type_desc;
    type_desc.type_description.type_name = internal_type;

    type_description_interfaces::msg::IndividualTypeDescription req_desc;
    type_description_interfaces::msg::IndividualTypeDescription resp_desc;

    if (interface_type == InterfaceType::ROS_SERVICE ||
        interface_type == InterfaceType::ROS_ACTION_CLIENT) {
      req_desc.type_name =
          internal_type + (interface_type == InterfaceType::ROS_ACTION_CLIENT
                               ? "_Goal"
                               : "_Request");
      ParseYamlFieldList(item["request_fields"], req_desc.fields);

      if (interface_type == InterfaceType::ROS_ACTION_CLIENT) {
        const bool has_action_name =
            std::any_of(req_desc.fields.begin(), req_desc.fields.end(),
                        [](const auto& f) { return f.name == "action_name"; });
        if (!has_action_name) {
          type_description_interfaces::msg::Field action_name_f;
          action_name_f.name = "action_name";
          action_name_f.type.type_id =
              type_description_interfaces::msg::FieldType::FIELD_TYPE_STRING;
          req_desc.fields.push_back(action_name_f);
        }
      }

      resp_desc.type_name =
          internal_type + (interface_type == InterfaceType::ROS_ACTION_CLIENT
                               ? "_Result"
                               : "_Response");
      ParseYamlFieldList(item["response_fields"], resp_desc.fields);
    } else if (interface_type == InterfaceType::ROS_TOPIC_SUBSCRIPTION) {
      req_desc.type_name = internal_type + "_Request";
      if (external_interface == "zenoh_queryable_proto") {
        type_description_interfaces::msg::Field f1;
        f1.name = "wait_for_next_message";
        f1.type.type_id =
            type_description_interfaces::msg::FieldType::FIELD_TYPE_BOOLEAN;
        req_desc.fields.push_back(f1);

        type_description_interfaces::msg::Field f2;
        f2.name = "timeout_seconds";
        f2.type.type_id =
            type_description_interfaces::msg::FieldType::FIELD_TYPE_DOUBLE;
        req_desc.fields.push_back(f2);
      }

      resp_desc.type_name = internal_type + "_Response";
      ParseYamlFieldList(item["response_fields"], resp_desc.fields);
    }

    type_desc.referenced_type_descriptions.push_back(req_desc);
    type_desc.referenced_type_descriptions.push_back(resp_desc);

    AddReferencedTypes(req_desc, custom_types, type_desc);
    AddReferencedTypes(resp_desc, custom_types, type_desc);

    type_description_interfaces::msg::Field req_f;
    req_f.name = (interface_type == InterfaceType::ROS_ACTION_CLIENT)
                     ? "goal"
                     : "request_message";
    req_f.type.type_id =
        type_description_interfaces::msg::FieldType::FIELD_TYPE_NESTED_TYPE;
    req_f.type.nested_type_name = req_desc.type_name;
    type_desc.type_description.fields.push_back(req_f);

    type_description_interfaces::msg::Field resp_f;
    resp_f.name = (interface_type == InterfaceType::ROS_ACTION_CLIENT)
                      ? "result"
                      : "response_message";
    resp_f.type.type_id =
        type_description_interfaces::msg::FieldType::FIELD_TYPE_NESTED_TYPE;
    resp_f.type.nested_type_name = resp_desc.type_name;
    type_desc.type_description.fields.push_back(resp_f);

    if (!build_dynamic_protobuf_schemas_in_memory(interface_type, package,
                                                  base_name, type_desc, pool,
                                                  external_interface)) {
      RCLCPP_ERROR(GetGatewayLogger(),
                   "Failed to build Protobuf schemas for %s",
                   internal_type.c_str());
      return false;
    }

    const std::string external_name = item["external_name"].as<std::string>();

    ServiceConfig srv_config;
    srv_config.interface_type = interface_type;
    srv_config.external_name = external_name;
    srv_config.external_interface = external_interface;
    srv_config.internal_name = internal_name;
    srv_config.internal_type = internal_type;
    srv_config.type_desc = type_desc;

    out_cache[external_name] = srv_config;
    if (interface_type == InterfaceType::ROS_SERVICE ||
        interface_type == InterfaceType::ROS_ACTION_CLIENT) {
      std::string grpc_path = "/";
      if (external_interface == "zenoh_queryable_proto") {
        grpc_path += "ai.intrinsic." + base_name + "_Service/Call";
      } else {
        const std::string sub =
            (interface_type == InterfaceType::ROS_ACTION_CLIENT ? ".action."
                                                                : ".srv.");
        grpc_path += package + sub + base_name + "_Service/Call";
      }
      out_cache[grpc_path] = srv_config;
      RCLCPP_INFO(
          GetGatewayLogger(), "Mapped gRPC path '%s' -> ROS %s %s",
          grpc_path.c_str(),
          (interface_type == InterfaceType::ROS_ACTION_CLIENT ? "action client"
                                                              : "service"),
          internal_name.c_str());
    }

    const char* type_str = (interface_type == InterfaceType::ROS_ACTION_CLIENT
                                ? "action client"
                                : (interface_type == InterfaceType::ROS_SERVICE
                                       ? "service"
                                       : "topic subscription"));
    RCLCPP_INFO(GetGatewayLogger(), "Mapped Zenoh key '%s' -> ROS %s %s",
                external_name.c_str(), type_str, internal_name.c_str());
  }
  return true;
}

bool DynamicTransactionDispatcher(
    const std::string& target_service_name,
    const std::string& target_service_type,
    const type_description_interfaces::msg::TypeDescription& type_desc,
    const google::protobuf::Message& dynamic_request,
    google::protobuf::Message& dynamic_response, std::string& error_msg) {
  const auto* req_type_desc = FindIndividualType(type_desc, "_Request");
  const auto* res_type_desc = FindIndividualType(type_desc, "_Response");
  if (req_type_desc == nullptr || res_type_desc == nullptr) {
    error_msg = "Dynamic TDS struct incomplete.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  auto req_layout = ComputeDynamicLayout(*req_type_desc, type_desc);
  auto res_layout = ComputeDynamicLayout(*res_type_desc, type_desc);

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib;
  try {
    ts_lib = rclcpp::get_typesupport_library(target_service_type,
                                             "rosidl_typesupport_c");
  } catch (const std::exception& e) {
    error_msg = "Failed to load C typesupport library.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s (%s)", error_msg.c_str(), e.what());
    return false;
  }

  const rosidl_service_type_support_t* service_ts = nullptr;
  try {
    service_ts = rclcpp::get_service_typesupport_handle(
        target_service_type, "rosidl_typesupport_c", *ts_lib);
  } catch (const std::exception& e) {
    error_msg = "Failed to get C service typesupport handle.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s (%s)", error_msg.c_str(), e.what());
    return false;
  }

  if (g_node == nullptr) {
    error_msg = "g_node is not initialized.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  std::shared_ptr<CachedServiceClient> cached_client;
  {
    std::lock_guard<std::mutex> lock(g_cached_service_clients_mutex);
    auto it = g_cached_service_clients.find(target_service_name);
    if (it == g_cached_service_clients.end()) {
      cached_client = std::make_shared<CachedServiceClient>();
      cached_client->client = rcl_get_zero_initialized_client();
      g_cached_service_clients[target_service_name] = cached_client;
    } else {
      cached_client = it->second;
    }
  }

  std::lock_guard<std::mutex> client_lock(cached_client->mutex);
  auto* rcl_node = g_node->get_node_base_interface()->get_rcl_node_handle();

  if (!cached_client->is_initialized) {
    RCLCPP_INFO(GetGatewayLogger(), "Initializing rcl client on '%s'",
                target_service_name.c_str());
    rcl_client_options_t client_ops = rcl_client_get_default_options();
    rcl_ret_t ret =
        rcl_client_init(&cached_client->client, rcl_node, service_ts,
                        target_service_name.c_str(), &client_ops);
    if (ret != RCL_RET_OK) {
      error_msg = "Failed to initialize rcl client.";
      RCLCPP_ERROR(GetGatewayLogger(), "%s (ret=%d)", error_msg.c_str(), ret);
      return false;
    }
    cached_client->is_initialized = true;
  }

  bool is_available = false;
  rcl_ret_t ret = rcl_service_server_is_available(
      rcl_node, &cached_client->client, &is_available);
  if (ret != RCL_RET_OK || !is_available) {
    const auto start_wait = std::chrono::steady_clock::now();
    const auto wait_timeout = std::chrono::seconds(5);
    bool service_ready = false;
    while ((std::chrono::steady_clock::now() - start_wait) < wait_timeout &&
           rclcpp::ok()) {
      ret = rcl_service_server_is_available(rcl_node, &cached_client->client,
                                            &is_available);
      if (ret == RCL_RET_OK && is_available) {
        service_ready = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!service_ready) {
      error_msg = "Target ROS 2 service offline.";
      RCLCPP_ERROR(GetGatewayLogger(), "Remote target service %s offline.",
                   target_service_name.c_str());
      return false;
    }
  }

  const size_t alloc_req_size = std::max<size_t>(1, req_layout.total_size);
  std::vector<uint8_t> req_buf(alloc_req_size, 0);
  std::vector<std::unique_ptr<uint8_t[]>> allocations;
  MapProtobufToCStruct(dynamic_request, req_layout, req_buf.data(), allocations,
                       type_desc);

  int64_t seq_id = 0;
  ret = rcl_send_request(&cached_client->client, req_buf.data(), &seq_id);
  if (ret != RCL_RET_OK) {
    error_msg = "Failed to send ROS 2 request.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s (ret=%d)", error_msg.c_str(), ret);
    return false;
  }

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(
      &wait_set, 0, 0, 0, 1, 0, 0,
      rclcpp::contexts::get_global_default_context()->get_rcl_context().get(),
      rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    error_msg = "Failed to initialize wait set.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s (ret=%d)", error_msg.c_str(), ret);
    return false;
  }
  auto cleanup_wait_set = rcpputils::make_scope_exit([&wait_set]() {
    rcl_ret_t r = rcl_wait_set_fini(&wait_set);
    (void)r;
  });

  bool response_received = false;
  const auto start_wait_resp = std::chrono::steady_clock::now();
  const auto resp_timeout = std::chrono::seconds(5);

  while ((std::chrono::steady_clock::now() - start_wait_resp) < resp_timeout &&
         rclcpp::ok()) {
    ret = rcl_wait_set_clear(&wait_set);
    ret = rcl_wait_set_add_client(&wait_set, &cached_client->client, nullptr);

    constexpr int64_t kTimeoutNs = 50000000;  // 50 ms
    ret = rcl_wait(&wait_set, kTimeoutNs);
    if (ret == RCL_RET_TIMEOUT) {
      continue;
    }
    if (ret != RCL_RET_OK) {
      error_msg = "Wait for response failed.";
      RCLCPP_ERROR(GetGatewayLogger(), "rcl_wait failed: %d", ret);
      return false;
    }
    if (wait_set.clients[0] != nullptr) {
      response_received = true;
      break;
    }
  }

  if (!response_received) {
    error_msg = "ROS 2 node timed out.";
    RCLCPP_ERROR(GetGatewayLogger(), "Service transaction timed out on '%s'.",
                 target_service_name.c_str());
    return false;
  }

  const size_t alloc_res_size = std::max<size_t>(1, res_layout.total_size);
  std::vector<uint8_t> resp_buf(alloc_res_size, 0);
  rmw_request_id_t req_header;
  ret = rcl_take_response(&cached_client->client, &req_header, resp_buf.data());
  if (ret != RCL_RET_OK) {
    error_msg = "Failed to take response.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s (ret=%d)", error_msg.c_str(), ret);
    return false;
  }

  MapCStructToProtobuf(res_layout, resp_buf.data(), dynamic_response,
                       type_desc);
  return true;
}

bool DynamicActionTransactionDispatcher(
    const std::string& target_action_name,
    const std::string& target_action_type,
    const type_description_interfaces::msg::TypeDescription& type_desc,
    const google::protobuf::Message& dynamic_request,
    google::protobuf::Message& dynamic_response, std::string& error_msg) {
  RCLCPP_DEBUG(GetGatewayLogger(), "Dispatching action '%s' [type: %s]",
               target_action_name.c_str(), target_action_type.c_str());

  const auto* goal_type_desc = FindIndividualType(type_desc, "_Goal");
  if (goal_type_desc == nullptr) {
    goal_type_desc = FindIndividualType(type_desc, "_Request");
  }
  const auto* res_type_desc = FindIndividualType(type_desc, "_Result");
  if (res_type_desc == nullptr) {
    res_type_desc = FindIndividualType(type_desc, "_Response");
  }

  if (goal_type_desc == nullptr || res_type_desc == nullptr) {
    error_msg =
        "Dynamic Action TDS struct incomplete for " + target_action_name;
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  auto goal_layout = ComputeDynamicLayout(*goal_type_desc, type_desc);
  auto res_layout = ComputeDynamicLayout(*res_type_desc, type_desc);

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib;
  try {
    ts_lib = rclcpp::get_typesupport_library(target_action_type,
                                             "rosidl_typesupport_c");
  } catch (const std::exception& e) {
    error_msg =
        "Failed to load action typesupport library: " + std::string(e.what());
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  if (ts_lib == nullptr) {
    error_msg = "Typesupport library handle is null.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) =
      rclcpp::extract_type_identifier(target_action_type);
  if (middle_module.empty()) {
    middle_module = "action";
  }
  const std::string symbol_name =
      "rosidl_typesupport_c__get_action_type_support_handle__" + package_name +
      "__" + middle_module + "__" + type_name;

  if (!ts_lib->has_symbol(symbol_name)) {
    error_msg = "Symbol not found in typesupport library: " + symbol_name;
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  const rosidl_action_type_support_t* (*get_ts)() = nullptr;
  try {
    get_ts = reinterpret_cast<const rosidl_action_type_support_t* (*)()>(
        ts_lib->get_symbol(symbol_name));
  } catch (const std::exception& e) {
    error_msg =
        "Exception retrieving typesupport symbol: " + std::string(e.what());
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  if (get_ts == nullptr) {
    error_msg =
        "Null function pointer for action typesupport symbol: " + symbol_name;
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  const rosidl_action_type_support_t* action_ts = get_ts();
  if (action_ts == nullptr) {
    error_msg = "Action typesupport handle is null.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  if (g_node == nullptr) {
    error_msg = "g_node is not initialized.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  std::shared_ptr<CachedActionClient> cached_client;
  {
    std::lock_guard<std::mutex> lock(g_cached_action_clients_mutex);
    auto it = g_cached_action_clients.find(target_action_name);
    if (it == g_cached_action_clients.end()) {
      cached_client = std::make_shared<CachedActionClient>();
      cached_client->client = rcl_action_get_zero_initialized_client();
      g_cached_action_clients[target_action_name] = cached_client;
    } else {
      cached_client = it->second;
    }
  }

  std::lock_guard<std::mutex> client_lock(cached_client->mutex);
  auto* rcl_node = g_node->get_node_base_interface()->get_rcl_node_handle();

  if (!cached_client->is_initialized) {
    RCLCPP_INFO(GetGatewayLogger(), "Initializing rcl action client on '%s'",
                target_action_name.c_str());
    rcl_action_client_options_t client_ops =
        rcl_action_client_get_default_options();
    rcl_ret_t ret =
        rcl_action_client_init(&cached_client->client, rcl_node, action_ts,
                               target_action_name.c_str(), &client_ops);
    if (ret != RCL_RET_OK) {
      error_msg = "Failed to initialize rcl action client on " +
                  target_action_name + " (error code: " + std::to_string(ret) +
                  ")";
      RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
      return false;
    }
    cached_client->is_initialized = true;
  }

  bool is_available = false;
  rcl_ret_t ret = rcl_action_server_is_available(
      rcl_node, &cached_client->client, &is_available);
  if (ret != RCL_RET_OK || !is_available) {
    RCLCPP_INFO(GetGatewayLogger(),
                "Waiting for action server '%s' to become available...",
                target_action_name.c_str());
    const auto start_wait = std::chrono::steady_clock::now();
    const auto wait_timeout = std::chrono::seconds(5);
    bool server_ready = false;
    while ((std::chrono::steady_clock::now() - start_wait) < wait_timeout &&
           rclcpp::ok()) {
      ret = rcl_action_server_is_available(rcl_node, &cached_client->client,
                                           &is_available);
      if (ret == RCL_RET_OK && is_available) {
        server_ready = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!server_ready) {
      error_msg = "Remote target action server '" + target_action_name +
                  "' offline or unreachable after 5s timeout.";
      RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
      return false;
    }
  }

  const size_t goal_offset =
      ((16 + goal_layout.max_alignment - 1) / goal_layout.max_alignment) *
      goal_layout.max_alignment;
  const size_t total_goal_req_size = goal_offset + goal_layout.total_size;
  std::vector<uint8_t> send_goal_req_buf(
      std::max<size_t>(16, total_goal_req_size), 0);

  uint8_t goal_uuid[16];
  GenerateRandomUuid(goal_uuid);
  std::memcpy(send_goal_req_buf.data(), goal_uuid, 16);

  std::vector<std::unique_ptr<uint8_t[]>> allocations;
  MapProtobufToCStruct(dynamic_request, goal_layout,
                       send_goal_req_buf.data() + goal_offset, allocations,
                       type_desc);

  int64_t goal_seq_id = 0;
  ret = rcl_action_send_goal_request(&cached_client->client,
                                     send_goal_req_buf.data(), &goal_seq_id);
  if (ret != RCL_RET_OK) {
    error_msg = "Failed to send action goal request (error code: " +
                std::to_string(ret) + ")";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  size_t num_subscriptions = 0;
  size_t num_guard_conditions = 0;
  size_t num_timers = 0;
  size_t num_clients = 0;
  size_t num_services = 0;
  size_t num_events = 0;
  ret = rcl_action_client_wait_set_get_num_entities(
      &cached_client->client, &num_subscriptions, &num_guard_conditions,
      &num_timers, &num_clients, &num_services);
  ret = rcl_wait_set_init(
      &wait_set, num_subscriptions, num_guard_conditions, num_timers,
      num_clients, num_services, num_events,
      rclcpp::contexts::get_global_default_context()->get_rcl_context().get(),
      rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    error_msg = "Failed to initialize wait set for action.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }
  auto cleanup_wait_set = rcpputils::make_scope_exit([&wait_set]() {
    rcl_ret_t r = rcl_wait_set_fini(&wait_set);
    (void)r;
  });

  bool goal_resp_ready = false;
  const auto start_wait_goal_resp = std::chrono::steady_clock::now();
  const auto goal_resp_timeout = std::chrono::seconds(5);

  while ((std::chrono::steady_clock::now() - start_wait_goal_resp) <
             goal_resp_timeout &&
         rclcpp::ok()) {
    ret = rcl_wait_set_clear(&wait_set);
    ret = rcl_action_wait_set_add_action_client(
        &wait_set, &cached_client->client, nullptr, nullptr);
    constexpr int64_t kTimeoutNs = 50000000;  // 50 ms
    ret = rcl_wait(&wait_set, kTimeoutNs);
    if (ret == RCL_RET_TIMEOUT) continue;
    if (ret != RCL_RET_OK) break;

    bool is_feedback_ready = false;
    bool is_status_ready = false;
    bool is_goal_resp = false;
    bool is_cancel_resp_ready = false;
    bool is_result_resp_ready = false;
    ret = rcl_action_client_wait_set_get_entities_ready(
        &wait_set, &cached_client->client, &is_feedback_ready, &is_status_ready,
        &is_goal_resp, &is_cancel_resp_ready, &is_result_resp_ready);
    if (ret == RCL_RET_OK && is_goal_resp) {
      goal_resp_ready = true;
      break;
    }
  }

  if (!goal_resp_ready) {
    error_msg = "Timed out waiting for action goal acceptance response.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  std::vector<uint8_t> send_goal_resp_buf(32, 0);
  rmw_request_id_t goal_resp_header;
  ret = rcl_action_take_goal_response(&cached_client->client, &goal_resp_header,
                                      send_goal_resp_buf.data());
  if (ret != RCL_RET_OK) {
    error_msg = "Failed to take goal response.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  bool accepted = false;
  std::memcpy(&accepted, send_goal_resp_buf.data(), sizeof(bool));
  if (!accepted) {
    error_msg = "Goal was rejected by action server.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  std::vector<uint8_t> get_result_req_buf(16, 0);
  std::memcpy(get_result_req_buf.data(), goal_uuid, 16);

  int64_t result_seq_id = 0;
  ret = rcl_action_send_result_request(
      &cached_client->client, get_result_req_buf.data(), &result_seq_id);
  if (ret != RCL_RET_OK) {
    error_msg = "Failed to send action result request.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  bool result_resp_ready = false;
  const auto start_wait_result = std::chrono::steady_clock::now();
  const auto result_timeout = std::chrono::seconds(60);

  while ((std::chrono::steady_clock::now() - start_wait_result) <
             result_timeout &&
         rclcpp::ok()) {
    ret = rcl_wait_set_clear(&wait_set);
    ret = rcl_action_wait_set_add_action_client(
        &wait_set, &cached_client->client, nullptr, nullptr);
    constexpr int64_t kTimeoutNs = 50000000;  // 50 ms
    ret = rcl_wait(&wait_set, kTimeoutNs);
    if (ret == RCL_RET_TIMEOUT) continue;
    if (ret != RCL_RET_OK) break;

    bool is_feedback_ready = false;
    bool is_status_ready = false;
    bool is_goal_resp_ready = false;
    bool is_cancel_resp_ready = false;
    bool is_res_ready = false;
    ret = rcl_action_client_wait_set_get_entities_ready(
        &wait_set, &cached_client->client, &is_feedback_ready, &is_status_ready,
        &is_goal_resp_ready, &is_cancel_resp_ready, &is_res_ready);
    if (ret == RCL_RET_OK && is_res_ready) {
      result_resp_ready = true;
      break;
    }
  }

  if (!result_resp_ready) {
    error_msg = "Timed out waiting for action result completion.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  const size_t res_offset =
      ((1 + res_layout.max_alignment - 1) / res_layout.max_alignment) *
      res_layout.max_alignment;
  const size_t total_res_size = res_offset + res_layout.total_size;
  std::vector<uint8_t> get_result_resp_buf(std::max<size_t>(8, total_res_size),
                                           0);

  rmw_request_id_t result_resp_header;
  ret = rcl_action_take_result_response(
      &cached_client->client, &result_resp_header, get_result_resp_buf.data());
  if (ret != RCL_RET_OK) {
    error_msg = "Failed to take action result response.";
    RCLCPP_ERROR(GetGatewayLogger(), "%s", error_msg.c_str());
    return false;
  }

  int8_t action_status = 0;
  std::memcpy(&action_status, get_result_resp_buf.data(), sizeof(int8_t));

  MapCStructToProtobuf(res_layout, get_result_resp_buf.data() + res_offset,
                       dynamic_response, type_desc);

  std::string status_str = "SUCCEEDED";
  if (action_status == 4) {
    status_str = "SUCCEEDED";
  } else if (action_status == 5) {
    status_str = "CANCELED";
  } else if (action_status == 6) {
    status_str = "ABORTED";
  } else {
    status_str = "STATUS_" + std::to_string(action_status);
  }

  RCLCPP_DEBUG(GetGatewayLogger(),
               "Action '%s' completed with status %s (code=%d)",
               target_action_name.c_str(), status_str.c_str(),
               static_cast<int>(action_status));

  const auto* resp_reflection = dynamic_response.GetReflection();
  const auto* resp_descriptor = dynamic_response.GetDescriptor();
  const auto* status_field =
      resp_descriptor->FindFieldByName("aquarium_skill_status");
  if (status_field != nullptr) {
    resp_reflection->SetString(&dynamic_response, status_field, status_str);
  }

  return (action_status == 4);
}

void HandleZenohQuery(const zenoh::Query& query,
                      const ServiceConfig& srv_config,
                      google::protobuf::DescriptorPool& pool,
                      google::protobuf::DynamicMessageFactory& factory) {
  const std::string key_str(query.get_keyexpr().as_string_view());
  RCLCPP_DEBUG(GetGatewayLogger(), "Zenoh query received on '%s'",
               key_str.c_str());

  auto payload_opt = query.get_payload();
  if (!payload_opt.has_value()) {
    RCLCPP_ERROR(GetGatewayLogger(), "Zenoh query on '%s' has no payload.",
                 key_str.c_str());
    return;
  }

  const std::string proto_payload = payload_opt.value().get().as_string();

  std::string package;
  std::string base_name;
  if (!parse_ros_type(srv_config.internal_type, package, base_name)) {
    RCLCPP_ERROR(GetGatewayLogger(), "Failed to parse ROS type: %s",
                 srv_config.internal_type.c_str());
    return;
  }

  std::string proto_package;
  std::string req_message_name;
  std::string resp_message_name;
  ResolveMessageSchemaNames(srv_config.interface_type, package, base_name,
                            srv_config.external_interface, proto_package,
                            req_message_name, resp_message_name);

  const google::protobuf::Descriptor* req_desc =
      pool.FindMessageTypeByName(proto_package + "." + req_message_name);
  const google::protobuf::Descriptor* resp_desc =
      pool.FindMessageTypeByName(proto_package + "." + resp_message_name);
  if (req_desc == nullptr || resp_desc == nullptr) {
    RCLCPP_ERROR(GetGatewayLogger(),
                 "Unresolvable dynamic message descriptors for %s.%s / %s",
                 proto_package.c_str(), req_message_name.c_str(),
                 resp_message_name.c_str());
    return;
  }

  const google::protobuf::Message* req_proto = factory.GetPrototype(req_desc);
  const google::protobuf::Message* resp_proto = factory.GetPrototype(resp_desc);
  if (req_proto == nullptr || resp_proto == nullptr) {
    RCLCPP_ERROR(GetGatewayLogger(),
                 "Failed to obtain dynamic message prototypes.");
    return;
  }

  std::unique_ptr<google::protobuf::Message> dynamic_request(req_proto->New());
  if (!dynamic_request->ParseFromString(proto_payload)) {
    RCLCPP_ERROR(GetGatewayLogger(),
                 "Failed to deserialize Protobuf query payload on '%s'.",
                 key_str.c_str());
    return;
  }

  RCLCPP_DEBUG(GetGatewayLogger(), "Query request [%s]: %s",
               std::string(req_desc->name()).c_str(),
               dynamic_request->ShortDebugString().c_str());

  std::unique_ptr<google::protobuf::Message> dynamic_response(
      resp_proto->New());

  if (srv_config.interface_type == InterfaceType::ROS_SERVICE) {
    std::string error_msg;
    const bool ok = DynamicTransactionDispatcher(
        srv_config.internal_name, srv_config.internal_type,
        srv_config.type_desc, *dynamic_request, *dynamic_response, error_msg);
    SetResponseStatusFields(dynamic_response.get(), ok, error_msg,
                            /*preserve_existing_skill_status=*/false);
  } else if (srv_config.interface_type == InterfaceType::ROS_ACTION_CLIENT) {
    std::string target_action_name = srv_config.internal_name;
    const google::protobuf::FieldDescriptor* action_name_field =
        req_desc->FindFieldByName("action_name");
    if (action_name_field != nullptr) {
      const std::string override_name =
          dynamic_request->GetReflection()->GetString(*dynamic_request,
                                                      action_name_field);
      if (!override_name.empty()) {
        target_action_name = override_name;
      }
    }

    std::string error_msg;
    const bool ok = DynamicActionTransactionDispatcher(
        target_action_name, srv_config.internal_type, srv_config.type_desc,
        *dynamic_request, *dynamic_response, error_msg);
    SetResponseStatusFields(dynamic_response.get(), ok, error_msg,
                            /*preserve_existing_skill_status=*/true);
  } else if (srv_config.interface_type ==
             InterfaceType::ROS_TOPIC_SUBSCRIPTION) {
    bool wait_for_next_message = false;
    double timeout_seconds = 0.0;

    if (srv_config.external_interface == "zenoh_queryable_proto") {
      const google::protobuf::Reflection* req_reflection =
          dynamic_request->GetReflection();
      const google::protobuf::FieldDescriptor* wait_field =
          req_desc->FindFieldByName("wait_for_next_message");
      const google::protobuf::FieldDescriptor* timeout_field =
          req_desc->FindFieldByName("timeout_seconds");
      if (wait_field != nullptr) {
        wait_for_next_message =
            req_reflection->GetBool(*dynamic_request, wait_field);
      }
      if (timeout_field != nullptr) {
        timeout_seconds =
            req_reflection->GetDouble(*dynamic_request, timeout_field);
      }
    }

    std::shared_ptr<SubscriptionCache> cache;
    {
      std::lock_guard<std::mutex> caches_lock(g_caches_mutex);
      auto it = g_subscription_caches.find(srv_config.internal_name);
      if (it != g_subscription_caches.end()) {
        cache = it->second;
      }
    }

    if (cache == nullptr) {
      RCLCPP_ERROR(GetGatewayLogger(), "Subscription cache not found for %s",
                   srv_config.internal_name.c_str());
      return;
    }

    std::unique_lock<std::mutex> lock(cache->mutex);
    bool success = true;

    if (wait_for_next_message) {
      const rclcpp::Time query_start_time = g_node->now();
      auto predicate = [cache, query_start_time]() {
        return cache->has_data && cache->last_publish_time > query_start_time;
      };
      if (timeout_seconds > 0.0) {
        auto timeout = std::chrono::duration<double>(timeout_seconds);
        success = cache->cv.wait_for(lock, timeout, predicate);
      } else {
        cache->cv.wait(lock, predicate);
      }
    }

    std::string status = "SUCCEEDED";
    if (!success) {
      status = "TIMEOUT";
    } else if (!cache->has_data) {
      status = "NO_DATA";
    }

    if (status == "SUCCEEDED") {
      const auto* msg_type_desc =
          FindIndividualType(srv_config.type_desc, "_Response");
      if (msg_type_desc != nullptr) {
        MapCStructToProtobuf(cache->layout, cache->cached_data.data(),
                             *dynamic_response, srv_config.type_desc);
      }
    }

    const google::protobuf::Reflection* resp_reflection =
        dynamic_response->GetReflection();
    const google::protobuf::FieldDescriptor* status_field =
        resp_desc->FindFieldByName("aquarium_skill_status");
    if (status_field != nullptr) {
      resp_reflection->SetString(dynamic_response.get(), status_field, status);
    }
  }

  std::string response_bytes;
  if (!dynamic_response->SerializeToString(&response_bytes)) {
    RCLCPP_ERROR(GetGatewayLogger(),
                 "Failed to serialize Protobuf response on '%s'.",
                 key_str.c_str());
    return;
  }

  RCLCPP_DEBUG(GetGatewayLogger(), "Replying on '%s' (%zu bytes)",
               key_str.c_str(), response_bytes.size());

  zenoh::Bytes res_bytes(response_bytes);
  zenoh::ZResult result;
  query.reply(query.get_keyexpr(), std::move(res_bytes),
              zenoh::Query::ReplyOptions::create_default(), &result);
  if (result != Z_OK) {
    RCLCPP_ERROR(GetGatewayLogger(),
                 "Failed to reply to Zenoh query on '%s': %d", key_str.c_str(),
                 result);
  }
}

void RunZenohServer(google::protobuf::DescriptorPool& pool,
                    google::protobuf::DynamicMessageFactory& factory) {
  std::string connect_endpoint = "tcp/127.0.0.1:7447";
  if (IsRunningInCluster()) {
    connect_endpoint =
        "tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447";
  }

  zenoh::ZResult result;
  auto config = zenoh::Config::create_default(&result);
  if (result == Z_OK) {
    config.insert_json5("connect/endpoints", "[\"" + connect_endpoint + "\"]",
                        &result);
  }
  if (result == Z_OK) {
    config.insert_json5("mode", "\"peer\"", &result);
  }
  if (result == Z_OK) {
    config.insert_json5("scouting/multicast/enabled", "false", &result);
  }
  if (result == Z_OK) {
    const char* env_override = std::getenv("ZENOH_CONFIG_OVERRIDE");
    if (env_override != nullptr && env_override[0] != '\0') {
      std::stringstream ss(env_override);
      std::string pair;
      while (std::getline(ss, pair, ';')) {
        std::stringstream pair_stream(pair);
        std::string key;
        std::string val;
        if (std::getline(pair_stream, key, '=') &&
            std::getline(pair_stream, val)) {
          config.insert_json5(key, val, &result);
          if (result == Z_OK &&
              (key == "connect/endpoints" || key == "connect")) {
            connect_endpoint = val;
          }
          if (result != Z_OK) {
            RCLCPP_WARN(GetGatewayLogger(),
                        "Failed to apply Zenoh config override %s=%s",
                        key.c_str(), val.c_str());
            result = Z_OK;
          }
        }
      }
    }
  }
  if (result != Z_OK) {
    RCLCPP_FATAL(GetGatewayLogger(), "Failed to configure Zenoh. Error: %d",
                 result);
    return;
  }

  RCLCPP_INFO(GetGatewayLogger(),
              "Opening Zenoh session (mode: peer, connect: %s)...",
              connect_endpoint.c_str());
  auto session = zenoh::Session::open(
      std::move(config), zenoh::Session::SessionOptions::create_default(),
      &result);
  if (result != Z_OK) {
    RCLCPP_FATAL(GetGatewayLogger(), "Failed to open Zenoh session.");
    return;
  }

  std::vector<zenoh::Queryable<void>> queryables;
  {
    std::lock_guard<std::mutex> lock(g_cache_mutex);
    for (const auto& [external_name, srv_config] : g_config_cache) {
      if (!external_name.empty() && external_name[0] == '/') {
        continue;
      }
      zenoh::ZResult q_res;
      auto q = session.declare_queryable(
          zenoh::KeyExpr(external_name, true, &q_res),
          [&pool, &factory, srv_config](const zenoh::Query& query) {
            HandleZenohQuery(query, srv_config, pool, factory);
          },
          zenoh::closures::none,
          zenoh::Session::QueryableOptions::create_default(), &q_res);
      if (q_res != Z_OK) {
        RCLCPP_ERROR(GetGatewayLogger(),
                     "Failed to declare Zenoh queryable for %s: %d",
                     external_name.c_str(), q_res);
      } else {
        queryables.push_back(std::move(q));
        RCLCPP_INFO(GetGatewayLogger(), "Mapped Zenoh queryable on key '%s'",
                    external_name.c_str());
      }
    }
  }

  RCLCPP_INFO(GetGatewayLogger(), "aquarium_gateway online");

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(GetGatewayLogger(), "Shutting down Zenoh queryables...");
  queryables.clear();
  zenoh::ZResult close_res;
  session.close(zenoh::Session::SessionCloseOptions::create_default(),
                &close_res);
}

void ShutdownGateway() {
  {
    std::lock_guard<std::mutex> caches_lock(g_caches_mutex);
    for (auto& [name, cache] : g_subscription_caches) {
      (void)name;
      if (cache != nullptr) {
        cache->cv.notify_all();
      }
    }
    g_subscription_caches.clear();
  }
  g_subscriptions.clear();

  rcl_node_t* rcl_node = nullptr;
  if (g_node != nullptr) {
    rcl_node = g_node->get_node_base_interface()->get_rcl_node_handle();
  }

  {
    std::lock_guard<std::mutex> lock(g_cached_service_clients_mutex);
    for (auto& [name, entry] : g_cached_service_clients) {
      (void)name;
      if (entry != nullptr && entry->is_initialized && rcl_node != nullptr) {
        rcl_ret_t ret = rcl_client_fini(&entry->client, rcl_node);
        (void)ret;
        entry->is_initialized = false;
      }
    }
    g_cached_service_clients.clear();
  }

  {
    std::lock_guard<std::mutex> lock(g_cached_action_clients_mutex);
    for (auto& [name, entry] : g_cached_action_clients) {
      (void)name;
      if (entry != nullptr && entry->is_initialized && rcl_node != nullptr) {
        rcl_ret_t ret = rcl_action_client_fini(&entry->client, rcl_node);
        (void)ret;
        entry->is_initialized = false;
      }
    }
    g_cached_action_clients.clear();
  }

  {
    std::lock_guard<std::mutex> lock(g_cache_mutex);
    g_config_cache.clear();
  }

  g_node.reset();
}

void setup_subscriptions(std::shared_ptr<rclcpp::Node> node) {
  std::lock_guard<std::mutex> lock(g_cache_mutex);
  for (const auto& [external_name, config] : g_config_cache) {
    (void)external_name;
    if (config.interface_type != InterfaceType::ROS_TOPIC_SUBSCRIPTION) {
      continue;
    }

    auto cache = std::make_shared<SubscriptionCache>();
    const auto* msg_type_desc =
        FindIndividualType(config.type_desc, "_Response");
    if (msg_type_desc == nullptr) {
      RCLCPP_ERROR(GetGatewayLogger(),
                   "Subscription setup error: missing response type desc for "
                   "%s",
                   config.internal_name.c_str());
      continue;
    }
    cache->layout = ComputeDynamicLayout(*msg_type_desc, config.type_desc);
    cache->cached_data.resize(cache->layout.total_size, 0);

    {
      std::lock_guard<std::mutex> caches_lock(g_caches_mutex);
      g_subscription_caches[config.internal_name] = cache;
    }

    auto weak_sub_holder =
        std::make_shared<std::weak_ptr<rclcpp::GenericSubscription>>();

    auto callback =
        [cache, weak_sub_holder](
            std::shared_ptr<const rclcpp::SerializedMessage> serialized_msg) {
          auto sub = weak_sub_holder->lock();
          if (!sub) return;
          const rosidl_message_type_support_t* ts_handle =
              &sub->get_message_type_support_handle();

          std::lock_guard<std::mutex> cache_lock(cache->mutex);
          const rcl_serialized_message_t& rcl_serialized_msg =
              serialized_msg->get_rcl_serialized_message();
          rmw_ret_t ret =
              rmw_deserialize(&rcl_serialized_msg, ts_handle,
                              static_cast<void*>(cache->cached_data.data()));
          if (ret == RMW_RET_OK) {
            cache->has_data = true;
            cache->last_publish_time = g_node->now();
            cache->cv.notify_all();
          } else {
            RCLCPP_ERROR(GetGatewayLogger(),
                         "Failed to deserialize subscribed message: %d", ret);
          }
        };

    auto generic_sub = node->create_generic_subscription(
        config.internal_name, config.internal_type, rclcpp::QoS(1), callback);
    *weak_sub_holder = generic_sub;
    g_subscriptions.push_back(generic_sub);

    RCLCPP_INFO(GetGatewayLogger(), "Created subscription on %s (%s)",
                config.internal_name.c_str(), config.internal_type.c_str());
  }
}

namespace {

class GenericReactor : public grpc::ServerGenericBidiReactor {
 public:
  GenericReactor(grpc::GenericCallbackServerContext* context,
                 google::protobuf::DescriptorPool& pool,
                 google::protobuf::DynamicMessageFactory& factory)
      : context_(context), pool_(pool), factory_(factory) {
    StartRead(&request_buf_);
  }

  void OnReadDone(bool ok) override {
    if (!ok) {
      Finish(grpc::Status(grpc::StatusCode::INTERNAL, "Read failed"));
      return;
    }
    ProcessRequest();
  }

  void OnWriteDone(bool ok) override {
    if (!ok) {
      Finish(grpc::Status(grpc::StatusCode::INTERNAL, "Write failed"));
      return;
    }
    Finish(grpc::Status::OK);
  }

  void OnDone() override { delete this; }

 private:
  void WriteSerializedResponse(const google::protobuf::Message& msg) {
    std::string serialized_resp;
    if (!msg.SerializeToString(&serialized_resp)) {
      Finish(grpc::Status(grpc::StatusCode::INTERNAL,
                          "Failed to serialize protobuf response"));
      return;
    }
    grpc::Slice response_slice(serialized_resp.data(), serialized_resp.size());
    response_buf_ = grpc::ByteBuffer(&response_slice, 1);
    StartWrite(&response_buf_);
  }

  void ProcessRequest() {
    const std::string method = context_->method();
    const std::string method_no_slash =
        (!method.empty() && method[0] == '/') ? method.substr(1) : method;

    RCLCPP_DEBUG(GetGatewayLogger(), "gRPC request received: method='%s'",
                 method.c_str());

    if ((method_no_slash == "intrinsic_proto.resources.ResourceRegistry/"
                            "ListResourceInstances" ||
         method_no_slash ==
             "intrinsic_proto.resources.ResourceRegistryInternal/"
             "ListResourceInstances") &&
        !g_resource_name.empty()) {
      const google::protobuf::Descriptor* resp_desc =
          pool_.FindMessageTypeByName(
              "intrinsic_proto.resources.ListResourceInstanceResponse");
      if (resp_desc != nullptr) {
        const google::protobuf::Message* resp_proto =
            factory_.GetPrototype(resp_desc);
        std::unique_ptr<google::protobuf::Message> response(resp_proto->New());

        const auto* refl = response->GetReflection();
        const auto* instances_field = resp_desc->FindFieldByName("instances");
        google::protobuf::Message* instance =
            refl->AddMessage(response.get(), instances_field);

        const std::string addr = "127.0.0.1:" + std::to_string(g_grpc_port);
        PopulateResourceInstanceProto(instance, g_resource_name, addr);

        RCLCPP_DEBUG(GetGatewayLogger(),
                     "ListResourceInstances returning '%s' at '%s'",
                     g_resource_name.c_str(), addr.c_str());
        WriteSerializedResponse(*response);
        return;
      }
    }

    if ((method_no_slash ==
             "intrinsic_proto.resources.ResourceRegistry/GetResourceInstance" ||
         method_no_slash ==
             "intrinsic_proto.resources.ResourceRegistryInternal/"
             "GetResourceInstance") &&
        !g_resource_name.empty()) {
      const google::protobuf::Descriptor* inst_desc =
          pool_.FindMessageTypeByName(
              "intrinsic_proto.resources.ResourceInstance");
      if (inst_desc != nullptr) {
        const google::protobuf::Message* inst_proto =
            factory_.GetPrototype(inst_desc);
        std::unique_ptr<google::protobuf::Message> instance(inst_proto->New());

        const std::string addr = "127.0.0.1:" + std::to_string(g_grpc_port);
        PopulateResourceInstanceProto(instance.get(), g_resource_name, addr);

        RCLCPP_DEBUG(GetGatewayLogger(),
                     "GetResourceInstance returning '%s' at '%s'",
                     g_resource_name.c_str(), addr.c_str());
        WriteSerializedResponse(*instance);
        return;
      }
    }

    ServiceConfig srv_config;
    bool found = false;
    {
      std::lock_guard<std::mutex> lock(g_cache_mutex);
      auto it = g_config_cache.find(method);
      if (it == g_config_cache.end()) {
        it = g_config_cache.find(method_no_slash);
      }
      if (it != g_config_cache.end()) {
        srv_config = it->second;
        found = true;
      }
    }

    if (!found) {
      RCLCPP_WARN(GetGatewayLogger(),
                  "No service config found for gRPC method: %s",
                  method.c_str());
      Finish(grpc::Status(grpc::StatusCode::NOT_FOUND,
                          "Method not mapped in gateway config"));
      return;
    }

    std::vector<grpc::Slice> slices;
    auto status = request_buf_.Dump(&slices);
    if (!status.ok()) {
      Finish(grpc::Status(grpc::StatusCode::INTERNAL,
                          "Failed to dump request buffer"));
      return;
    }
    std::string serialized_request;
    for (const auto& slice : slices) {
      serialized_request.append(reinterpret_cast<const char*>(slice.begin()),
                                slice.size());
    }

    std::string package;
    std::string base_name;
    if (!parse_ros_type(srv_config.internal_type, package, base_name)) {
      Finish(
          grpc::Status(grpc::StatusCode::INTERNAL, "Failed to parse ROS type"));
      return;
    }

    std::string proto_package;
    std::string req_message_name;
    std::string resp_message_name;
    ResolveMessageSchemaNames(srv_config.interface_type, package, base_name,
                              srv_config.external_interface, proto_package,
                              req_message_name, resp_message_name);

    const google::protobuf::Descriptor* req_desc =
        pool_.FindMessageTypeByName(proto_package + "." + req_message_name);
    const google::protobuf::Descriptor* resp_desc =
        pool_.FindMessageTypeByName(proto_package + "." + resp_message_name);
    if (req_desc == nullptr || resp_desc == nullptr) {
      Finish(grpc::Status(grpc::StatusCode::INTERNAL,
                          "Failed to resolve descriptors"));
      return;
    }

    const google::protobuf::Message* req_proto =
        factory_.GetPrototype(req_desc);
    std::unique_ptr<google::protobuf::Message> dynamic_request(
        req_proto->New());
    if (!dynamic_request->ParseFromString(serialized_request)) {
      Finish(grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                          "Failed to parse protobuf request"));
      return;
    }

    RCLCPP_DEBUG(GetGatewayLogger(), "gRPC request body [%s]: %s",
                 srv_config.internal_name.c_str(),
                 dynamic_request->ShortDebugString().c_str());

    const google::protobuf::Message* resp_proto =
        factory_.GetPrototype(resp_desc);
    std::unique_ptr<google::protobuf::Message> dynamic_response(
        resp_proto->New());

    if (srv_config.interface_type == InterfaceType::ROS_SERVICE) {
      std::string error_msg;
      const bool ok = DynamicTransactionDispatcher(
          srv_config.internal_name, srv_config.internal_type,
          srv_config.type_desc, *dynamic_request, *dynamic_response, error_msg);
      SetResponseStatusFields(dynamic_response.get(), ok, error_msg,
                              /*preserve_existing_skill_status=*/false);
    } else if (srv_config.interface_type == InterfaceType::ROS_ACTION_CLIENT) {
      std::string error_msg;
      const bool ok = DynamicActionTransactionDispatcher(
          srv_config.internal_name, srv_config.internal_type,
          srv_config.type_desc, *dynamic_request, *dynamic_response, error_msg);
      SetResponseStatusFields(dynamic_response.get(), ok, error_msg,
                              /*preserve_existing_skill_status=*/true);
    } else {
      Finish(grpc::Status(grpc::StatusCode::UNIMPLEMENTED,
                          "Interface type not supported via gRPC currently"));
      return;
    }

    RCLCPP_DEBUG(GetGatewayLogger(), "gRPC response body [%s]: %s",
                 srv_config.internal_name.c_str(),
                 dynamic_response->ShortDebugString().c_str());
    WriteSerializedResponse(*dynamic_response);
  }

  grpc::GenericCallbackServerContext* context_;
  google::protobuf::DescriptorPool& pool_;
  google::protobuf::DynamicMessageFactory& factory_;
  grpc::ByteBuffer request_buf_;
  grpc::ByteBuffer response_buf_;
};

class GenericServiceImpl : public grpc::CallbackGenericService {
 public:
  GenericServiceImpl(google::protobuf::DescriptorPool& pool,
                     google::protobuf::DynamicMessageFactory& factory)
      : pool_(pool), factory_(factory) {}

  grpc::ServerGenericBidiReactor* CreateReactor(
      grpc::GenericCallbackServerContext* context) override {
    return new GenericReactor(context, pool_, factory_);
  }

 private:
  google::protobuf::DescriptorPool& pool_;
  google::protobuf::DynamicMessageFactory& factory_;
};

}  // namespace

void RunGrpcServer(google::protobuf::DescriptorPool& pool,
                   google::protobuf::DynamicMessageFactory& factory) {
  const int port = g_node->get_parameter("grpc_port").as_int();
  const std::string server_address = "0.0.0.0:" + std::to_string(port);
  GenericServiceImpl service(pool, factory);

  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterCallbackGenericService(&service);

  g_grpc_server = builder.BuildAndStart();
  RCLCPP_INFO(GetGatewayLogger(), "gRPC server listening on %s",
              server_address.c_str());
  g_grpc_server->Wait();
  RCLCPP_INFO(GetGatewayLogger(), "gRPC server shut down.");
}

}  // namespace aquarium_gateway
