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

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <memory>

#include "google/protobuf/util/json_util.h"

#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "flowstate_ros_gz_bridge_config.pb.h"
#include "ros_gz_bridge/ros_gz_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sim_connection.h"


class FlowstateRosGzBridge : public ros_gz_bridge::RosGzBridge {
 public:
  using SimConnection = intrinsic::simulation::SimConnection;
  // Constructor
  explicit FlowstateRosGzBridge(
      std::optional<intrinsic_proto::assets::v1::ResolvedDependency> resolved_deps,
      const std::string& simulation_server_address,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : ros_gz_bridge::RosGzBridge(options) {
      absl::StatusOr<std::shared_ptr<SimConnection>> res;
      if (resolved_deps.has_value()) {
        res = SimConnection::CreateFromResolvedDependency(*resolved_deps);
      } else {
        res = SimConnection::Create(simulation_server_address);
      }
      if (res.ok())
      {
        this->sim_conn_ = *res;
        // Set the gz::transport::Node for the ros gz bridge to the simulation connection node
        this->gz_node_ = this->sim_conn_->Node();
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed initializing Simulation Connection");
        // TODO(luca) a make function that returns a pointer instead so we can fail
        return;
      }
    }
 private:
  std::shared_ptr<SimConnection> sim_conn_;
};

flowstate::RosGzBridgeConfig MakeTestConfig() {
  flowstate::RosGzBridgeConfig config;
  auto* bridge = config.add_bridges();
  bridge->set_ros_topic_name("/clock");
  bridge->set_gz_topic_name("/clock");
  bridge->set_ros_type_name("rosgraph_msgs/msg/Clock");
  bridge->set_gz_type_name("gz.msgs.Clock");
  bridge->set_lazy(false);
  bridge->set_direction(flowstate::RosGzBridgeConfig::GZ_TO_ROS);
  bridge->set_qos_profile(flowstate::RosGzBridgeConfig::CLOCK);
  return config;
}

intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    // Return default context for running locally
    std::cerr << "Warning: using default RuntimeContext\n";
    flowstate::RosGzBridgeServiceConfig default_service_config;
    *default_service_config.mutable_ros_gz_bridge_config() = MakeTestConfig();
    runtime_context.mutable_config()->PackFrom(default_service_config);
  }
  return runtime_context;
}

int main(int , char**) {
  auto runtime_context = GetRuntimeContext();
  flowstate::RosGzBridgeServiceConfig service_config;
  if (!runtime_context.config().UnpackTo(&service_config)) {
    std::cerr << "Unable to unpack runtime_context\n";
    return EXIT_FAILURE;
  }

  // Parse the bridge config to json
  std::string json_config;
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  const auto status = google::protobuf::util::MessageToJsonString(service_config.ros_gz_bridge_config(), &json_config, options);
  if (status.ok()) {
    // A bit hacky, manually remove the top field to flatten the structure
    // and make it a repeated yaml
    std::size_t pos = json_config.find("[");
    if (pos != std::string::npos) {
      json_config.erase(0, pos);
      json_config.pop_back();
    } else {
      std::cerr << "Failed processing json, [ not found" << std::endl;
      return EXIT_FAILURE;
    }
  } else {
    std::cerr << "Failed converting to json: " << status << std::endl;
    return EXIT_FAILURE;
  }

  // We can't use the recommended tmpfile because we need to pass a file path
  char filename[] = "/tmp/ros_gz_bridge_XXXXXX";
  int fd = mkstemp(filename);

  const auto bytes_written = write(fd, json_config.c_str(), json_config.length());
  if (bytes_written == -1 || static_cast<std::size_t>(bytes_written) != json_config.length()) {
    std::cerr << "Failed writing config" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string config_file_param = std::string("config_file:=") + filename;
  // Get ROS arguments
  std::vector<const char *> ros_argv = {"--ros-args", "-p", config_file_param.c_str()};

  std::optional<intrinsic_proto::assets::v1::ResolvedDependency> resolved_deps = std::nullopt;
  if (service_config.has_gazebo_simulator()) {
    resolved_deps = service_config.gazebo_simulator();
  }

  rclcpp::init(ros_argv.size(), ros_argv.data());
  rclcpp::spin(std::make_shared<FlowstateRosGzBridge>(resolved_deps, runtime_context.simulation_server_address()));
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
