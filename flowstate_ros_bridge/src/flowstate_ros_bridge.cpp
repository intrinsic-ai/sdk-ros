// Copyright 2025 Intrinsic Innovation LLC
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

#include "flowstate_ros_bridge.hpp"

#include "absl/flags/flag.h"
#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "intrinsic/platform/pubsub/zenoh_util/zenoh_config.h"

namespace flowstate_ros_bridge {
constexpr const char* kExecutiveAddressParamName = "executive_service_address";
constexpr const char* kSkillAddressParamName = "skill_registry_address";
constexpr const char* kSolutionAddressParamName = "solution_service_address";
constexpr const char* kAutostartParamName = "autostart";
constexpr const char* kFlowstateZenohRouterParamName =
    "flowstate_zenoh_router_address";
constexpr const char* kWorldAddressParamName = "world_service_address";
constexpr const char* kGeometryAddressParamName = "geometry_service_address";
constexpr const char* kServiceTunnelParamName = "service_tunnel";
constexpr const char* kBridgePluginParamName = "bridge_plugins";
constexpr const char* kExecutiveDeadlineParamName =
    "executive_deadline_seconds";
constexpr const char* kExecutiveUpdateRateMillisParamName =
    "executive_update_rate_millis";

///=============================================================================
FlowstateROSBridge::FlowstateROSBridge(const rclcpp::NodeOptions& options)
    : LifecycleNode("flowstate_ros_bridge", options),
      bridge_loader_("flowstate_ros_bridge",
                     "flowstate_ros_bridge::BridgeInterface") {
  // Declare ROS parameters needed by the flowstate_ros_bridge.
  this->declare_parameter(kServiceTunnelParamName, "");
  const std::string& service_tunnel =
      this->get_parameter(kServiceTunnelParamName).get_value<std::string>();

  this->declare_parameter(
      kExecutiveAddressParamName,
      service_tunnel.empty()
          ? "executive.app-intrinsic-app-chart.svc.cluster.local:8080"
          : service_tunnel);
  this->declare_parameter(kExecutiveDeadlineParamName, 5);
  this->declare_parameter(kExecutiveUpdateRateMillisParamName, 1000);

  this->declare_parameter(
      kSkillAddressParamName,
      service_tunnel.empty()
          ? "skill-registry.app-intrinsic-app-chart.svc.cluster.local:8080"
          : service_tunnel);

  this->declare_parameter(
      kSolutionAddressParamName,
      service_tunnel.empty()
          ? "frontend.app-intrinsic-base.svc.cluster.local:8082"
          : service_tunnel);

  this->declare_parameter(
      kWorldAddressParamName,
      service_tunnel.empty() ? "world.app-intrinsic-base.svc.cluster.local:8080"
                             : service_tunnel);

  this->declare_parameter(
      kGeometryAddressParamName,
      service_tunnel.empty()
          ? "geomservice.app-intrinsic-base.svc.cluster.local:8080"
          : service_tunnel);

  this->declare_parameter(kBridgePluginParamName, std::vector<std::string>{});

  const bool autostart = this->declare_parameter(kAutostartParamName, true);
  this->declare_parameter(kFlowstateZenohRouterParamName,
                          "tcp/localhost:17447");

  absl::SetFlag(&FLAGS_zenoh_router,
                this->get_parameter(kFlowstateZenohRouterParamName)
                    .get_value<std::string>());
  this->pubsub_ = std::make_shared<intrinsic::PubSub>(this->get_name());

  // Initialize the executive client.
  this->executive_ = std::make_shared<Executive>(
      this->get_parameter(kExecutiveAddressParamName).get_value<std::string>(),
      this->get_parameter(kSkillAddressParamName).get_value<std::string>(),
      this->get_parameter(kSolutionAddressParamName).get_value<std::string>(),
      this->get_parameter(kExecutiveDeadlineParamName).get_value<std::size_t>(),
      this->get_parameter(kExecutiveUpdateRateMillisParamName)
          .get_value<std::size_t>());

  // Initialize the world client.
  this->world_ = std::make_shared<World>(
      this->pubsub_,
      this->get_parameter(kWorldAddressParamName).get_value<std::string>(),
      this->get_parameter(kGeometryAddressParamName).get_value<std::string>());

  bridge_ids_ = this->get_parameter(kBridgePluginParamName).as_string_array();

  for (const auto& id : bridge_ids_) {
    try {
      auto bridge = bridge_loader_.createUniqueInstance(id);
      bridge->declare_ros_parameters(*this);
      bridges_.push_back(std::move(bridge));
    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to load bridge_plugin [" << id
                 << "]. Detailed error: " << e.what();
      continue;
    }
  }

  if (autostart) {
    this->configure();
    this->activate();
  }
}

///=============================================================================
auto FlowstateROSBridge::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) -> CallbackReturn {
  // Configure client services.
  if (!this->executive_->connect().ok()) {
    return CallbackReturn::FAILURE;
  }
  if (!this->world_->connect().ok()) {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

///=============================================================================
auto FlowstateROSBridge::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) -> CallbackReturn {
  // Initialize/start the various bridges.
  std::vector<pluginlib::UniquePtr<BridgeInterface>>::iterator iter;
  for (iter = bridges_.begin(); iter != bridges_.end(); ++iter) {
    try {
      if (!(*iter)->initialize(*this, this->executive_, this->world_)) {
        return CallbackReturn::FAILURE;
      }
    } catch (const std::exception& e) {
      LOG(ERROR) << "Error initializing bridges: " << e.what();
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}

///=============================================================================
auto FlowstateROSBridge::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) -> CallbackReturn {
  // TODO(Yadunund): Should bridges have a "deactivate" api?
  return CallbackReturn::SUCCESS;
}

///=============================================================================
auto FlowstateROSBridge::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) -> CallbackReturn {
  bridges_.clear();
  executive_.reset();
  world_.reset();
  LOG(INFO) << "Closing PubSub session...";
  pubsub_.reset();
  LOG(INFO) << "Done.";
  return CallbackReturn::SUCCESS;
}
}  // namespace flowstate_ros_bridge

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(flowstate_ros_bridge::FlowstateROSBridge)
