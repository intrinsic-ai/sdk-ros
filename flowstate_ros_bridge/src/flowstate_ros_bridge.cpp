// Copyright 2025 Intrinsic Innovation LLC
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

#include "flowstate_ros_bridge.hpp"

#include "absl/flags/flag.h"
#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "intrinsic/platform/pubsub/zenoh_util/zenoh_config.h"
#include <intrinsic/util/grpc/grpc.h>

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

const std::size_t kExecutiveDeadlineSeconds = 5;
const std::size_t kWorldDeadlineSeconds = 10;

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
  grpc::ChannelArguments executive_channel_args = intrinsic::DefaultGrpcChannelArgs();
  // The skill registry may need to call out to one or more skill information
  // services. Those services might not be ready at startup. We configure a
  // retry policy to mitigate b/283020857.
  // (See
  // https://github.com/grpc/grpc-go/blob/master/examples/features/retry/README.md
  //  for an example of this gRPC feature.)
  executive_channel_args.SetServiceConfigJSON(R"(
      {
        "methodConfig": [{
          "name": [{"service": "intrinsic_proto.skills.SkillRegistry"}],
          "waitForReady": true,
          "timeout": "300s",
          "retryPolicy": {
              "maxAttempts": 10,
              "initialBackoff": "0.1s",
              "maxBackoff": "10s",
              "backoffMultiplier": 1.5,
              "retryableStatusCodes": [ "UNAVAILABLE" ]
          }
        }]
      })");
  executive_channel_args.SetMaxReceiveMessageSize(10000000); // 10 MB
  executive_channel_args.SetMaxSendMessageSize(10000000);    // 10 MB

  auto executive_address = this->get_parameter(kExecutiveAddressParamName).get_value<std::string>();
  auto maybe_executive_channel = intrinsic::CreateClientChannel(
    executive_address, absl::Now() + absl::Seconds(kExecutiveDeadlineSeconds));
  if (!maybe_executive_channel.ok()) {
    LOG(ERROR) << maybe_executive_channel;
    return CallbackReturn::FAILURE;
  }
  auto executive_channel = maybe_executive_channel.value();
  LOG(INFO) << "Executive address: " << executive_address;

  auto skill_registry_address = this->get_parameter(kSkillAddressParamName).get_value<std::string>();
  auto maybe_skill_registry_channel = intrinsic::CreateClientChannel(
    skill_registry_address, absl::Now() + absl::Seconds(kExecutiveDeadlineSeconds));
  if (!maybe_skill_registry_channel.ok()) {
    LOG(ERROR) << maybe_skill_registry_channel;
    return CallbackReturn::FAILURE;
  }
  auto skill_registry_channel = maybe_skill_registry_channel.value();
  LOG(INFO) << "Skill registry address: " << skill_registry_address;

  auto solution_address = this->get_parameter(kSolutionAddressParamName).get_value<std::string>();
  auto maybe_solution_channel = intrinsic::CreateClientChannel(
    solution_address, absl::Now() + absl::Seconds(kExecutiveDeadlineSeconds));
  if (!maybe_solution_channel.ok()) {
    LOG(ERROR) << maybe_solution_channel;
    return CallbackReturn::FAILURE;
  }
  auto solution_channel = maybe_solution_channel.value();
  LOG(INFO) << "Solution service address: " << solution_address;

  // Initialize the executive client.
  this->executive_ = std::make_shared<Executive>(
    executive_channel, skill_registry_channel, solution_channel, kExecutiveDeadlineSeconds);

  grpc::ChannelArguments world_channel_args = intrinsic::DefaultGrpcChannelArgs();
  // We might eventually need a retry policy here, like in executive (?)
  // Some of the meshes that we'll receive in the geometry client are large,
  // like a few 10's of MB.
  world_channel_args.SetMaxReceiveMessageSize(-1);    // no limit
  world_channel_args.SetMaxSendMessageSize(10000000); // 10 MB

  auto world_address = this->get_parameter(kWorldAddressParamName).get_value<std::string>();
  auto maybe_world_channel = intrinsic::CreateClientChannel(
    world_address, absl::Now() + absl::Seconds(kWorldDeadlineSeconds));
  if (!maybe_world_channel.ok()) {
    LOG(ERROR) << maybe_world_channel;
    return CallbackReturn::FAILURE;
  }
  auto world_channel = maybe_world_channel.value();
  LOG(INFO) << "World address: " << world_address;

  auto geometry_address = this->get_parameter(kGeometryAddressParamName).get_value<std::string>();
  auto maybe_geometry_channel = intrinsic::CreateClientChannel(
    geometry_address, absl::Now() + absl::Seconds(kWorldDeadlineSeconds));
  if (!maybe_geometry_channel.ok()) {
    LOG(ERROR) << maybe_geometry_channel;
    return CallbackReturn::FAILURE;
  }
  auto geometry_channel = maybe_geometry_channel.value();
  LOG(INFO) << "Geometry address: " << geometry_address;

  // Initialize the world client.
  this->world_ = std::make_shared<World>(
      this->pubsub_, world_channel, geometry_channel, kWorldDeadlineSeconds);

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
