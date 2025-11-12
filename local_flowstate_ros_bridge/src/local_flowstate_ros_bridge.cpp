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

#include "local_flowstate_ros_bridge.hpp"


#include <absl/status/status.h>
#include <flowstate_ros_bridge/bridge_interface.hpp>
#include <intrinsic/platform/pubsub/zenoh_publisher_data.h> // required in gcc because it is more strict in forward declaration
#include <intrinsic/util/grpc/channel.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#define ASSIGN_OR_RETURN_FAILURE_CONCAT_IMPL(x, y) x##y
#define ASSIGN_OR_RETURN_FAILURE_CONCAT(x, y)                                  \
  ASSIGN_OR_RETURN_FAILURE_CONCAT_IMPL(x, y)
#define ASSIGN_OR_RETURN_FAILURE(lhs, rhs)                                     \
  auto ASSIGN_OR_RETURN_FAILURE_CONCAT(macro_internal_status_, __LINE__) =     \
      (rhs);                                                                   \
  if (!ASSIGN_OR_RETURN_FAILURE_CONCAT(macro_internal_status_, __LINE__)       \
           .ok()) {                                                            \
    LOG(ERROR) << ASSIGN_OR_RETURN_FAILURE_CONCAT(macro_internal_status_,      \
                                                  __LINE__)                    \
                      .status();                                               \
    return CallbackReturn::FAILURE;                                            \
  }                                                                            \
  lhs = std::move(                                                             \
      ASSIGN_OR_RETURN_FAILURE_CONCAT(macro_internal_status_, __LINE__)        \
          .value())

namespace local_flowstate_ros_bridge {

constexpr const char *kOrgProjectParamName = "org";
constexpr const char *kClusterParamName = "cluster";
constexpr const char *kAutostartParamName = "autostart";
constexpr const char *kBridgePluginParamName = "bridge_plugins";

LocalFlowstateROSBridge::LocalFlowstateROSBridge(
    const rclcpp::NodeOptions &options)
    : LifecycleNode("flowstate_ros_bridge", options),
      bridge_loader_("flowstate_ros_bridge",
                     "flowstate_ros_bridge::BridgeInterface") {
  rcl_interfaces::msg::ParameterDescriptor org_pd;
  org_pd.name = kOrgProjectParamName;
  org_pd.description = "Organization in the form org@project";
  this->declare_parameter(kOrgProjectParamName, "", org_pd);

  rcl_interfaces::msg::ParameterDescriptor cluster_pd;
  cluster_pd.name = kClusterParamName;
  cluster_pd.description = "Target cluster";
  this->declare_parameter(kClusterParamName, "", cluster_pd);

  const bool autostart = this->declare_parameter(kAutostartParamName, true);

  this->declare_parameter(kBridgePluginParamName, std::vector<std::string>{});

  this->pubsub_ = std::make_shared<intrinsic::PubSub>(this->get_name());

  bridge_ids_ = this->get_parameter(kBridgePluginParamName).as_string_array();

  for (const auto &id : bridge_ids_) {
    try {
      auto bridge = bridge_loader_.createUniqueInstance(id);
      bridge->declare_ros_parameters(*this);
      bridges_.push_back(std::move(bridge));
    } catch (const std::exception &e) {
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

auto LocalFlowstateROSBridge::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn {
  auto org_project =
      this->get_parameter(kOrgProjectParamName).get_value<std::string>();
  ASSIGN_OR_RETURN_FAILURE(
      auto org_info, intrinsic::Channel::OrgInfo::FromString(org_project));

  auto cluster =
      this->get_parameter(kClusterParamName).get_value<std::string>();

  auto maybe_cluster_channel =
      intrinsic::Channel::MakeFromCluster(org_info, cluster);
  if (!maybe_cluster_channel.ok()) {
    if (maybe_cluster_channel.status().code() == absl::StatusCode::kNotFound) {
      std::cerr << "Cannot find authentication information." << std::endl
                << "Run `inctl auth login --org=" << org_project
                << "` to authenticate to flowstate." << std::endl
                << std::endl
                << "Download inctl from "
                   "https://github.com/intrinsic-ai/sdk/releases."
                << std::endl;
    } else {
      LOG(ERROR) << maybe_cluster_channel.status();
    }
    return CallbackReturn::FAILURE;
  }
  auto grpc_channel = maybe_cluster_channel.value()->GetChannel();

  // Initialize the executive client.
  this->executive_ = std::make_shared<flowstate_ros_bridge::Executive>(
      grpc_channel, grpc_channel, grpc_channel);

  // Initialize the world client.
  // FIXME(koonpeng): pubsub does not work because flowstate does not expose it
  // atm.
  this->world_ = std::make_shared<flowstate_ros_bridge::World>(
      this->pubsub_, grpc_channel, grpc_channel);

  // Configure client services.
  auto connect_result = this->executive_->connect();
  if (!connect_result.ok()) {
    LOG(ERROR) << connect_result;
    return CallbackReturn::FAILURE;
  }
  if (!this->world_->connect().ok()) {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

auto LocalFlowstateROSBridge::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn {
  // Initialize/start the various bridges.
  for (const auto &b : bridges_) {
    try {
      if (!b->initialize(*this, this->executive_, this->world_)) {
        return CallbackReturn::FAILURE;
      }
    } catch (const std::exception &e) {
      LOG(ERROR) << "Error initializing bridges: " << e.what();
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}

auto LocalFlowstateROSBridge::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn {
  // TODO(Yadunund): Should bridges have a "deactivate" api?
  return CallbackReturn::SUCCESS;
}

auto LocalFlowstateROSBridge::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn {
  bridges_.clear();
  executive_.reset();
  world_.reset();
  LOG(INFO) << "Closing PubSub session...";
  pubsub_.reset();
  LOG(INFO) << "Done.";
  return CallbackReturn::SUCCESS;
}

} // namespace local_flowstate_ros_bridge

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
    local_flowstate_ros_bridge::LocalFlowstateROSBridge)
