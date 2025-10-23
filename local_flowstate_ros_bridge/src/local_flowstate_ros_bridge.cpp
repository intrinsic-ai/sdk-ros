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

#include "channel_factory.hpp"

#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

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

  auto org_project =
      this->get_parameter(kOrgProjectParamName).get_value<std::string>();
  auto cluster =
      this->get_parameter(kClusterParamName).get_value<std::string>();

  // Initialize the executive client.
  this->executive_ = std::make_shared<flowstate_ros_bridge::Executive>(
      "", "", "", // ClusterChannelFactory gets the service addresses from the
                  // org and cluster
      5, 1000, std::make_unique<ClusterChannelFactory>(org_project, cluster));

  // Initialize the world client.
  this->world_ = std::make_shared<flowstate_ros_bridge::World>(
      this->pubsub_, "", "", // ClusterChannelFactory gets the service
                             // addresses from the org and cluster
      10, std::make_unique<ClusterChannelFactory>(org_project, cluster));

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
