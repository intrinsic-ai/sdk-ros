#include "diagnostics_bridge.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_timer.hpp"

// This is the magic macro that registers this class with pluginlib.
PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::DiagnosticsBridge,
                       flowstate_ros_bridge::BridgeInterface)

namespace flowstate_ros_bridge {

constexpr const char* kDiagnosticsUpdateRateParamName = "diagnostics_update_rate_hz";

///=============================================================================
void DiagnosticsBridge::declare_ros_parameters(
    ROSNodeInterfaces ros_node_interfaces) {
  const auto& param_interface =
      ros_node_interfaces
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  param_interface->declare_parameter(kDiagnosticsUpdateRateParamName, rclcpp::ParameterValue(1.0));
}

///=============================================================================
bool DiagnosticsBridge::initialize(
    ROSNodeInterfaces ros_node_interfaces,
    std::shared_ptr<Executive> /*executive_client*/,
    std::shared_ptr<World> /*world_client*/,
    std::shared_ptr<Diagnostics> diagnostics_client) {
  diagnostics_ = diagnostics_client;
  // Get parameters.
  const double update_rate_hz =
      ros_node_interfaces.get<rclcpp::node_interfaces::NodeParametersInterface>()->get_parameter(kDiagnosticsUpdateRateParamName).as_double();
  update_period_ =
      std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_hz));

  // Create the ROS publisher.
  auto topics_interface = ros_node_interfaces.get<rclcpp::node_interfaces::NodeTopicsInterface>();
  auto params_interface = ros_node_interfaces.get<rclcpp::node_interfaces::NodeParametersInterface>();
  diagnostics_pub_ = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      params_interface,
      topics_interface,
      "/diagnostics", rclcpp::SystemDefaultsQoS());

  // Create the ROS timer to trigger updates.
  // The call to create_wall_timer on the interface is failing to compile.
  // Using the free function `rclcpp::create_timer` is a more robust
  // alternative that should resolve this.
  timer_ = rclcpp::create_timer(
      ros_node_interfaces.get<rclcpp::node_interfaces::NodeBaseInterface>(),
      ros_node_interfaces.get<rclcpp::node_interfaces::NodeTimersInterface>(),
      ros_node_interfaces.get<rclcpp::node_interfaces::NodeClockInterface>()->get_clock(),
      update_period_, std::bind(&DiagnosticsBridge::update_diagnostics, this));

  return true;
}

void DiagnosticsBridge::update_diagnostics() {
  if (!diagnostics_) {
    return;
  }

  // 1. Make the request via the diagnostics module.
  absl::StatusOr<std::vector<intrinsic_proto::services::v1::InstanceState>>
      response_or = diagnostics_->get_diagnostics();

  if (!response_or.ok()) {
    RCLCPP_WARN(rclcpp::get_logger("DiagnosticsBridge"),
                "Failed to get instance states: %s",
                response_or.status().ToString().c_str());
    return;
  }

  // 2. Translate the response to a ROS message.
  auto diag_array_msg =
      std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  diag_array_msg->header.stamp = rclcpp::Clock().now();

  for (const auto& instance_state : *response_or) {
    diagnostic_msgs::msg::DiagnosticStatus status_msg;
    status_msg.name = "service:" + instance_state.name();
    status_msg.hardware_id = "platform";
    status_msg.level = to_diagnostic_level(instance_state.state().state_code());

    // Add key-value pairs for more detail.
    diagnostic_msgs::msg::KeyValue kv_state;
    kv_state.key = "Platform State";
    kv_state.value = intrinsic_proto::services::v1::State::StateCode_Name(
        instance_state.state().state_code());
    status_msg.values.push_back(kv_state);

    if (instance_state.state().has_extended_status()) {
      status_msg.message = instance_state.state().extended_status().DebugString();
    } else {
      status_msg.message = "OK";
    }

    diag_array_msg->status.push_back(status_msg);
  }

  // 3. Publish the ROS message.
  diagnostics_pub_->publish(std::move(diag_array_msg));
}

int8_t DiagnosticsBridge::to_diagnostic_level(
    intrinsic_proto::services::v1::State::StateCode state_code) {
  using DiagLevel = diagnostic_msgs::msg::DiagnosticStatus;
  using State = intrinsic_proto::services::v1::State;

  // The C++ values for nested protobuf enums are scoped within the parent
  // message's generated class.
  if (state_code == State::STATE_CODE_ENABLED) {
    return DiagLevel::OK;
  } else if (state_code == State::STATE_CODE_DISABLED ||
             state_code == State::STATE_CODE_STOPPED) {
    return DiagLevel::WARN;
  } else if (state_code == State::STATE_CODE_ERROR) {
    return DiagLevel::ERROR;
  }
  return DiagLevel::STALE;
}

}  // namespace flowstate_ros_bridge
