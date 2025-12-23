#ifndef DIAGNOSTICS_BRIDGE_HPP_
#define DIAGNOSTICS_BRIDGE_HPP_

#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "flowstate_ros_bridge/diagnostics.hpp"

namespace flowstate_ros_bridge {

class DiagnosticsBridge : public BridgeInterface {
 public:
  DiagnosticsBridge() = default;
  ~DiagnosticsBridge() override = default;

  void declare_ros_parameters(
      ROSNodeInterfaces ros_node_interfaces) final;

  bool initialize(
      ROSNodeInterfaces ros_node_interfaces,
      std::shared_ptr<Executive> executive_client,
      std::shared_ptr<World> world_client,
      std::shared_ptr<Diagnostics> diagnostics_client) final;

 private:
  std::shared_ptr<Diagnostics> diagnostics_;

  // ROS publisher for the diagnostics messages.
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;

  // Logger.
  rclcpp::Logger logger_{rclcpp::get_logger("DiagnosticsBridge")};

  // ROS timer to periodically poll for diagnostics.
  rclcpp::TimerBase::SharedPtr timer_;

  // Member function called by the timer to perform the update.
  void update_diagnostics();

  // Helper to convert platform state to ROS diagnostic level.
  int8_t to_diagnostic_level(
      intrinsic_proto::services::v1::State::StateCode state_code);

  // Configurable parameters.
  std::chrono::milliseconds update_period_{1000};
  std::chrono::seconds grpc_deadline_{5};
};

}  // namespace flowstate_ros_bridge
#endif  // DIAGNOSTICS_BRIDGE_HPP_
