#include "icon_hwm_controller/icon_hwm_controller.hpp"

namespace icon_hwm_controller {

controller_interface::InterfaceConfiguration IconHwmController::command_interface_configuration() const {
  return {};
}

controller_interface::InterfaceConfiguration IconHwmController::state_interface_configuration() const {
  return {};
}

controller_interface::CallbackReturn IconHwmController::on_init() {
  return {};
}

controller_interface::CallbackReturn IconHwmController::on_configure(const rclcpp_lifecycle::State & previous_state) {
  return {};
}

controller_interface::CallbackReturn IconHwmController::on_activate(const rclcpp_lifecycle::State & previous_state) {
  return {};
}

controller_interface::CallbackReturn IconHwmController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
  return {};
}

controller_interface::return_type IconHwmController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
  return {};
}

}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  icon_hwm_controller::IconHwmController, controller_interface::ControllerInterface)
