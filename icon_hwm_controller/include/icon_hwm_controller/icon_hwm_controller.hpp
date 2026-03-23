#pragma once

#include "controller_interface/controller_interface.hpp"

namespace icon_hwm_controller {

class IconHwmController final : public controller_interface::ControllerInterface {
 public:
  IconHwmController() = default;

  /// Get configuration for controller's required command interfaces.
  /**
   * Method used by the controller_manager to get the set of command interfaces used by the
   * controller. Each controller can use individual method to determine interface names that in
   * simples case have the following format: `<joint>/<interface>`. The method is called only in
   * `inactive` or `active` state, i.e., `on_configure` has to be called first. The configuration is
   * used to check if controller can be activated and to claim interfaces from hardware. The claimed
   * interfaces are populated in the \ref ControllerInterfaceBase::command_interfaces_
   * "command_interfaces_" member.
   *
   * \returns configuration of command interfaces.
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /// Get configuration for controller's required state interfaces.
  /**
   * Method used by the controller_manager to get the set of state interface used by the controller.
   * Each controller can use individual method to determine interface names that in simples case
   * have the following format: `<joint>/<interface>`.
   * The method is called only in `inactive` or `active` state, i.e., `on_configure` has to be
   * called first.
   * The configuration is used to check if controller can be activated and to claim interfaces from
   * hardware.
   * The claimed interfaces are populated in the
   * \ref ControllerInterfaceBase::state_interfaces_ "state_interfaces_" member.
   *
   * \returns configuration of state interfaces.
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;


  /// Extending interface with initialization method which is individual for each controller
  controller_interface::CallbackReturn on_init() override;


  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * Control step update. Command interfaces are updated based on on reference inputs and current
   * states.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time since the last control loop iteration
   * \returns return_type::OK if update is successfully, otherwise return_type::ERROR.
   */
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

 private:
};

}
