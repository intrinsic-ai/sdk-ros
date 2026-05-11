#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "icon_controller_service_state/icon_controller_service_state.hpp"

namespace intrinsic::icon_service_state {

class ServiceStateNode final : public  ::rclcpp::Node {
 public:
  ServiceStateNode()
      : Node("icon_hwm_service_state"),
        list_hardware_components_client_(create_client<controller_manager_msgs::srv::ListHardwareComponents>("/controller_manager/list_hardware_components")),
        set_hardware_component_state_client_(create_client<controller_manager_msgs::srv::SetHardwareComponentState>("/controller_manager/set_hardware_component_state")),
        list_controllers_client_(create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers")),
        unload_controller_client_(create_client<controller_manager_msgs::srv::UnloadController>("/controller_manager/unload_controller")),
        load_controller_client_(create_client<controller_manager_msgs::srv::LoadController>("/controller_manager/load_controller")),
        configure_controller_client_(create_client<controller_manager_msgs::srv::ConfigureController>("/controller_manager/configure_controller")),
        service_state_impl_(list_hardware_components_client_, set_hardware_component_state_client_, list_controllers_client_, unload_controller_client_, load_controller_client_, configure_controller_client_)
  {
    subscription_ = create_subscription<icon_hwm_controller_msgs::msg::HardwareModuleState>(
      "hardware_module_state", 10,
      [this](const icon_hwm_controller_msgs::msg::HardwareModuleState::SharedPtr msg) {
        service_state_impl_.NotifyHardwareModuleState(*msg);
      });
  }

 private:
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr list_hardware_components_client_;
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr set_hardware_component_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client_;

  rclcpp::Subscription<icon_hwm_controller_msgs::msg::HardwareModuleState>::SharedPtr subscription_;

  IconControllerServiceState service_state_impl_;
};

}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<intrinsic::icon_service_state::ServiceStateNode>());
  rclcpp::shutdown();
  return 0;
}
