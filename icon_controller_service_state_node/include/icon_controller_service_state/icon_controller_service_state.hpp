#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"

#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "icon_hwm_controller_msgs/msg/hardware_module_state.hpp"

namespace intrinsic::icon_service_state {

class IconControllerServiceState final : public intrinsic_proto::services::v1::ServiceState::Service {
 public:
  IconControllerServiceState(
      rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr list_hardware_components_client,
      rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr set_hardware_component_state_client,
      rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client,
      rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client,
      rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client,
      rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client
  );

 ::grpc::Status GetState(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::GetStateRequest* request,
      intrinsic_proto::services::v1::SelfState* response) override;

  ::grpc::Status Enable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::EnableRequest* request,
      intrinsic_proto::services::v1::EnableResponse* response) override;

  ::grpc::Status Disable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::DisableRequest* request,
      intrinsic_proto::services::v1::DisableResponse* response) override;

  void NotifyHardwareModuleState(const icon_hwm_controller_msgs::msg::HardwareModuleState& state);

 private:
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr list_hardware_components_client_;
rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr set_hardware_component_state_client_;

  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client_;
rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client_;
rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client_;

  icon_hwm_controller_msgs::msg::HardwareModuleState latest_state_;
};
}
