#include "icon_controller_service_state/icon_controller_service_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"

namespace intrinsic::icon_service_state {

IconControllerServiceState::IconControllerServiceState(
    rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr list_hardware_components_client,
    rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr set_hardware_component_state_client,
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client,
    rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client,
    rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client,
    rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client
) : list_hardware_components_client_(list_hardware_components_client),
    set_hardware_component_state_client_(set_hardware_component_state_client),
    list_controllers_client_(list_controllers_client),
    unload_controller_client_(unload_controller_client),
    load_controller_client_(load_controller_client),
    configure_controller_client_(configure_controller_client) {}

void IconControllerServiceState::NotifyHardwareModuleState(const icon_hwm_controller_msgs::msg::HardwareModuleState& state) {
  latest_state_ = state;
}

 ::grpc::Status IconControllerServiceState::GetState(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::GetStateRequest* request,
      intrinsic_proto::services::v1::SelfState* response) {
   return {};
 }

  ::grpc::Status IconControllerServiceState::Enable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::EnableRequest* request,
      intrinsic_proto::services::v1::EnableResponse* response) {
   return {};
 }

  ::grpc::Status IconControllerServiceState::Disable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::DisableRequest* request,
      intrinsic_proto::services::v1::DisableResponse* response) {
   return {};
 }
}
