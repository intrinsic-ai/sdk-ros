#include "intrinsic/utils/strerror.hpp"
#include "icon_hwm_controller/icon_hwm_controller.hpp"

#include <cmath>
#include <algorithm>

#include "intrinsic/thread/thread.hpp"
#include "intrinsic/utils/log.hpp"
#include "intrinsic/utils/time.hpp"
#include "intrinsic/hal/joint_command_utils.hpp"
#include "intrinsic/hal/joint_state_utils.hpp"
#include "intrinsic/hal/joint_limits_utils.hpp"
#include "intrinsic/hal/hardware_module_state_utils.hpp"
#include "intrinsic/hal/hardware_interface_traits.hpp"
#include "intrinsic/hal/hardware_interface_registry.hpp"
#include "intrinsic/hal/icon_state_register.hpp"
#include "intrinsic/shared_memory_manager/domain_socket_server.hpp"
#include "intrinsic/shared_memory_manager/shared_memory_manager.hpp"

#include "realtime_tools/realtime_helpers.hpp"

namespace intrinsic::hal::hardware_interface_traits {

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointPositionCommand,
                                 intrinsic_fbs::BuildJointPositionCommand,
                                 "intrinsic_fbs.JointPositionCommand")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointCommandedPosition,
                                 intrinsic_fbs::BuildJointCommandedPosition,
                                 "intrinsic_fbs.JointCommandedPosition")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointPositionState,
                                 intrinsic_fbs::BuildJointPositionState,
                                 "intrinsic_fbs.JointPositionState")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointVelocityState,
                                 intrinsic_fbs::BuildJointVelocityState,
                                 "intrinsic_fbs.JointVelocityState")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointLimits,
                                 intrinsic_fbs::BuildJointLimits,
                                 "intrinsic_fbs.JointLimits")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::HardwareModuleState,
                                 intrinsic_fbs::BuildHardwareModuleState,
                                 "intrinsic_fbs.HardwareModuleState")

#if 0
INTRINSIC_ADD_HARDWARE_INTERFACE(::intrinsic_fbs::PayloadCommand,
                                 intrinsic_fbs::BuildPayloadCommand,
                                 "intrinsic_fbs.PayloadCommand")

INTRINSIC_ADD_HARDWARE_INTERFACE(::intrinsic_fbs::PayloadState,
                                 intrinsic_fbs::BuildPayloadState,
                                 "intrinsic_fbs.PayloadState")
#endif
}

namespace icon_hwm_controller {

controller_interface::InterfaceConfiguration IconHwmController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  // By specifying INDIVIDUAL here, we ensure that the interfaces
  // appear in the same order we request them in.
  //
  // See https://github.com/ros-controls/ros2_control/blob/7c5e76766307705b3ef0c28247a17c91814fb311/controller_interface/include/controller_interface/controller_interface_base.hpp#L373-L400
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & dof_name : params_.dof_names) {
    config.names.push_back(dof_name + "/" + params_.command_interface);
  }
  return config;
}

controller_interface::InterfaceConfiguration IconHwmController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  // By specifying INDIVIDUAL here, we ensure that the interfaces
  // appear in the same order we request them in.
  //
  // See https://github.com/ros-controls/ros2_control/blob/7c5e76766307705b3ef0c28247a17c91814fb311/controller_interface/include/controller_interface/controller_interface_base.hpp#L373-L400
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & dof_name : params_.dof_names) {
    for (const auto & interface_type : params_.reference_and_state_interfaces) {
      config.names.push_back(dof_name + "/" + interface_type);
    }
  }
  return config;
}

controller_interface::CallbackReturn IconHwmController::on_init() {
  try {
    param_listener_ = std::make_unique<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  params_ = param_listener_->get_params();

  if (params_.name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'name' (ICON module name) is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Create Shared Memory Manager
  std::string shm_namespace = params_.shm_namespace;
  auto shared_memory_manager = intrinsic::hal::SharedMemoryManager::Create(shm_namespace, params_.name);
  if (!shared_memory_manager.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create SharedMemoryManager: %s", shared_memory_manager.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  shm_manager_ = std::move(shared_memory_manager.value());
  auto domain_socket_server = intrinsic::hal::DomainSocketServer::Create(
    intrinsic::hal::SocketDirectoryFromNamespace(shm_manager_->SharedMemoryNamespace()),
    shm_manager_->ModuleName(),
    intrinsic::hal::DomainSocketServer::kDefaultLockAcquireTimeout
  );
  if (!domain_socket_server.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create DomainSocketServer: %s", domain_socket_server.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  domain_socket_server_ = std::move(domain_socket_server.value());

 intrinsic::hal::HardwareInterfaceRegistry registry(*shm_manager_);
  // Advertise IconState
  auto icon_state = registry.AdvertiseInterface<intrinsic_fbs::IconState>(intrinsic::hal::kIconStateInterfaceName);
  if (!icon_state.has_value()){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise IconState: %s", icon_state.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  icon_state_ = std::move(icon_state.value());

  // Advertise JointPositionState
  // Build a default flatbuffer for JointPositionState with the correct number of DOFs
  auto joint_position_state = registry.AdvertiseMutableStrictInterface<intrinsic_fbs::JointPositionState>(
    "joint_position_state",
    params_.dof_names.size()
  );
  if (!joint_position_state.has_value()){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise JointPositionState: %s", joint_position_state.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_position_state_ = std::move(joint_position_state).value();

  // Advertise JointVelocityState
  // Build a default flatbuffer for JointVelocityState with the correct number of DOFs
  auto joint_velocity_state = registry.AdvertiseMutableStrictInterface<intrinsic_fbs::JointVelocityState>(
    "joint_velocity_state",
    params_.dof_names.size()
  );
  if (!joint_velocity_state.has_value()){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise JointVelocityState: %s", joint_velocity_state.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_velocity_state_ = std::move(joint_velocity_state).value();

  // Advertise JointPositionCommand
  auto joint_position_command = registry.AdvertiseStrictInterface<intrinsic_fbs::JointPositionCommand>(
    "joint_position_command",
    params_.dof_names.size());
  if (!joint_position_command.has_value()){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise JointPositionCommand: %s", joint_position_command.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_position_command_ = std::move(joint_position_command).value();

  // Advertise HardwareModuleState
  {
    auto hardware_module_state = registry.AdvertiseMutableInterface<intrinsic_fbs::HardwareModuleState>(
      "hardware_module_state");
    if (!hardware_module_state.has_value()){
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise HardwareModuleState: %s", hardware_module_state.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    hardware_module_state_ = std::move(hardware_module_state).value();
  }
  
  // Service Clients
  switch_controller_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
  set_hw_state_client_ = get_node()->create_client<controller_manager_msgs::srv::SetHardwareComponentState>("/controller_manager/set_hardware_component_state");

  // Remote Trigger Servers
  
  auto lock_memory = params_.lock_memory;
  auto cpu_core = params_.cpu_core;
  auto realtime_priority_low = params_.realtime_priority_low;
  auto realtime_priority_high = params_.realtime_priority_high;
  if (realtime_priority_low != -1 && realtime_priority_high != -1 && realtime_priority_low > realtime_priority_high) {
    RCLCPP_ERROR(get_node()->get_logger(), "realtime_priority_low is greater than realtime_priority_high. Ensure that the parameter configuration sets low to be lower than high.");
    return controller_interface::CallbackReturn::ERROR;
  }
  bool has_realtime_kernel = realtime_tools::has_realtime_kernel();

  auto setup_rt_thread = [=](int priority) -> intrinsic::Status {
    if (!has_realtime_kernel) {
      return intrinsic::OkStatus();
    }
    if (lock_memory) {
      auto lock_memory_result = realtime_tools::lock_memory();
      if (!lock_memory_result.first) {
        return intrinsic::Status{
          .code=intrinsic::StatusCode::kInternal,
          .message=(std::stringstream() 
              << "Failed to lock memory: " << lock_memory_result.second).str(),
        };
      }
    }
    if (cpu_core >= 0) {
     const auto affinity_result = realtime_tools::set_current_thread_affinity({cpu_core});
     if (!affinity_result.first) {
       return intrinsic::Status{
         .code=intrinsic::StatusCode::kInternal,
         .message=(std::stringstream() 
             << "Failed to set thread affinity: " << affinity_result.second).str(),
       };
     }
    }
    if (priority >= 0) {
      if (!realtime_tools::configure_sched_fifo(priority)){
        return intrinsic::Status{
          .code=intrinsic::StatusCode::kInternal,
          .message=(std::stringstream() 
              << "Failed to set realtime priority with error " << errno 
              << " (" << intrinsic::StrError(errno).data() << ")").str(),
        };
      }
    }
    return intrinsic::OkStatus();
  };
  {
    auto prepare_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "prepare", [this](){ (void)Prepare(); });
    if (!prepare_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create prepare server: %s", prepare_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    prepare_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(prepare_server_result.value()));
  }
  {
    auto activate_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "activate", [this](){ (void)Activate(); });
    if (!activate_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create activate server: %s", activate_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    activate_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(activate_server_result.value()));
  }
  {
    auto deactivate_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "deactivate", [this](){ (void)Deactivate(); });
    if (!deactivate_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create deactivate server: %s", deactivate_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    deactivate_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(deactivate_server_result.value()));
  }
  {
    auto enable_motion_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "enable_motion", [this](){ (void)EnableMotion(); });
    if (!enable_motion_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create enable_motion server: %s", enable_motion_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    enable_motion_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(enable_motion_server_result.value()));
  }
  {
    auto disable_motion_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "disable_motion", [this](){ (void)DisableMotion(); });
    if (!disable_motion_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create disable_motion server: %s", disable_motion_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    disable_motion_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(disable_motion_server_result.value()));
  }
  {
    auto clear_faults_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "clear_faults", [this](){ (void)ClearFaults(); });
    if (!clear_faults_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create clear_faults server: %s", clear_faults_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    clear_faults_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(clear_faults_server_result.value()));
  }
  {
    auto shutdown_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "shutdown", [this](){ (void)Shutdown(); });
    if (!shutdown_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create shutdown server: %s", shutdown_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    shutdown_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(shutdown_server_result.value()));
  }
  {
    auto read_status_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "read_status", [this](){ (void)ReadStatus(); });
    if (!read_status_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create read_status server: %s", read_status_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    read_status_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(read_status_server_result.value()));
  }
  {
    auto apply_command_server_result = intrinsic::hal::RemoteTriggerServer::Create(*shm_manager_, "apply_command", [this](){ (void)ApplyCommand(); });
    if (!apply_command_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create apply_command server: %s", apply_command_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    apply_command_server_ = std::make_unique<intrinsic::hal::RemoteTriggerServer>(
      std::move(apply_command_server_result.value()));
  }
  // Start the background threads for remote trigger servers.
  activate_server_->StartAsync(std::bind_front(setup_rt_thread, realtime_priority_low));
  deactivate_server_->StartAsync(std::bind_front(setup_rt_thread, realtime_priority_low));
  // ReadStatus and ApplyCommand run much more often than the others, so they get higher priority.
  read_status_server_->StartAsync(std::bind_front(setup_rt_thread, realtime_priority_high));
  apply_command_server_->StartAsync(std::bind_front(setup_rt_thread, realtime_priority_high));
  // The remaining servers *MUST NOT* run at realtime priority, since they can block and must not delay the execution of any of the realtime threads.
  auto state_change_query_thread_body = [this](){
    while (!stop_requested_){
      prepare_server_->Query();
      enable_motion_server_->Query();
      disable_motion_server_->Query();
      clear_faults_server_->Query();
      shutdown_server_->Query();
    }
  };
  state_change_query_thread_ = intrinsic::Thread(state_change_query_thread_body);
  // Must be clock driver. If not, return an error.
  if (!params_.drives_realtime_clock) {
    RCLCPP_ERROR(get_node()->get_logger(), "This controller must be a clock driver.");
    return controller_interface::CallbackReturn::ERROR;
  }
  auto clock_res = intrinsic::RealtimeClock::Create(*shm_manager_);
  if (!clock_res.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create RealtimeClock.");
    return controller_interface::CallbackReturn::ERROR;
  }
  clock_ = std::move(clock_res.value());

  // Start the domain socket server
  if (auto s = domain_socket_server_->AddSegmentInfoServeShmDescriptors(*shm_manager_); !s.ok()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to start domain socket server: %s", s.message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  cycle_counter_ = 0;
  faulted_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Stop all of the servers, join any threads, and reset the unique_ptrs.
  stop_requested_ = true;
  
  domain_socket_server_.reset();
  clock_.reset();
  state_change_query_thread_.join();
  apply_command_server_.reset();
  read_status_server_.reset();
  shutdown_server_.reset();
  clear_faults_server_.reset();
  disable_motion_server_.reset();
  enable_motion_server_.reset();
  deactivate_server_.reset();
  activate_server_.reset();
  prepare_server_.reset();
  set_hw_state_client_.reset();
  switch_controller_client_.reset();
  hardware_module_state_ = {};
  hardware_module_state_ = {};
  joint_position_command_ = {};
  joint_position_state_ = {};
  icon_state_ = {};
  shm_manager_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type IconHwmController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
  cycle_counter_++;
  
  UpdateHwmState();

  DetectFaults();

  // clock_ must not be nullptr here (if it is, the initialization failed)
  if (!clock_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Clock driver not initialized.");
    return controller_interface::return_type::ERROR;
  }
  auto now_shm = intrinsic::Now();
  auto deadline = now_shm + period.to_chrono<std::chrono::nanoseconds>();
  (void)clock_->TickBlockingWithDeadline(now_shm, deadline);
  // After TickBlocking returns, ICON has finished ReadStatus/ApplyCommand

  joint_position_state_.UpdatedAt(now_shm);
  joint_velocity_state_.UpdatedAt(now_shm);

  return controller_interface::return_type::OK;
}

void IconHwmController::DetectFaults() {
  faulted_ = false;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    if (std::isnan(state_interfaces_[i].get_optional().value_or(std::numeric_limits<double>::quiet_NaN()))) {
      faulted_ = true;
      break;
    }
  }
}

intrinsic::Status IconHwmController::Prepare() {
  if (!SetStateDirectly(intrinsic_fbs::StateCode::kPreparing)) {
    return {intrinsic::StatusCode::kFailedPrecondition, "Transition to Preparing not allowed"};
  }
  SetStateDirectly(intrinsic_fbs::StateCode::kPrepared);
  return intrinsic::OkStatus();
}

intrinsic::RealtimeStatus IconHwmController::Activate() {
  if (!SetStateDirectly(intrinsic_fbs::StateCode::kActivating)) {
    return {intrinsic::StatusCode::kFailedPrecondition, "Transition to Activating not allowed"};
  }
  SetStateDirectly(intrinsic_fbs::StateCode::kActivated);
  return intrinsic::RtOkStatus();
}

intrinsic::RealtimeStatus IconHwmController::Deactivate() {
  if (!SetStateDirectly(intrinsic_fbs::StateCode::kDeactivating)) {
    return {intrinsic::StatusCode::kFailedPrecondition, "Transition to Deactivating not allowed"};
  }
  SetStateDirectly(intrinsic_fbs::StateCode::kDeactivated);
  return intrinsic::RtOkStatus();
}

intrinsic::Status IconHwmController::EnableMotion() {
  if (!SetStateDirectly(intrinsic_fbs::StateCode::kMotionEnabling)) {
    return {intrinsic::StatusCode::kFailedPrecondition, "Transition to MotionEnabling not allowed"};
  }
  
  if (!params_.hardware_interface_name.empty()) {
    auto res = CallSetHwState(params_.hardware_interface_name, 3); // 3 = ACTIVE
    if (!res.ok()) {
      // TODO(nilsb): prepend "Failed to activate hardware interface: "
      SetStateDirectly(
        intrinsic_fbs::StateCode::kFaulted, 
        res.message);
      return res;
    }
  }

  if (clock_ != nullptr) {
    auto res = clock_->Reset(std::chrono::seconds(20));
    if (!res.ok()) {
      // TODO(nilsb): prepend "Failed to reset clock: "
      SetStateDirectly(
        intrinsic_fbs::StateCode::kFaulted, 
        res.GetMessage());
      return ToStatus(res);
    }
  }

  if (!params_.controllers_to_activate.empty() || !params_.controllers_to_deactivate.empty()) {
    auto res = CallSwitchController(params_.controllers_to_activate, params_.controllers_to_deactivate);
    if (!res.ok()) {
      // TODO(nilsb): prepend "Failed to switch controllers: "
      SetStateDirectly(
        intrinsic_fbs::StateCode::kFaulted, 
        res.message);
      return res;
    }
  }

  SetStateDirectly(intrinsic_fbs::StateCode::kMotionEnabled);
  return intrinsic::OkStatus();
}

intrinsic::Status IconHwmController::DisableMotion() {
  if (!SetStateDirectly(intrinsic_fbs::StateCode::kMotionDisabling)) {
    return {intrinsic::StatusCode::kFailedPrecondition, "Transition to MotionDisabling not allowed"};
  }
  
  if (!params_.controllers_to_activate.empty()) {
     // Deactivate the controllers we activated
     (void)CallSwitchController({}, params_.controllers_to_activate);
  }

  SetStateDirectly(intrinsic_fbs::StateCode::kActivated);
  return intrinsic::OkStatus();
}

intrinsic::Status IconHwmController::ClearFaults() {
  if (!SetStateDirectly(intrinsic_fbs::StateCode::kClearingFaults)) {
    return {intrinsic::StatusCode::kFailedPrecondition, "Transition to ClearingFaults not allowed"};
  }
  faulted_ = false;
  SetStateDirectly(intrinsic_fbs::StateCode::kActivated);
  return intrinsic::OkStatus();
}

intrinsic::Status IconHwmController::Shutdown() {
  SetStateDirectly(intrinsic_fbs::StateCode::kDeactivated);
  return intrinsic::OkStatus();
}

intrinsic::RealtimeStatus IconHwmController::ReadStatus() {
  auto * mutable_pos_state = joint_position_state_.MutableValue();
  auto * pos_vec = mutable_pos_state->mutable_position();
  
  auto * mutable_vel_state = joint_velocity_state_.MutableValue();
  auto * vel_vec = mutable_vel_state->mutable_velocity();
  
  size_t num_dofs = params_.dof_names.size();
  for (size_t i = 0; i < num_dofs; ++i) {
    // Interfaces in `state_interfaces_` are in the same order that we listed our desired
    // interfaces in state_interface_configuration()
    //
    // That is, joints appear in the order they do in the configuration, 
    // and for each joint, the state interfaces (usually position and velocity) do the same.
    auto pos_val = state_interfaces_[i * params_.reference_and_state_interfaces.size()].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    pos_vec->Mutate(i, pos_val);
    
    auto vel_val = state_interfaces_[i * params_.reference_and_state_interfaces.size() + 1].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    vel_vec->Mutate(i, vel_val);
  }

  auto now = intrinsic::Now();
  joint_position_state_.UpdatedAt(now);
  joint_velocity_state_.UpdatedAt(now);

  return intrinsic::RtOkStatus();
}

intrinsic::RealtimeStatus IconHwmController::ApplyCommand() {
  if (faulted_) {
    return {intrinsic::StatusCode::kFailedPrecondition, "Hardware is in fault state."};
  }

  auto cmd_res = joint_position_command_.Value();
  if (!cmd_res.has_value()) {
    // Command not updated this cycle?
    return cmd_res.error();
  }

  const auto * cmd = cmd_res.value();
  const auto * pos_vec = cmd->position();
  
  if (pos_vec->size() != command_interfaces_.size()) {
    return {intrinsic::StatusCode::kInternal, "Command vector size mismatch."};
  }

  for (size_t i = 0; i < pos_vec->size(); ++i) {
    (void)command_interfaces_[i].set_value(pos_vec->Get(i));
  }

  return intrinsic::RtOkStatus();
}

void IconHwmController::UpdateHwmState() {
  auto * mutable_state = *hardware_module_state_;
  
  mutable_state->mutate_code(state_code_.load());
  
  // Update message if faulted
  if (state_code_.load() == intrinsic_fbs::StateCode::kFaulted || 
      state_code_.load() == intrinsic_fbs::StateCode::kFatallyFaulted) {
     auto * msg_bytes = mutable_state->mutable_message();
     size_t len = std::min(fault_reason_.length(), (size_t)255);
     for (size_t i = 0; i < len; ++i) msg_bytes->Mutate(i, fault_reason_[i]);
     for (size_t i = len; i < 256; ++i) msg_bytes->Mutate(i, 0);
  }
  
  hardware_module_state_.UpdatedAt(intrinsic::Now());
}

bool IconHwmController::SetStateDirectly(
    intrinsic_fbs::StateCode state, 
    std::string_view fault_reason,
    bool force, 
    bool silent) {
  auto current_state = state_code_.load();
  auto guard_res = intrinsic::hal::HardwareModuleTransitionGuard(current_state, state);
  if (!force && guard_res != intrinsic::hal::TransitionGuardResult::kAllowed) {
    if (!silent && guard_res == intrinsic::hal::TransitionGuardResult::kProhibited) {
      RCLCPP_ERROR(get_node()->get_logger(), "Switching from %s to %s is prohibited!",
        intrinsic_fbs::EnumNameStateCode(current_state),
        intrinsic_fbs::EnumNameStateCode(state));
    }
    return false;
  }
  
  const bool state_changed = current_state != state;
  if (!silent && state_changed) {
    if (fault_reason.empty()) {
      RCLCPP_INFO(get_node()->get_logger(), "Switching from %s to %s",
        intrinsic_fbs::EnumNameStateCode(current_state),
        intrinsic_fbs::EnumNameStateCode(state));
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Switching from %s to %s with message '%.*s'",
        intrinsic_fbs::EnumNameStateCode(current_state),
        intrinsic_fbs::EnumNameStateCode(state),
        static_cast<int>(fault_reason.size()), fault_reason.data());
    }
  }

  if (!state_changed && fault_reason_ == fault_reason) {
    // Don't update timestamp when state and message is the same as before.
    return false;
  }
  // TODO(nilsb): When transitioning *away* from kMotionEnabled, make sure we take 
  // appropriate action to disable. Currently this class is a prototype that pretty much 
  // treats all transitions as no-ops, but in the future we'll need to do something.
  state_code_.store(state);
  fault_reason_ = fault_reason;

  if (*hardware_module_state_ != nullptr) {
    auto * mutable_state = *hardware_module_state_;
    mutable_state->mutate_code(state);
    
    auto * msg_bytes = mutable_state->mutable_message();
    const size_t max_len = msg_bytes->size();
    const size_t len = std::min(fault_reason.length(), max_len);
    std::memset(msg_bytes->data(), 0, max_len);
    std::memcpy(msg_bytes->data(), fault_reason.data(), len);
    
    hardware_module_state_.UpdatedAt(intrinsic::Now());
  }
  return state_changed;
}

intrinsic::Status IconHwmController::CallSwitchController(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate) {
  if (!switch_controller_client_->wait_for_service(std::chrono::seconds(1))) {
    return {intrinsic::StatusCode::kUnavailable, "SwitchController service not available"};
  }
  
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  
  auto result_future = switch_controller_client_->async_send_request(request);
  if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return {intrinsic::StatusCode::kDeadlineExceeded, "SwitchController timeout"};
  }
  
  auto response = result_future.get();
  if (!response->ok) {
    return {intrinsic::StatusCode::kInternal, "SwitchController failed"};
  }
  return intrinsic::OkStatus();
}

intrinsic::Status IconHwmController::CallSetHwState(const std::string& name, uint8_t state) {
  if (!set_hw_state_client_->wait_for_service(std::chrono::seconds(1))) {
    return {intrinsic::StatusCode::kUnavailable, "SetHardwareComponentState service not available"};
  }
  
  auto request = std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  request->name = name;
  request->target_state.id = state;
  
  auto result_future = set_hw_state_client_->async_send_request(request);
  if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return {intrinsic::StatusCode::kDeadlineExceeded, "SetHardwareComponentState timeout"};
  }
  
  auto response = result_future.get();
  if (!response->ok) {
    return {intrinsic::StatusCode::kInternal, "SetHardwareComponentState failed"};
  }
  return intrinsic::OkStatus();
}

} // namespace icon_hwm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  icon_hwm_controller::IconHwmController, controller_interface::ControllerInterface)
