#include <fstream>
#include <memory>

#include "absl/log/log.h"
#include "class_loader/class_loader.hpp"
#include "flowstate_ros_bridge.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

///=============================================================================
intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    // Return default context for running locally
    std::cerr << "Warning: using default RuntimeContext\n";
  }
  return runtime_context;
}

///=============================================================================
int main(int argc, char* argv[]) {
  auto runtime_context = GetRuntimeContext();
  intrinsic::FlowstateRosBridgeConfig ros_config;
  if (!runtime_context.config().UnpackTo(&ros_config)) {
    LOG(WARNING) << "Error unpacking FlowstateRosBridgeConfig from service "
                    "config file... Passing empty ros args to node";
  }

  // Handle optional external router address
  std::string external_router_address;
  if (!ros_config.external_zenoh_router_address().empty()) {
    external_router_address = ros_config.external_zenoh_router_address();
  }

  // If external router address is provided, override the Zenoh environment
  // variable
  std::string zenoh_config_override =
      "listen/endpoints=[\"ws/0.0.0.0:" +
      std::to_string(runtime_context.http_port()) + "\"];connect/endpoints=[\"";
  if (!external_router_address.empty()) {
    zenoh_config_override += external_router_address;
  } else {
    // If no external address, look for "flowstate_zenoh_router_address" param
    zenoh_config_override += ros_config.flowstate_zenoh_router_address();
  }
  zenoh_config_override += "\"]";
  setenv("ZENOH_CONFIG_OVERRIDE", zenoh_config_override.c_str(), 1);
  LOG(INFO) << "ZENOH_CONFIG_OVERRIDE: " << zenoh_config_override;

  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::NodeOptions options;

  std::vector<rclcpp::Parameter> params;
  // Get parameters from config
  params.emplace_back("executive_service_address",
                      ros_config.executive_service_address());
  params.emplace_back("skill_registry_address",
                      ros_config.skill_registry_address());
  params.emplace_back("solution_service_address",
                      ros_config.solution_service_address());
  params.emplace_back("world_service_address",
                      ros_config.world_service_address());
  params.emplace_back("geometry_service_address",
                      ros_config.geometry_service_address());
  params.emplace_back("flowstate_zenoh_router_address",
                      ros_config.flowstate_zenoh_router_address());
  const auto& bridge_plugins_proto = ros_config.bridge_plugins();
  std::vector<std::string> plugin_list(bridge_plugins_proto.begin(),
                                       bridge_plugins_proto.end());
  rclcpp::Parameter bridge_plugins_param("bridge_plugins", plugin_list);
  params.push_back(std::move(bridge_plugins_param));

  options.parameter_overrides(params);

  // Get namespace from config
  std::vector<std::string> remap_rules;
  remap_rules.push_back("--ros-args");
  if (ros_config.workcell_id() != "") {
    remap_rules.push_back("-r");
    remap_rules.push_back("__ns:=/" + ros_config.workcell_id());
  }
  options.arguments(remap_rules);

  // Create and spin the FlowstateROSBridge node
  // Adapted from rclcpp_components::node_main.cpp.in
  // TODO: Find a cleaner way of implementing this, this is basically the
  // generated file from rclcpp_components::node_main.cpp.in
  std::string library_name = "libflowstate_ros_bridge_component.so";
  std::string class_name =
      "rclcpp_components::NodeFactoryTemplate<flowstate_ros_bridge::"
      "FlowstateROSBridge>";

  LOG(INFO) << "Load library " << library_name;
  auto loader = std::make_unique<class_loader::ClassLoader>(library_name);
  std::vector<std::string> classes =
      loader->getAvailableClasses<rclcpp_components::NodeFactory>();

  if (std::find(classes.begin(), classes.end(), class_name) == classes.end()) {
    LOG(INFO) << "Class " << class_name << " not found in library "
              << library_name;
    return 1;
  }
  LOG(INFO) << "Instantiate class " << class_name;
  std::shared_ptr<rclcpp_components::NodeFactory> node_factory = nullptr;
  try {
    node_factory =
        loader->createInstance<rclcpp_components::NodeFactory>(class_name);
  } catch (const std::exception& ex) {
    LOG(ERROR) << "Failed to load library " << ex.what();
    return 1;
  } catch (...) {
    LOG(ERROR) << "Failed to load library";
    return 1;
  }
  // Scope to destruct node_wrapper before shutdown
  {
    rclcpp_components::NodeInstanceWrapper node_wrapper =
        node_factory->create_node_instance(options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node =
        node_wrapper.get_node_base_interface();
    exec.add_node(node);

    exec.spin();

    exec.remove_node(node_wrapper.get_node_base_interface());
  }

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}
