cmake_minimum_required(VERSION 3.8)
project(local_flowstate_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(flowstate_ros_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(local_flowstate_ros_bridge_component SHARED
 src/channel_factory.cpp
 src/local_flowstate_ros_bridge.cpp
 src/local_flowstate_ros_bridge_main.cpp
)
target_link_libraries(local_flowstate_ros_bridge_component
  PRIVATE
    flowstate_ros_bridge::flowstate_ros_bridge_interface
    Boost::program_options
    rclcpp_lifecycle::rclcpp_lifecycle
    pluginlib::pluginlib
    rclcpp_components::component
)

rclcpp_components_register_node(local_flowstate_ros_bridge_component
  PLUGIN "local_flowstate_ros_bridge::LocalFlowstateROSBridge"
  EXECUTABLE local_flowstate_ros_bridge
  EXECUTOR EventsExecutor
)

install(TARGETS
  local_flowstate_ros_bridge_component
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
