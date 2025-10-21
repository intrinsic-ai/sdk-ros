cmake_minimum_required(VERSION 3.8)
project(local_flowstate_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(flowstate_ros_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(flowstate_ros_bridge
 src/local_flowstate_ros_bridge_main.cpp
)
target_include_directories(flowstate_ros_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(flowstate_ros_bridge
  flowstate_ros_bridge::executive
  Boost::program_options
)

install(TARGETS
  flowstate_ros_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
