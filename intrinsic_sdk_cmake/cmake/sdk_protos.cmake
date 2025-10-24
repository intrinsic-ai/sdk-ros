file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/protos_gen)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py)

file(GLOB_RECURSE intrinsic_proto_SRCS "${intrinsic_sdk_SOURCE_DIR}/**/*.proto")
list(FILTER intrinsic_proto_SRCS EXCLUDE REGEX "_test\\.proto$")

set(exclude_SRCS
  intrinsic/icon/proto/service.proto
  third_party/ros2/ros_interfaces/jazzy/diagnostic_msgs/srv/self_test.proto
  third_party/ros2/ros_interfaces/jazzy/diagnostic_msgs/srv/add_diagnostics.proto
  third_party/ros2/ros_interfaces/jazzy/rcl_interfaces/srv/set_parameters.proto
  third_party/ros2/ros_interfaces/jazzy/rcl_interfaces/srv/describe_parameters.proto
  third_party/ros2/ros_interfaces/jazzy/rcl_interfaces/srv/get_parameters.proto
  third_party/ros2/ros_interfaces/jazzy/rcl_interfaces/srv/set_parameters_atomically.proto
  third_party/ros2/ros_interfaces/jazzy/rcl_interfaces/srv/list_parameters.proto
  third_party/ros2/ros_interfaces/jazzy/rcl_interfaces/srv/get_parameter_types.proto
  third_party/ros2/ros_interfaces/jazzy/rcl_interfaces/srv/get_logger_levels.proto
  third_party/ros2/ros_interfaces/jazzy/rcl_interfaces/srv/set_logger_levels.proto
  third_party/ros2/ros_interfaces/jazzy/nav_msgs/srv/set_map.proto
  third_party/ros2/ros_interfaces/jazzy/nav_msgs/srv/load_map.proto
  third_party/ros2/ros_interfaces/jazzy/nav_msgs/srv/get_map.proto
  third_party/ros2/ros_interfaces/jazzy/nav_msgs/srv/get_plan.proto
  third_party/ros2/ros_interfaces/jazzy/pcl_msgs/srv/update_filename.proto
  third_party/ros2/ros_interfaces/jazzy/sensor_msgs/srv/set_camera_info.proto
  third_party/ros2/ros_interfaces/jazzy/test_msgs/srv/arrays.proto
  third_party/ros2/ros_interfaces/jazzy/test_msgs/action/nested_message.proto
  third_party/ros2/ros_interfaces/jazzy/test_msgs/action/fibonacci.proto
  third_party/ros2/ros_interfaces/jazzy/test_msgs/srv/basic_types.proto
  third_party/ros2/ros_interfaces/jazzy/test_msgs/srv/empty.proto
  third_party/ros2/ros_interfaces/jazzy/visualization_msgs/srv/get_interactive_markers.proto
  third_party/ros2/ros_interfaces/jazzy/control_msgs/action/parallel_gripper_command.proto
  third_party/ros2/ros_interfaces/jazzy/control_msgs/action/single_joint_position.proto
  third_party/ros2/ros_interfaces/jazzy/control_msgs/action/joint_trajectory.proto
  third_party/ros2/ros_interfaces/jazzy/control_msgs/action/gripper_command.proto
  third_party/ros2/ros_interfaces/jazzy/control_msgs/action/point_head.proto
  third_party/ros2/ros_interfaces/jazzy/control_msgs/srv/query_trajectory_state.proto
  third_party/ros2/ros_interfaces/jazzy/control_msgs/action/follow_joint_trajectory.proto
  third_party/ros2/ros_interfaces/jazzy/control_msgs/srv/query_calibration_state.proto
  third_party/ros2/ros_interfaces/jazzy/lifecycle_msgs/srv/get_state.proto
  third_party/ros2/ros_interfaces/jazzy/lifecycle_msgs/srv/get_available_transitions.proto
  third_party/ros2/ros_interfaces/jazzy/lifecycle_msgs/srv/get_available_states.proto
  third_party/ros2/ros_interfaces/jazzy/lifecycle_msgs/srv/change_state.proto
  third_party/ros2/ros_interfaces/jazzy/type_description_interfaces/srv/get_type_description.proto
  third_party/ros2/ros_interfaces/jazzy/std_srvs/srv/set_bool.proto
  third_party/ros2/ros_interfaces/jazzy/std_srvs/srv/empty.proto
  third_party/ros2/ros_interfaces/jazzy/std_srvs/srv/trigger.proto
  third_party/ros2/ros_interfaces/jazzy/action_msgs/srv/cancel_goal.proto
  third_party/ros2/ros_interfaces/jazzy/composition_interfaces/srv/load_node.proto
  third_party/ros2/ros_interfaces/jazzy/composition_interfaces/srv/unload_node.proto
  third_party/ros2/ros_interfaces/jazzy/composition_interfaces/srv/list_nodes.proto
  third_party/ros2/ros_interfaces/jazzy/tf2_msgs/action/lookup_transform.proto
  third_party/ros2/ros_interfaces/jazzy/tf2_msgs/srv/frame_graph.proto
)

foreach(file_to_remove IN LISTS exclude_SRCS)
  list(REMOVE_ITEM intrinsic_proto_SRCS ${intrinsic_sdk_SOURCE_DIR}/${file_to_remove})
endforeach()

set(grpc_SOURCE_DIR "${gRPC_DIR}/../../../share/grpc-proto")

# Prepare additional proto dependencies.
set(grpc_SRCS
  ${grpc_SOURCE_DIR}/src/proto/grpc/health/v1/health.proto
)
set(googleapis_SRCS
  ${googleapis_SOURCE_DIR}/google/api/annotations.proto
  ${googleapis_SOURCE_DIR}/google/api/client.proto
  ${googleapis_SOURCE_DIR}/google/api/field_behavior.proto
  ${googleapis_SOURCE_DIR}/google/api/http.proto
  ${googleapis_SOURCE_DIR}/google/api/launch_stage.proto
  ${googleapis_SOURCE_DIR}/google/longrunning/operations.proto
  ${googleapis_SOURCE_DIR}/google/rpc/code.proto
  ${googleapis_SOURCE_DIR}/google/rpc/status.proto
  ${googleapis_SOURCE_DIR}/google/type/color.proto
)
set(grpc_gateway_SRCS
  ${grpc_gateway_SOURCE_DIR}/protoc-gen-openapiv2/options/annotations.proto
  ${grpc_gateway_SOURCE_DIR}/protoc-gen-openapiv2/options/openapiv2.proto
)

set(sdk_protos
  ${intrinsic_proto_SRCS}
  ${googleapis_SRCS}
  ${grpc_gateway_SRCS}
  ${grpc_SRCS}
)
set(sdk_proto_import_dirs
  ${intrinsic_sdk_SOURCE_DIR}
  ${googleapis_SOURCE_DIR}
  ${grpc_gateway_SOURCE_DIR}
  ${grpc_SOURCE_DIR}
)

# Generate code from protos and build into a library.
add_library(intrinsic_sdk_protos STATIC
  ${sdk_protos}
)
set_property(TARGET intrinsic_sdk_protos PROPERTY POSITION_INDEPENDENT_CODE ON)
protobuf_generate(
  TARGET intrinsic_sdk_protos
  LANGUAGE cpp
  IMPORT_DIRS ${sdk_proto_import_dirs}
  PROTOC_OPTIONS --experimental_editions
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/protos_gen
)
target_include_directories(intrinsic_sdk_protos
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/protos_gen>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(intrinsic_sdk_protos PUBLIC protobuf::libprotobuf)

# Build service related protos into a separate library.
add_library(intrinsic_sdk_services
  STATIC
    ${intrinsic_proto_SRCS}
    ${googleapis_SOURCE_DIR}/google/longrunning/operations.proto
    ${grpc_SRCS}
)
protobuf_generate(
  TARGET intrinsic_sdk_services
  LANGUAGE grpc
  PLUGIN protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
  IMPORT_DIRS ${sdk_proto_import_dirs}
  PROTOC_OPTIONS --experimental_editions
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/protos_gen
  GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc
)
target_include_directories(intrinsic_sdk_services
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/protos_gen>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(intrinsic_sdk_services PUBLIC intrinsic_sdk_protos)
set_property(TARGET intrinsic_sdk_services PROPERTY POSITION_INDEPENDENT_CODE ON)

# Generate Python code from the protos.
protobuf_generate(
  LANGUAGE python
  PROTOS ${sdk_protos}
  IMPORT_DIRS ${sdk_proto_import_dirs}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py
  OUT_VAR sdk_protos_python_sources
)
# Create empty __init__.py files for each python package generated by protoc.
set(python_package_directories "")
foreach(generated_python_file_maybe_hyphen ${sdk_protos_python_sources})
  file(RELATIVE_PATH rel_generated_python_file_maybe_hyphen
    "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py"
    "${generated_python_file_maybe_hyphen}"
  )
  string(REPLACE "-" "_" rel_generated_python_file "${rel_generated_python_file_maybe_hyphen}")
  set(generated_python_file "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/${rel_generated_python_file}")
  get_filename_component(generated_python_file_dir "${generated_python_file}" DIRECTORY)
  list(APPEND python_package_directories "${generated_python_file_dir}")
endforeach()
list(REMOVE_DUPLICATES python_package_directories)
set(python_init_py_files "")
# add __init__.py to "leaf" directories
foreach(python_package_directory ${python_package_directories})
  file(MAKE_DIRECTORY "${python_package_directory}")
  set(python_init_py_file "${python_package_directory}/__init__.py")
  list(APPEND python_init_py_files "${python_init_py_file}")
  file(TOUCH "${python_init_py_file}")
endforeach()
# second pass to add __init__.py to intermediate directories now that the folder structure exists.
file(GLOB_RECURSE leaf_python_init_files_and_dirs
  LIST_DIRECTORIES true
  RELATIVE "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/"
  "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/*/__init__.py"
)
foreach(leaf_python_init_file_or_dir ${leaf_python_init_files_and_dirs})
  if(IS_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/${leaf_python_init_file_or_dir}")
    set(python_init_py_file "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/${leaf_python_init_file_or_dir}/__init__.py")
    list(APPEND python_init_py_files "${python_init_py_file}")
    file(TOUCH "${python_init_py_file}")
  endif()
endforeach()
# final pass because glob doesn't get the root folders correctly.
file(GLOB root_directories
  LIST_DIRECTORIES true
  "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/*"
)
foreach(root_directory ${root_directories})
  if(IS_DIRECTORY "${root_directory}")
    set(python_init_py_file "${root_directory}/__init__.py")
    list(APPEND python_init_py_files "${python_init_py_file}")
    file(TOUCH "${python_init_py_file}")
  endif()
endforeach()
list(REMOVE_DUPLICATES python_init_py_files)
add_custom_target(intrinsic_sdk_protos_python ALL DEPENDS ${sdk_protos_python_sources} ${python_init_py_files})
# Generated Python code is installed with the Python sdk files later.

# Generate a descriptor set.
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/intrinsic_proto.desc
  COMMAND protobuf::protoc
  ARGS
    -I${intrinsic_sdk_SOURCE_DIR}
    -I${googleapis_SOURCE_DIR}
    -I${grpc_gateway_SOURCE_DIR}
    -I${grpc_SOURCE_DIR}
    --include_imports
    --experimental_editions
    --descriptor_set_out=${CMAKE_CURRENT_BINARY_DIR}/intrinsic_proto.desc
    ${intrinsic_proto_SRCS}
  DEPENDS ${protobuf_PROTOC_EXE} ${intrinsic_proto_SRCS}
  COMMENT "Generating descriptor set"
  VERBATIM
)
add_custom_target(intrinsic_proto_desc ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/intrinsic_proto.desc)

# Install generated C++ proto code.
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/protos_gen/"
  DESTINATION "include/${PROJECT_NAME}"
  FILES_MATCHING # install only matched files
  PATTERN "*.h" # select header files
)

# Install the descriptor file.
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/intrinsic_proto.desc"
  DESTINATION "share/${PROJECT_NAME}"
)

# Install the libraries.
install(
  TARGETS
    intrinsic_sdk_protos
    intrinsic_sdk_services
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the proto files.
install(
  DIRECTORY "${CMAKE_SOURCE_DIR}/sdk/intrinsic" # source directory
  DESTINATION "share/${PROJECT_NAME}/protos" # target directory
  FILES_MATCHING # install only matched files
  PATTERN "*.proto" # select header files
)
