# Glob the source files and then exclude files that don't make sense to be in the glob.
# Note(wjwwood): we know this isn't the "right" way to do this, but it helps us catch
# additions to the sdk when it is upgraded until we have a more structured approach
# to extract build targets out of bazel.
file(GLOB_RECURSE intrinsic_SRCS "${intrinsic_sdk_SOURCE_DIR}/**/*.cc")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "_test\\.cc$")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "_test_utils\\.cc$")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "_main\\.cc$")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "_tmpl\\.cc$")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/examples/")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/skills/testing/")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/release/hello_world.cc")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/release/reset_simulation.cc")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/tools")
# TODO(wjwwood): figure out the compiler errors here and re-add these icon files when possible.
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/control/c_api")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/hal")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/hardware_modules")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/utils/realtime_metrics.cc")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/utils/metrics_logger.cc")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/interprocess/remote_trigger")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/interprocess/shared_memory_lockstep")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/icon/interprocess/shared_memory_manager")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/platform/pubsub/python/")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/tools")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/util/proto/source_code_info_view_py.cc")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/util/path_resolver/")
list(FILTER intrinsic_SRCS EXCLUDE REGEX "/intrinsic/scene/product/")

add_library(${PROJECT_NAME} SHARED ${intrinsic_SRCS})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${intrinsic_sdk_SOURCE_DIR}>"
  # Add the directory where fbs headers are generated
  "$<BUILD_INTERFACE:${intrinsic_fbs_gen_dir}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    absl::base
    absl::flags_parse
    absl::flags_usage
    absl::log
    absl::log_internal_check_op
    absl::time
    Eigen3::Eigen
    flatbuffers::flatbuffers
    gRPC::grpc++
    ortools::ortools
    opencensus-cpp::stats
    protobuf::libprotobuf
    pybind11::pybind11
    pybind11_abseil::absl_casters
    Python::Python
    # Local targets
    intrinsic_sdk_protos
    intrinsic_sdk_services
)
# TODO(wjwwood): figure out why this is needed
#   I did this to fix a linker error in dependent packages, see:
#   https://zhangboyi.gitlab.io/post/2020-09-14-resolve-dso-missing-from-command-line-error/
target_link_options(${PROJECT_NAME} INTERFACE "-Wl,--copy-dt-needed-entries")
add_dependencies(${PROJECT_NAME} intrinsic_sdk_fbs)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
# TODO(wjwwood): this is a bit fragile because it only works in the install case
#   we could use something like ament_index to make it relocatable
target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    -DINTRINSIC_SDK_CMAKE_LIB_PATH="${CMAKE_INSTALL_PREFIX}/lib"
    -DINTRINSIC_SDK_CMAKE_SHARE_PATH="${CMAKE_INSTALL_PREFIX}/share"
)

# Install the library.
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the sdk headers.
install(
  DIRECTORY "${intrinsic_sdk_SOURCE_DIR}/intrinsic"
  DESTINATION "include/${PROJECT_NAME}"
  FILES_MATCHING
  PATTERN "*.h"
)
