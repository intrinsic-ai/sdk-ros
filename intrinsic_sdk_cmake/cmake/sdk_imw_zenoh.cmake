file(GLOB imw_zenoh_SRCS "${intrinsic_sdk_SOURCE_DIR}/intrinsic/middleware/zenoh/*.cc")
add_library(imw_zenoh SHARED ${imw_zenoh_SRCS})
add_library(${PROJECT_NAME}::imw_zenoh ALIAS imw_zenoh)
set_target_properties(imw_zenoh PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(imw_zenoh PRIVATE
  "$<BUILD_INTERFACE:${intrinsic_sdk_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${intrinsic_fbs_gen_dir}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
find_package(zenoh_cpp_vendor REQUIRED)
target_link_libraries(imw_zenoh PRIVATE
  absl::base
  absl::log
  absl::status
  absl::statusor
  absl::strings
  absl::synchronization
  gz-transport::gz-transport
  zenohc::lib
)

set(intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME
  "intrinsic/platform/pubsub/zenoh_util")
set(intrinsic_sdk_IMW_ZENOH_CONFIG_RELPATH
  "${intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME}/peer_config.json")
set(intrinsic_sdk_IMW_ZENOH_CONFIG_PATH
  "${intrinsic_sdk_SOURCE_DIR}/${intrinsic_sdk_IMW_ZENOH_CONFIG_RELPATH}")

# Install the files for use during runtime.
install(
  TARGETS imw_zenoh
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
if(EXISTS "${intrinsic_sdk_IMW_ZENOH_CONFIG_PATH}")
  install(
    FILES "${intrinsic_sdk_IMW_ZENOH_CONFIG_PATH}"
    DESTINATION "share/${intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME}"
  )
endif()
