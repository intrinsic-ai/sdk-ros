# Through a complex process, the "imw" library is compiled elsewhere and
# committed to the SDK as a binary, but only amd64.
if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(intrinsic_sdk_IMW_ZENOH_LIBRARY_DIRNAME
    "intrinsic/insrc/middleware")
  set(intrinsic_sdk_IMW_ZENOH_LIBRARY_RELPATH
    "${intrinsic_sdk_IMW_ZENOH_LIBRARY_DIRNAME}/libimw_zenoh.so.1")
  set(intrinsic_sdk_IMW_ZENOH_LIBRARY_PATH
    "${intrinsic_sdk_SOURCE_DIR}/${intrinsic_sdk_IMW_ZENOH_LIBRARY_RELPATH}")
else()
  message(FATAL_ERROR "Only x86_64 machines are supported, found: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
endif()

set(intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME
  "intrinsic/platform/pubsub/zenoh_util")
set(intrinsic_sdk_IMW_ZENOH_CONFIG_RELPATH
  "${intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME}/peer_config.json")
set(intrinsic_sdk_IMW_ZENOH_CONFIG_PATH
  "${intrinsic_sdk_SOURCE_DIR}/${intrinsic_sdk_IMW_ZENOH_CONFIG_RELPATH}")

# Install the files for use during runtime.
install(
  FILES "${intrinsic_sdk_IMW_ZENOH_LIBRARY_PATH}"
  DESTINATION "lib/${intrinsic_sdk_IMW_ZENOH_LIBRARY_DIRNAME}"
)
install(
  FILES "${intrinsic_sdk_IMW_ZENOH_CONFIG_PATH}"
  DESTINATION "share/${intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME}"
)
