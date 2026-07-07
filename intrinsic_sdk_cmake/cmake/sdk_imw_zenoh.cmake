set(intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME
  "intrinsic/platform/pubsub/zenoh_util")
set(intrinsic_sdk_IMW_ZENOH_CONFIG_RELPATH
  "${intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME}/peer_config.json")
set(intrinsic_sdk_IMW_ZENOH_CONFIG_PATH
  "${intrinsic_sdk_SOURCE_DIR}/${intrinsic_sdk_IMW_ZENOH_CONFIG_RELPATH}")

# Install the files for use during runtime.
if(EXISTS "${intrinsic_sdk_IMW_ZENOH_CONFIG_PATH}")
  install(
    FILES "${intrinsic_sdk_IMW_ZENOH_CONFIG_PATH}"
    DESTINATION "share/${intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME}"
  )
endif()
