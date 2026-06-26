set(sdk_bins_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk_bins")
file(MAKE_DIRECTORY "${sdk_bins_DIR}")

# Build libimw_zenoh.so using Bazel.
add_custom_command(
  OUTPUT "${sdk_bins_DIR}/libimw_zenoh.so"
  WORKING_DIRECTORY "${intrinsic_sdk_SOURCE_DIR}"
  COMMAND
    ${bazelisk_vendor_EXECUTABLE}
      --nohome_rc
      --quiet
      build
        --experimental_convenience_symlinks=ignore
        //incode/middleware/zenoh:libimw_zenoh.so
  COMMAND
    bash -c "cp \$(${bazelisk_vendor_EXECUTABLE} --nohome_rc info bazel-bin)/incode/middleware/zenoh/libimw_zenoh.so ${sdk_bins_DIR}/libimw_zenoh.so"
  VERBATIM
)

add_custom_target(imw_zenoh_bin
  ALL
  DEPENDS
    "${sdk_bins_DIR}/libimw_zenoh.so"
)

# Create an imported shared library
add_library(imw_zenoh SHARED IMPORTED)
set_target_properties(imw_zenoh PROPERTIES
  IMPORTED_LOCATION "${sdk_bins_DIR}/libimw_zenoh.so"
)
add_dependencies(imw_zenoh imw_zenoh_bin)

set(intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME
  "intrinsic/platform/pubsub/zenoh_util")
set(intrinsic_sdk_IMW_ZENOH_CONFIG_RELPATH
  "${intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME}/peer_config.json")
set(intrinsic_sdk_IMW_ZENOH_CONFIG_PATH
  "${intrinsic_sdk_SOURCE_DIR}/${intrinsic_sdk_IMW_ZENOH_CONFIG_RELPATH}")

# Install the files for use during runtime.
install(
  FILES "${sdk_bins_DIR}/libimw_zenoh.so"
  DESTINATION lib
)
install(
  FILES "${intrinsic_sdk_IMW_ZENOH_CONFIG_PATH}"
  DESTINATION "share/${intrinsic_sdk_IMW_ZENOH_CONFIG_DIRNAME}"
)
