# TODO(wjwwood): a few dirty hacks here to get tools we need out of the sdk.
#   This wouldn't be necessary with inbuild, so replace with that when possible.
#   This also only works because these are static binaries with no dependencies.
#   Also, it does not capture stderr from the bazel build, which is noisy.
#   Also, probably other significant issues I'm overlooking, but this works for now.
#   Edit: now we're just getting inbuild, but we can completely remove this stuff
#   once we decide on how to distribute inbuild.

set(sdk_bins_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk_bins")

# Build inbuild
add_custom_command(
  OUTPUT "${sdk_bins_DIR}/inbuild"
  WORKING_DIRECTORY "${intrinsic_sdk_SOURCE_DIR}/intrinsic"
  COMMAND
    ${bazelisk_vendor_EXECUTABLE}
      --nohome_rc
      --quiet
      run
        --experimental_convenience_symlinks=ignore
        --run_under=cp
        //intrinsic/tools/inbuild
        "${sdk_bins_DIR}/inbuild"
  VERBATIM
)
add_custom_target(inbuild
  ALL
  DEPENDS
    "${sdk_bins_DIR}/inbuild"
)
install(
  PROGRAMS
    "${sdk_bins_DIR}/inbuild"
  DESTINATION bin
)
# Create an imported executable and namespace it to imitate find_package()
add_executable(inbuild_import IMPORTED)
set_target_properties(inbuild_import
  PROPERTIES
    IMPORTED_LOCATION "${sdk_bins_DIR}/inbuild"
)
add_dependencies(inbuild_import inbuild)
add_executable(${PROJECT_NAME}::inbuild ALIAS inbuild_import)
