# Build inbuild from the SDK
set(sdk_bins_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk_bins")
file(MAKE_DIRECTORY "${sdk_bins_DIR}")
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