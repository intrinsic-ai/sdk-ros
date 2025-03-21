# TODO(wjwwood): a few dirty hacks here to get tools we need out of the sdk.
#   This wouldn't be necessary with inbuild, so replace with that when possible.
#   This also only works because these are static binaries with no dependencies.
#   Also, it does not capture stderr from the bazel build, which is noisy.
#   Also, probably other significant issues I'm overlooking, but this works for now.

# Build the static binary for generating service bundles.
set(sdk_bins_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk_bins")
file(MAKE_DIRECTORY "${sdk_bins_DIR}")
add_custom_command(
  OUTPUT "${sdk_bins_DIR}/skillbundlegen"
  WORKING_DIRECTORY "${intrinsic_sdk_SOURCE_DIR}/intrinsic"
  COMMAND
    ${bazelisk_vendor_EXECUTABLE}
      --nohome_rc
      --quiet
      run
        --experimental_convenience_symlinks=ignore
        --run_under=cp
        //intrinsic/skills/build_defs:skillbundlegen
        "${sdk_bins_DIR}/skillbundlegen"
  VERBATIM
)
add_custom_target(skillbundlegen
  ALL
  DEPENDS
    "${sdk_bins_DIR}/skillbundlegen"
)
install(
  PROGRAMS
    "${sdk_bins_DIR}/skillbundlegen"
  DESTINATION bin
)
# Create an imported executable and namespace it to imitate find_package()
add_executable(skillbundlegen_import IMPORTED)
set_target_properties(skillbundlegen_import
  PROPERTIES
    IMPORTED_LOCATION "${sdk_bins_DIR}/skillbundlegen"
)
add_dependencies(skillbundlegen_import skillbundlegen)
add_executable(${PROJECT_NAME}::skillbundlegen ALIAS skillbundlegen_import)

# Build the static binary for generating service configs.
add_custom_command(
  OUTPUT "${sdk_bins_DIR}/skillserviceconfiggen_main"
  WORKING_DIRECTORY "${intrinsic_sdk_SOURCE_DIR}/intrinsic"
  COMMAND
    ${bazelisk_vendor_EXECUTABLE}
      --nohome_rc
      --quiet
      run
        --experimental_convenience_symlinks=ignore
        --run_under=cp
        //intrinsic/skills/build_defs:skillserviceconfiggen_main
        "${sdk_bins_DIR}/skillserviceconfiggen_main"
  VERBATIM
)
add_custom_target(skillserviceconfiggen_main
  ALL
  DEPENDS
    "${sdk_bins_DIR}/skillserviceconfiggen_main"
)
install(
  PROGRAMS
    "${sdk_bins_DIR}/skillserviceconfiggen_main"
  DESTINATION bin
)
# Create an imported executable and namespace it to imitate find_package()
add_executable(skillserviceconfiggen_main_import IMPORTED)
set_target_properties(skillserviceconfiggen_main_import
  PROPERTIES
    IMPORTED_LOCATION "${sdk_bins_DIR}/skillserviceconfiggen_main"
)
add_dependencies(skillserviceconfiggen_main_import skillserviceconfiggen_main)
add_executable(${PROJECT_NAME}::skillserviceconfiggen_main ALIAS skillserviceconfiggen_main_import)

# Build the static binary for generating service main files.
add_custom_command(
  OUTPUT "${sdk_bins_DIR}/skill_service_generator"
  WORKING_DIRECTORY "${intrinsic_sdk_SOURCE_DIR}/intrinsic"
  COMMAND
    ${bazelisk_vendor_EXECUTABLE}
      --nohome_rc
      --quiet
      run
        --experimental_convenience_symlinks=ignore
        --run_under=cp
        //intrinsic/skills/generator:skill_service_generator
        "${sdk_bins_DIR}/skill_service_generator"
  VERBATIM
)
add_custom_target(skill_service_generator
  ALL
  DEPENDS
    "${sdk_bins_DIR}/skill_service_generator"
)
install(
  PROGRAMS
    "${sdk_bins_DIR}/skill_service_generator"
  DESTINATION bin
)
# Create an imported executable and namespace it to imitate find_package()
add_executable(skill_service_generator_import IMPORTED)
set_target_properties(skill_service_generator_import
  PROPERTIES
    IMPORTED_LOCATION "${sdk_bins_DIR}/skill_service_generator"
)
add_dependencies(skill_service_generator_import skill_service_generator)
add_executable(${PROJECT_NAME}::skill_service_generator ALIAS skill_service_generator_import)
