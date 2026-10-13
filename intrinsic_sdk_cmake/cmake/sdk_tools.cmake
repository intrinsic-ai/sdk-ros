# TODO(wjwwood): a few dirty hacks here to get tools we need out of the sdk.
#   This wouldn't be necessary with inbuild, so replace with that when possible.
#   This also only works because these are static binaries with no dependencies.
#   Also, it does not capture stderr from the bazel build, which is noisy.
#   Also, probably other significant issues I'm overlooking, but this works for now.
#   Edit: now we're just getting inbuild, but we can completely remove this stuff
#   once we decide on how to distribute inbuild.

set(sdk_bins_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk_bins")
file(MAKE_DIRECTORY "${sdk_bins_DIR}")

if(INTRINSIC_SDK_CMAKE_BUILD_INBUILD)
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
else()
  # Download the inbuild binary released alongside the pinned SDK version (sdk_version, from fetch_sdk.cmake).
  # Upstream SDK releases currently publish only inbuild-linux-amd64.
  if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
    set(_inbuild_url
      "https://github.com/intrinsic-ai/sdk/releases/download/${sdk_version}/inbuild-linux-amd64")
  else()
    message(FATAL_ERROR
      "No released inbuild binary for ${CMAKE_HOST_SYSTEM_NAME}/${CMAKE_HOST_SYSTEM_PROCESSOR} "
      "(only linux-amd64 is published in intrinsic-ai/sdk releases). "
      "Set -DINTRINSIC_SDK_CMAKE_BUILD_INBUILD=ON to build it from source.")
  endif()
  # Stamp the downloaded binary with its SDK version so a version bump re-downloads it in existing build trees.
  set(_inbuild_version_file "${sdk_bins_DIR}/inbuild.version")
  set(_inbuild_cached_version "")
  if(EXISTS "${_inbuild_version_file}")
    file(READ "${_inbuild_version_file}" _inbuild_cached_version)
  endif()
  if(NOT EXISTS "${sdk_bins_DIR}/inbuild" OR NOT _inbuild_cached_version STREQUAL sdk_version)
    message(STATUS "Downloading inbuild from ${_inbuild_url}")
    # Download to a temporary name so a failed or interrupted download is never mistaken for a complete one.
    file(DOWNLOAD "${_inbuild_url}" "${sdk_bins_DIR}/inbuild.part" STATUS _inbuild_status INACTIVITY_TIMEOUT 60)
    list(GET _inbuild_status 0 _inbuild_code)
    if(NOT _inbuild_code EQUAL 0)
      list(GET _inbuild_status 1 _inbuild_msg)
      file(REMOVE "${sdk_bins_DIR}/inbuild.part")
      message(FATAL_ERROR
        "Failed to download inbuild from ${_inbuild_url}: ${_inbuild_msg}. "
        "Set INTRINSIC_SDK_CMAKE_BUILD_INBUILD=ON to build it from source.")
    endif()
    file(CHMOD "${sdk_bins_DIR}/inbuild.part"
      PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
    file(RENAME "${sdk_bins_DIR}/inbuild.part" "${sdk_bins_DIR}/inbuild")
    file(WRITE "${_inbuild_version_file}" "${sdk_version}")
  endif()
  # Keep the target so add_dependencies() below works the same in both modes.
  add_custom_target(inbuild)
endif()
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
