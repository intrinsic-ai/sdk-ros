# Fetch the sdk and make it available for use locally.

set(SDK_VERSION_JSON_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_version.json")
file(READ "${SDK_VERSION_JSON_FILE}" sdk_version_json)
string(JSON sdk_version GET ${sdk_version_json} "sdk_version")
message(STATUS "intrinsic-ai/sdk version: ${sdk_version}")

include(FetchContent)
# Fetch the intrinsic sdk source code during configure stage.
FetchContent_Declare(
  intrinsic_sdk
  URL https://github.com/intrinsic-ai/sdk/archive/refs/tags/${sdk_version}.tar.gz
  URL_HASH SHA256=5a2a57655816561967852229cd3c8d02a9942ea6c65e2ee2d8f63c2b6eb7025e
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  PATCH_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/apply_patch.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/001_zenoh_helpers_cc_no_runfiles.patch
)
FetchContent_GetProperties(intrinsic_sdk)
if(NOT intrinsic_sdk_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_Populate(intrinsic_sdk)
endif()
