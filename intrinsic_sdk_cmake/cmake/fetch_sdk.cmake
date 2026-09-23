# Fetch the sdk and make it available for use locally.

set(sdk_version "0.dev1")

#set(SDK_VERSION_JSON_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_version.json")
#file(READ "${SDK_VERSION_JSON_FILE}" sdk_version_json)
#string(JSON sdk_version GET ${sdk_version_json} "sdk_version")
#string(JSON sdk_checksum GET ${sdk_version_json} "sdk_checksum")
#message(STATUS "intrinsic-ai/sdk version: ${sdk_version} ${sdk_checksum}")

include(FetchContent)
# Fetch the intrinsic sdk source code during configure stage.
#FetchContent_Declare(
#  intrinsic_sdk
#  URL https://github.com/intrinsic-ai/sdk/archive/refs/tags/${sdk_version}.tar.gz
#  URL_HASH ${sdk_checksum}
#  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
#  PATCH_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/apply_patch.sh
#    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/001_zenoh_helpers_cc_no_runfiles.patch
#)

# Override with local path
FetchContent_Declare(
    intrinsic_sdk
    SOURCE_DIR "/usr/local/google/home/shameek/sdk"
    PATCH_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/apply_patch.sh
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/001_zenoh_helpers_cc_no_runfiles.patch
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/002_gcc13_publisher_noexcept.patch
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/003_icon_strerror_glibc2_39.patch
)

FetchContent_GetProperties(intrinsic_sdk)
if(NOT intrinsic_sdk_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_MakeAvailable(intrinsic_sdk)
endif()
