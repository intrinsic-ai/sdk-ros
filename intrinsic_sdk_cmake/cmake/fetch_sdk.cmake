# Fetch the sdk and make it available for use locally.

set(SDK_VERSION_JSON_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_version.json")
file(READ "${SDK_VERSION_JSON_FILE}" sdk_version_json)
string(JSON sdk_version GET ${sdk_version_json} "sdk_version")
string(JSON sdk_checksum GET ${sdk_version_json} "sdk_checksum")
message(STATUS "intrinsic-ai/sdk version: ${sdk_version} ${sdk_checksum}")

include(FetchContent)
# Fetch the intrinsic sdk source code during configure stage.
FetchContent_Declare(
  intrinsic_sdk
  URL https://github.com/intrinsic-ai/sdk/archive/refs/tags/${sdk_version}.tar.gz
  URL_HASH ${sdk_checksum}
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  PATCH_COMMAND git apply --ignore-space-change --ignore-whitespace 
    "sdk_patches/0001-Remove-use-of-runfiles-from-the-zenoh-helpers.patch"
    "sdk_patches/0002-Vendor-generated-strong_int.h-file-for-amd64.patch"
)
FetchContent_GetProperties(intrinsic_sdk)
if(NOT intrinsic_sdk_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_MakeAvailable(intrinsic_sdk)
endif()

