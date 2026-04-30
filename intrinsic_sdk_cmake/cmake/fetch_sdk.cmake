# Fetch the sdk and make it available for use locally.

find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('intrinsic_sdk_bundle_library_py'))"
  OUTPUT_VARIABLE intrinsic_sdk_bundle_library_py_SHARE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(SDK_VERSION_JSON_FILE "${intrinsic_sdk_bundle_library_py_SHARE_DIR}/resource/sdk_version.json")
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
  PATCH_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/apply_patch.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/001_zenoh_helpers_cc_no_runfiles.patch
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/002_gcc13_publisher_noexcept.patch
)
FetchContent_GetProperties(intrinsic_sdk)
if(NOT intrinsic_sdk_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_MakeAvailable(intrinsic_sdk)
endif()
