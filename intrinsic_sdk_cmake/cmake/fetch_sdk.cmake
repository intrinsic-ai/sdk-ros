# Fetch the sdk and make it available for use locally.

set(SDK_VERSION_JSON_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_version.json")
file(READ "${SDK_VERSION_JSON_FILE}" sdk_version_json)

string(JSON sdk_version GET ${sdk_version_json} "sdk_version")
string(JSON sdk_checksum GET ${sdk_version_json} "sdk_checksum")
message(STATUS "intrinsic-ai/sdk version: ${sdk_version} ${sdk_checksum}")

set(TOOLS_DIR ${CMAKE_BINARY_DIR}/bin)
file(MAKE_DIRECTORY ${TOOLS_DIR})

string(JSON inctl_version GET ${sdk_version_json} "inctl_version")
string(JSON inctl_checksum GET ${sdk_version_json} "inctl_checksum")
message(STATUS "inctl version: ${inctl_version} ${inctl_checksum}")
set(INCTL_URL https://github.com/intrinsic-ai/sdk/releases/download/${inctl_version}/inctl-linux-amd64)
set(INCTL_DEST ${TOOLS_DIR}/inctl)

string(JSON inbuild_version GET ${sdk_version_json} "inbuild_version")
string(JSON inbuild_checksum GET ${sdk_version_json} "inbuild_checksum")
message(STATUS "inbuild version: ${inbuild_version} ${inbuild_checksum}")
set(INBUILD_URL https://github.com/intrinsic-ai/sdk/releases/download/${inctl_version}/inbuild-linux-amd64)
set(INBUILD_DEST ${TOOLS_DIR}/inbuild)

include(FetchContent)
# Fetch the intrinsic sdk source code during configure stage.
FetchContent_Declare(
  intrinsic_sdk
  URL https://github.com/intrinsic-ai/sdk/archive/refs/tags/${sdk_version}.tar.gz
  URL_HASH ${sdk_checksum}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE 
  PATCH_COMMAND git apply --ignore-space-change --ignore-whitespace 
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/0001-Remove-use-of-runfiles-from-the-zenoh-helpers.patch"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/0002-Vendor-generated-strong_int.h-file-for-amd64.patch"
)
FetchContent_GetProperties(intrinsic_sdk)
if(NOT intrinsic_sdk_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_MakeAvailable(intrinsic_sdk)
endif()

add_custom_target(
  download_inctl
  COMMAND ${CMAKE_COMMAND} -E echo "Downloading inctl..."
  COMMAND curl -L -o ${INCTL_DEST} ${INCTL_URL}
  COMMAND ${CMAKE_COMMAND} -E touch ${TOOLS_DIR}/inctl_stamp.txt 
  COMMENT "Downloading and verifying inctl tool"
  VERBATIM
)

add_custom_target(
  download_inbuild
  COMMAND ${CMAKE_COMMAND} -E echo "Downloading inbuild..."
  COMMAND curl -L -o ${INBUILD_DEST} ${INBUILD_URL}
  COMMAND ${CMAKE_COMMAND} -E touch ${TOOLS_DIR}/inbuild_stamp.txt 
  COMMENT "Downloading and verifying inctl tool"
  VERBATIM
)

add_custom_target(download_tools COMMENT "Downloading all tools")
add_dependencies(download_tools download_inctl download_inbuild)

install(
  PROGRAMS ${INCTL_DEST}
  DESTINATION bin
)

install(
  PROGRAMS ${INBUILD_DEST}
  DESTINATION bin
)
