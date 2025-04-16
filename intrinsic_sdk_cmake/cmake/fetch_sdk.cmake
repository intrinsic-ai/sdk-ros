# Fetch the sdk and make it available for use locally.

include(FetchContent)
# Fetch the intrinsic sdk source code during configure stage.
FetchContent_Declare(
  intrinsic_sdk
  URL https://github.com/intrinsic-ai/sdk/archive/refs/tags/v1.17.20250331.tar.gz
  URL_HASH SHA256=0de09dec1340474c861b6b288e4bc142300ac277031b31aa09e1cebc32ea9f5b
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  PATCH_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/apply_patch.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk_patches/001_zenoh_helpers_cc_no_runfiles.patch
)
FetchContent_GetProperties(intrinsic_sdk)
if(NOT intrinsic_sdk_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_Populate(intrinsic_sdk)
endif()
