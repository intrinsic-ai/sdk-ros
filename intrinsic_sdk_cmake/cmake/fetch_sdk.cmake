# Fetch the sdk and make it available for use locally.

include(FetchContent)
# Fetch the intrinsic sdk source code during configure stage.
FetchContent_Declare(
  intrinsic_sdk
  URL https://github.com/intrinsic-ai/sdk/archive/refs/tags/v1.16.20250303.tar.gz
  URL_HASH SHA256=2181ae09684bc35e47b251e21d969a4e0f3cbf72818470caaa416e2834748d32
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
)
FetchContent_GetProperties(intrinsic_sdk)
if(NOT intrinsic_sdk_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_Populate(intrinsic_sdk)
endif()
