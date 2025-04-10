# Fetch the sdk and make it available for use locally.

include(FetchContent)
# Fetch the intrinsic sdk source code during configure stage.
FetchContent_Declare(
  intrinsic_sdk
  URL https://github.com/intrinsic-ai/sdk/archive/refs/tags/v1.16.20250210.tar.gz
  URL_HASH SHA256=be55bdb2c761b7d55476c2ee843c4e444e0953f03aafce2481797eec98e19ac0
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
)
FetchContent_GetProperties(intrinsic_sdk)
if(NOT intrinsic_sdk_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_Populate(intrinsic_sdk)
endif()
