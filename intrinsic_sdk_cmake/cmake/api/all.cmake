# Copyright 2025 Intrinsic Innovation LLC
#
# You are hereby granted a non-exclusive, worldwide, royalty-free license to use,
# copy, modify, and distribute this Intrinsic SDK in source code or binary form for use
# in connection with the services and APIs provided by Intrinsic Innovation LLC (“Intrinsic”).
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# If you use this Intrinsic SDK with any Intrinsic services, your use is subject to the Intrinsic
# Platform Terms of Service [https://intrinsic.ai/platform-terms].  If you create works that call
# Intrinsic APIs, you must agree to the terms of service for those APIs separately. This license
# does not grant you any special rights to use the services.
#
# This copyright notice shall be included in all copies or substantial portions of the software.

# prevent multiple inclusion
if(DEFINED _INTRINSIC_SDK_CMAKE_CMAKE_API_ALL_INCLUDED)
  message(FATAL_ERROR "intrinsic_sdk_cmake/cmake/api/all.cmake included multiple times")
endif()
set(_INTRINSIC_SDK_CMAKE_CMAKE_API_ALL_INCLUDED TRUE)

if(NOT DEFINED intrinsic_sdk_cmake_DIR)
  message(FATAL_ERROR "intrinsic_sdk_cmake_DIR is unexpectedly not set")
endif()

if(NOT DEFINED intrinsic_sdk_cmake_API_DIR)
  set(intrinsic_sdk_cmake_API_DIR
    "${intrinsic_sdk_cmake_DIR}/../../../share/intrinsic_sdk_cmake/cmake/api")
endif()

# Protobuf generation API
include("${intrinsic_sdk_cmake_API_DIR}/intrinsic_sdk_protobuf_generate.cmake")

# Skill APIs
include("${intrinsic_sdk_cmake_API_DIR}/skill/intrinsic_sdk_generate_skill.cmake")
include("${intrinsic_sdk_cmake_API_DIR}/skill/intrinsic_sdk_generate_skill_config.cmake")
include("${intrinsic_sdk_cmake_API_DIR}/skill/intrinsic_sdk_generate_skill_container_image.cmake")
include("${intrinsic_sdk_cmake_API_DIR}/skill/intrinsic_sdk_generate_skill_main_cc.cmake")
include("${intrinsic_sdk_cmake_API_DIR}/skill/intrinsic_sdk_generate_skill_bundle.cmake")

# Service APIs
include("${intrinsic_sdk_cmake_API_DIR}/service/intrinsic_sdk_generate_service_manifest.cmake")
