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

include_guard(GLOBAL)

include("${intrinsic_sdk_cmake_DIR}/cmake/api/intrinsic_sdk_protobuf_generate.cmake")
include("${intrinsic_sdk_cmake_DIR}/cmake/api/skill/intrinsic_sdk_generate_skill_config.cmake")

#
# Generate all the components of an Intrinsic Skill and bundle them into an
# asset that can be installed into the Intrinsic Platform.
#
# :param SKILL_NAME: the name of the skill
# :type SKILL_NAME: string
# :param MANIFEST: the path to the manifest file
# :type MANIFEST: string
# :param PROTOS_TARGET: the cmake target for generating the skill's proto files
# :type PROTOS_TARGET: cmake target
# :param SOURCES: the list of source files for the skill target
# :type SOURCES: list of strings
#
# @public
#
function(intrinsic_sdk_generate_skill)
  set(options)
  set(one_value_args SKILL_NAME MANIFEST PROTOS_TARGET)
  set(multi_value_args SOURCES)

  cmake_parse_arguments(
    GENERATE_ARGS
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN})

  intrinsic_sdk_protobuf_generate(
    NAME ${GENERATE_ARGS_SKILL_NAME}
    SOURCES ${GENERATE_ARGS_SOURCES}
    TARGET ${GENERATE_ARGS_PROTOS_TARGET}
  )
  intrinsic_sdk_generate_skill_config(
    SKILL_NAME ${GENERATE_ARGS_SKILL_NAME}
    MANIFEST ${GENERATE_ARGS_MANIFEST}
  )
endfunction()
