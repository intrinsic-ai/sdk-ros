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

#
# Generate the C++ main function file for the skill.
#
# This file needs to be generated because it contains variable things like the
# skill name and the name of the skill's header, etc.
#
# :param MANIFEST_PBBIN: the path to the pbbin file for the skill manifest.
#   This can be generated using intrinsic_sdk_generate_skill_manifest_pbbin().
# :type MANIFEST_PBBIN: string
# :param HEADER_FILES: the paths to additional header files that need to be included
# :type HEADER_FILES: list of strings
# :param MAIN_FILE_OUTPUT: the path of the output C++ main function file
# :type MAIN_FILE_OUTPUT: string
#
# @public
#
function(intrinsic_sdk_generate_skill_main_cc)
  set(options)
  set(one_value_args MANIFEST_PBBIN MAIN_FILE_OUTPUT)
  set(multi_value_args HEADER_FILES)

  cmake_parse_arguments(
    arg
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  list(JOIN arg_HEADER_FILES "," joined_header_files)

  add_custom_command(
    OUTPUT ${arg_MAIN_FILE_OUTPUT}
    COMMAND intrinsic_sdk_cmake::skill_service_generator
      --manifest=${arg_MANIFEST_PBBIN}
      --lang=cpp
      --out=${arg_MAIN_FILE_OUTPUT}
      --cc_headers=${joined_header_files}
    DEPENDS ${arg_MANIFEST_PBBIN}
    COMMENT "Generating skill cpp main file: ${arg_MAIN_FILE_OUTPUT}"
  )
endfunction()
