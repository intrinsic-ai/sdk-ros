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
# :param MANIFEST: the path to the skill manifest in textproto format.
# :type MANIFEST: string
# :param HEADER_FILE: the path to the header file containing the
#     "create skill" method referenced in the Skill Manifest.
# :type HEADER_FILE: lstring
# :param MAIN_FILE_OUTPUT: the path of the output C++ main function file
# :type MAIN_FILE_OUTPUT: string
#
# @public
#
function(intrinsic_sdk_generate_skill_main_cc)
  set(options)
  set(one_value_args MANIFEST_PBBIN MAIN_FILE_OUTPUT HEADER_FILE)
  set(multi_value_args)

  cmake_parse_arguments(
    arg
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  add_custom_command(
    OUTPUT ${arg_MAIN_FILE_OUTPUT}
    COMMAND inbuild_import
      skill generate entrypoint
      --manifest=${arg_MANIFEST_PBBIN}
      --language=cpp
      --output=${arg_MAIN_FILE_OUTPUT}
      --cc_header=${arg_HEADER_FILE}
    DEPENDS ${arg_MANIFEST_PBBIN}
    COMMENT "Generating skill cpp main file: ${arg_MAIN_FILE_OUTPUT}"
  )
endfunction()
