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
# Generate the skill config file for the skill.
#
# :param TARGET: the name for the target that generates the skill config file
# :type TARGET: string
# :param SKILL_NAME: the name of the skill
# :type SKILL_NAME: string
# :param MANIFEST_PBBIN: the path to the manifest file
# :type MANIFEST_PBBIN: string
# :param PROTO_DESCRIPTOR_FILE: the path to the proto descriptor file
# :type PROTO_DESCRIPTOR_FILE: string
# :param SKILL_CONFIG_FILE_OUTPUT: the output path for the skill config file
# :type SKILL_CONFIG_FILE_OUTPUT: string
#
# @public
#
function(intrinsic_sdk_generate_skill_config)
  set(options)
  set(one_value_args
    TARGET
    # TODO(wjwwood): extract skill name from manifest
    SKILL_NAME
    MANIFEST_PBBIN
    PROTO_DESCRIPTOR_FILE
    SKILL_CONFIG_FILE_OUTPUT
  )
  set(multi_value_args)

  cmake_parse_arguments(
    arg
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  set(OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

  # Generate the binary proto skill config
  add_custom_command(
    OUTPUT ${arg_SKILL_CONFIG_FILE_OUTPUT}
    COMMAND intrinsic_sdk_cmake::skillserviceconfiggen_main
    ARGS
      --manifest_pbbin_filename=${arg_MANIFEST_PBBIN}
      --proto_descriptor_filename=${arg_PROTO_DESCRIPTOR_FILE}
      --output_config_filename=${arg_SKILL_CONFIG_FILE_OUTPUT}
    COMMENT "Generating skill config for ${arg_SKILL_NAME}"
    DEPENDS
      ${arg_SKILL_CONFIG_FILE_OUTPUT}
      ${arg_PROTO_DESCRIPTOR_FILE}
  )

  add_custom_target(${arg_TARGET}
    DEPENDS
      ${arg_SKILL_CONFIG_FILE_OUTPUT}
  )
endfunction()
