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
# :param SKILL_NAME: the name of the skill
# :type SKILL_NAME: string
# :param MANIFEST: the path to the manifest file
# :type MANIFEST: string
#
# @public
#
function(intrinsic_sdk_generate_skill_config)
  set(options)
  set(one_value_args SKILL_NAME MANIFEST)
  set(multi_value_args)

  cmake_parse_arguments(
    GENERATE_ARGS
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  set(OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

  # Generate binary version of the manifest textproto file using protoc.
  add_custom_command(
    OUTPUT ${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_manifest.pbbin
    COMMAND protobuf::protoc
      --encode=intrinsic_proto.skills.SkillManifest
      --descriptor_set_in=${intrinsic_sdk_cmake_DESCRIPTOR_SET_FILE}
      < ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_ARGS_MANIFEST}
      > ${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_manifest.pbbin
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_ARGS_MANIFEST}
    COMMENT "Generating skill manifest for ${GENERATE_ARGS_SKILL_NAME}"
  )

  # Generate the binary proto skill config
  # add_custom_command(
  #   OUTPUT ${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_skill_config.pbbin
  #   COMMAND ${intrinsic_sdk_DIR}/../../../bin/skill_service_config_main
  #   ARGS
  #     --manifest_pbbin_filename=${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_manifest.pbbin
  #     --proto_descriptor_filename=${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_protos.desc
  #     --output_config_filename=${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_skill_config.pbbin
  #   COMMENT "Generating skill config for ${GENERATE_ARGS_SKILL_NAME}"
  #   DEPENDS
  #     ${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_manifest.pbbin
  #     ${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_protos.desc
  # )

  add_custom_target(${GENERATE_ARGS_SKILL_NAME}_skill_config
    DEPENDS
      ${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_manifest.pbbin
      # ${OUT_DIR}/${GENERATE_ARGS_SKILL_NAME}_skill_config.pbbin
  )
endfunction()
