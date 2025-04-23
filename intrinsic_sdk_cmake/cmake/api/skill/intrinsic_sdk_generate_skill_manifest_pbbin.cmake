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
# Generate a protobuf bin file (pbbin) from the skill manifest textproto file.
#
# Other commands require the skill manifest in the pbbin format.
#
# :param TARGET: the name for the target that generates the pbbin file
# :type TARGET: string
# :param MANIFEST_TEXTPROTO: the path to the manifest textproto file
# :type MANIFEST_TEXTPROTO: string
# :param MANIFEST_PBBIN_OUTPUT: the output path for the new pbbin manifest file
# :type MANIFEST_PBBIN_OUTPUT: string
#
# @public
#
function(intrinsic_sdk_generate_skill_manifest_pbbin)
  set(options)
  set(one_value_args TARGET MANIFEST_TEXTPROTO MANIFEST_PBBIN_OUTPUT)
  set(multi_value_args)

  cmake_parse_arguments(
    arg
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN})

  add_custom_command(
    OUTPUT ${arg_MANIFEST_PBBIN_OUTPUT}
    COMMAND protobuf::protoc
      --encode=intrinsic_proto.skills.SkillManifest
      --descriptor_set_in=${intrinsic_sdk_cmake_DESCRIPTOR_SET_FILE}
      < ${CMAKE_CURRENT_SOURCE_DIR}/${arg_MANIFEST_TEXTPROTO}
      > ${arg_MANIFEST_PBBIN_OUTPUT}
    DEPENDS ${arg_MANIFEST_TEXTPROTO} ${intrinsic_sdk_cmake_DESCRIPTOR_SET_FILE}
    COMMENT "Generating skill manifest pbbin file: ${arg_MANIFEST_PBBIN_OUTPUT}"
    VERBATIM
  )

  add_custom_target(${arg_TARGET}
    DEPENDS
      ${arg_MANIFEST_PBBIN_OUTPUT}
  )

  if(TARGET intrinsic_proto_desc)
    add_dependencies(${arg_TARGET} intrinsic_proto_desc)
  endif()
endfunction()
