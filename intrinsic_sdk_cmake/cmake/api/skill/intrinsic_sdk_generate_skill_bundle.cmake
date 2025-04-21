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
# Generate a skill bundle that can be deployed to Flowstate.
#
# :param MANIFEST: the path to the manifest file
# :type MANIFEST: string
# :param PROTOS_TARGET: the cmake target for generating the skill's proto files
# :type PROTOS_TARGET: cmake target
# :param SOURCES: the list of source files for the skill target
# :type SOURCES: list of strings
#
# @public
#
function(intrinsic_sdk_generate_skill_bundle)
set(options)
set(one_value_args
  TARGET
  MANIFEST
  PROTO_DESCRIPTOR_FILE
  OCI_IMAGE
  SKILL_BUNDLE_OUTPUT
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
  OUTPUT ${arg_SKILL_BUNDLE_OUTPUT}
  COMMAND inbuild
  ARGS
    skill bundle
    --manifest=${arg_MANIFEST}
    --file_descriptor_set=${arg_PROTO_DESCRIPTOR_FILE}
    --oci_image=${arg_OCI_IMAGE}
    --output=${arg_SKILL_BUNDLE_OUTPUT}
  COMMENT "Generating skill config for ${arg_TARGET}"
  DEPENDS
    ${arg_PROTO_DESCRIPTOR_FILE}
    ${arg_OCI_IMAGE}
)

add_custom_target(${arg_TARGET}
  DEPENDS
    ${arg_SKILL_BUNDLE_OUTPUT}
)
endfunction()
