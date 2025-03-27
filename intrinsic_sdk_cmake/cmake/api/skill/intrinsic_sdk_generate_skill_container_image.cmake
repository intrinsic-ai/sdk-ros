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

if(NOT DEFINED intrinsic_sdk_cmake_API_DIR)
  message(FATAL_ERROR "intrinsic_sdk_cmake_API_DIR not defined, include via cmake/api/all.cmake")
endif()

#
# Generate the skill container image.
#
# This container image will build the user's skill in a container and produce
# an archive (.tar) containing the container image.
#
# The process for building the container by default is to:
#
# - start with the ghcr.io/intrinsic-ai/intrinsic_sdk_cmake:latest image
# - copy the contents of the CONTAINER_CONTEXT_DIRECTORY into the image
# - install the dependencies of the package with rosdep install
# - build the user's package with colcon and --packages-up-to $SKILL_PACKAGE
# - restart with ghcr.io/intrinsic-ai/intrinsic_sdk_cmake:latest
# - install exec (run-time) only dependencies with rosdep install
# - copy the install folder of the user's colcon build
# - set up the container to run the SKILL_EXECUTABLE on start
#
# So the CONTAINER_CONTEXT_DIRECTORY needs to contain everything you need to
# build the SKILL_PACKAGE beyond intrinsic_sdk_cmake and what is installed via
# rosdep install.
#
# Note that the skill main executable must be installed to be used in the
# standard skill dockerfile.
#
# This target is not added to ALL by default, since this is an expensive target
# to build and usually only needs to be built when deploying.
# It is recommended to invoke this target manually with cmake using
# `cmake --build <build dir> --target <TARGET>` or with colcon using the
# `colcon build --cmake-target <TARGET> --packages-select <pkg name>` options.
#
# :param TARGET: the name of the target created by this function
# :type TARGET: string
# :param SKILL_EXECUTABLE: the relative path to the skill executable in the
#   install folder
# :type SKILL_EXECUTABLE: string
# :param SKILL_CONFIG: the relative path to the skill config file in the
#   install folder
# :type SKILL_CONFIG: string
# :param SKILL_NAME: the name of the skill
# :type SKILL_NAME: string
# :param SKILL_PACKAGE: the name of the cmake project/ament pacakge that
#   contains the skill
# :type SKILL_PACKAGE: string
# :param SKILL_ASSET_ID_ORG: the name of the organization in the full skill name
# :type SKILL_ASSET_ID_ORG: string
# :param CONTAINER_CONTEXT_DIRECTORY: the directory to build the container in,
#   which will also be the directory whose contents will be copied into the
#   colcon workspace's src folder inside the container and should contain all
#   the packages that are needed to build the skill package beyond rosdep
#   dependencies.
# :type CONTAINER_CONTEXT_DIRECTORY: string
# :param CONTAINER_IMAGE_OUTPUT: the output path for the container image .tar
# :type CONTAINER_IMAGE_OUTPUT: string
#
# @public
#
function(intrinsic_sdk_generate_skill_container_image)
  set(options)
  set(one_value_args
    TARGET
    SKILL_EXECUTABLE
    SKILL_CONFIG
    SKILL_NAME
    SKILL_PACKAGE
    SKILL_ASSET_ID_ORG
    CONTAINER_CONTEXT_DIRECTORY
    CONTAINER_IMAGE_OUTPUT
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

  # Generate container image using the skill Dockerfile.
  add_custom_target(
    ${arg_TARGET}
    BYPRODUCTS ${arg_CONTAINER_IMAGE_OUTPUT}
    COMMAND podman build
      -f "${intrinsic_sdk_cmake_API_DIR}/skill/resource/skill.Dockerfile"
      --format="docker"
      --output="type=tar,dest=${arg_CONTAINER_IMAGE_OUTPUT}"
      --build-arg SKILL_NAME=${arg_SKILL_NAME}
      --build-arg SKILL_PACKAGE=${arg_SKILL_PACKAGE}
      --build-arg SKILL_EXECUTABLE=${arg_SKILL_EXECUTABLE}
      --build-arg SKILL_CONFIG=${arg_SKILL_CONFIG}
      --build-arg SKILL_ASSET_ID_ORG=${arg_SKILL_ASSET_ID_ORG}
      .
    # COMMAND podman save
    #   --format="docker-archive"
    #   --output="${arg_CONTAINER_IMAGE_OUTPUT}"
    WORKING_DIRECTORY ${arg_CONTAINER_CONTEXT_DIRECTORY}
    COMMENT "Generating skill container image for ${arg_SKILL_NAME}: ${arg_CONTAINER_IMAGE_OUTPUT}"
  )
endfunction()
