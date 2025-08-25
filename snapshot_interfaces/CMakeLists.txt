###############################################################################
# Copyright 2025 Intrinsic Innovation LLC
#
# You are hereby granted a non-exclusive, worldwide, royalty-free license to
# use, copy, modify, and distribute this Intrinsic SDK in source code or binary
# form for use in connection with the services and APIs provided by Intrinsic
# Innovation LLC (“Intrinsic”).
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# If you use this Intrinsic SDK with any Intrinsic services, your use is
# subject to the Intrinsic Platform Terms of Service
# [https://intrinsic.ai/platform-terms].  If you create works that call
# Intrinsic APIs, you must agree to the terms of service for those APIs
# separately. This license does not grant you any special rights to use the
# services.
#
# This copyright notice shall be included in all copies or substantial portions
# of the software.
###############################################################################

cmake_minimum_required(VERSION 3.8)
project(snapshot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DiscoveredCamera.msg"
  "msg/SensorInfo.msg"
  "msg/ImageSnapshot.msg"
  "msg/PointCloud2Snapshot.msg"
  "msg/TemperatureSnapshot.msg"
  "msg/ImuSnapshot.msg"
  "srv/Discover.srv"
  "srv/Describe.srv"
  "srv/Snapshot.srv"
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  sensor_msgs
)

ament_package()
