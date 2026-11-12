# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Preserve environmental variables mapping rmw_zenoh_cpp for executing nodes
    env_vars = os.environ.copy()
    env_vars['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'
    ros_distro = os.environ.get('ROS_DISTRO', 'lyrical')

    # 1. Boot the standard Zenoh Router Daemon
    zenoh_router = ExecuteProcess(
        cmd=[f'/opt/ros/{ros_distro}/lib/rmw_zenoh_cpp/rmw_zenohd'],
        name='rmw_zenohd_test',
        env=env_vars,
        output='screen'
    )

    # 2. Boot the standard service provider Node A: /add_two_ints_server
    ros_server_a = ExecuteProcess(
        cmd=[f'/opt/ros/{ros_distro}/lib/demo_nodes_cpp/add_two_ints_server'],
        name='add_two_ints_server_test',
        env=env_vars,
        output='screen'
    )

    # 3. Boot the standard lifecycle provider Node B: /lc_talker
    ros_server_b = ExecuteProcess(
        cmd=[f'/opt/ros/{ros_distro}/lib/lifecycle/lifecycle_talker'],
        name='lifecycle_talker_test',
        env=env_vars,
        output='screen'
    )

    # 4. Boot our compiled C++ GTest dynamic test runner client executable
    aquarium_prefix = get_package_prefix('aquarium')
    gtest_client_bin = os.path.join(
        aquarium_prefix, 'lib', 'aquarium', 'test_live_introspection'
    )

    gtest_client = ExecuteProcess(
        cmd=[gtest_client_bin],
        name='test_live_introspection_client',
        env=env_vars,
        output='screen'
    )

    # Register all executing nodes under standard dynamic launcher descriptions
    return LaunchDescription([
        zenoh_router,
        ros_server_a,
        ros_server_b,
        gtest_client,
        launch_testing.actions.ReadyToTest()
    ]), {
        'zenoh_router': zenoh_router,
        'ros_server_a': ros_server_a,
        'ros_server_b': ros_server_b,
        'gtest_client': gtest_client
    }


# Active run-time testing assertions prior to termination
class TestLiveIntrospectionActive(unittest.TestCase):

    def test_gtest_client_terminates(self, proc_info, gtest_client):
        # Await dynamic verification results (15s timeout for Zenoh discovery)
        proc_info.assertWaitForShutdown(process=gtest_client, timeout=15.0)


# Post-shutdown test validation blocks checking GTest client exits
@launch_testing.post_shutdown_test()
class TestLiveIntrospectionPostShutdown(unittest.TestCase):

    def test_gtest_client_exit_code(self, proc_info, gtest_client):
        # Assert exit code 0 (all dynamic GTest catalog assertions passed)
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest_client)
