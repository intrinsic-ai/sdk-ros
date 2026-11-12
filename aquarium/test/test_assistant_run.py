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
import signal
import time
import unittest
import subprocess

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
import launch_testing.asserts
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    env_vars = os.environ.copy()
    env_vars['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'
    ros_distro = os.environ.get('ROS_DISTRO', 'lyrical')

    # 1. Boot Zenoh Router
    zenoh_router = ExecuteProcess(
        cmd=[f'/opt/ros/{ros_distro}/lib/rmw_zenoh_cpp/rmw_zenohd'],
        name='rmw_zenohd_test',
        env=env_vars,
        output='screen'
    )

    # 2. Boot aquarium assistant run in background with selection file
    aquarium_share = get_package_share_directory('aquarium')
    selection_file_path = os.path.join(
        aquarium_share, 'test', 'test_selection.yaml'
    )

    aquarium_prefix = get_package_prefix('aquarium')
    aquarium_bin = os.path.join(
        aquarium_prefix, 'lib', 'aquarium', 'aquarium'
    )
    assistant_run = ExecuteProcess(
        cmd=[
            aquarium_bin,
            'assistant',
            'run',
            'demo_nodes_cpp',
            'add_two_ints_server',
            '--selection',
            selection_file_path,
        ],
        name='aquarium_assistant_run',
        env=env_vars,
        output='screen'
    )

    return LaunchDescription([
        zenoh_router,
        assistant_run,
        launch_testing.actions.ReadyToTest()
    ]), {
        'zenoh_router': zenoh_router,
        'assistant_run': assistant_run
    }


class TestAssistantRunActive(unittest.TestCase):

    def test_assistant_launches_node_and_cleans_up(
        self, proc_info, assistant_run
    ):
        # 1. Wait for headless assistant to exit
        proc_info.assertWaitForShutdown(process=assistant_run, timeout=15.0)

        # 2. Assert exit code 0
        launch_testing.asserts.assertExitCodes(proc_info, process=assistant_run)

        # 3. Verify that aquarium.yaml was generated with expected content
        config_path = 'aquarium.yaml'
        self.assertTrue(
            os.path.exists(config_path),
            f'Output config file {config_path} not found.',
        )

        try:
            with open(config_path, 'r') as f:
                config_content = f.read()

            self.assertIn('/add_two_ints', config_content)
            self.assertIn('type_hash', config_content)
            self.assertIn('request_fields', config_content)
            self.assertIn('response_fields', config_content)
        finally:
            # Clean up generated aquarium.yaml from the working directory
            if os.path.exists(config_path):
                os.remove(config_path)

        # 4. Verify that the service is now gone (cleaned up)
        env_vars = os.environ.copy()
        env_vars['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'

        service_cleaned_up = False
        services_output_post = ""
        for _ in range(30):  # Poll for up to 15 seconds
            time.sleep(0.5)
            try:
                services_output_post = subprocess.check_output(
                    ['ros2', 'service', 'list'],
                    env=env_vars,
                    text=True,
                    timeout=5.0
                )
            except subprocess.SubprocessError:
                continue

            if '/add_two_ints' not in services_output_post:
                service_cleaned_up = True
                break

        self.assertTrue(
            service_cleaned_up,
            'Service /add_two_ints was not cleaned up:'
            f' {services_output_post}',
        )
