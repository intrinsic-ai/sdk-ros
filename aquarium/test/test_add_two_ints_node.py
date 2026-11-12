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
import pty
import signal
import time
import unittest
import subprocess
import yaml

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
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

    return LaunchDescription([
        zenoh_router,
        launch_testing.actions.ReadyToTest()
    ]), {
        'zenoh_router': zenoh_router
    }


class TestAddTwoIntsNode(unittest.TestCase):

    def test_tui_generation(self):
        # Run the assistant in a temporary directory and generate aquarium.yaml.
        import tempfile
        import shutil

        tmpdir = tempfile.mkdtemp(prefix='aquarium_test_')
        self.addCleanup(shutil.rmtree, tmpdir)

        env_vars = os.environ.copy()
        env_vars['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'

        # Locate the installed aquarium executable.
        aquarium_prefix = get_package_prefix('aquarium')
        aquarium_bin = os.path.join(
            aquarium_prefix, 'lib', 'aquarium', 'aquarium'
        )

        # Spawn the assistant run command using PTY to interact with TUI
        master, slave = pty.openpty()

        cmd = [
            aquarium_bin,
            'assistant',
            'run',
            'demo_nodes_cpp',
            'add_two_ints_server',
        ]

        proc = subprocess.Popen(
            cmd,
            stdin=slave,
            stdout=slave,
            stderr=slave,
            env=env_vars,
            cwd=tmpdir,
            text=True,
            close_fds=True
        )
        os.close(slave)

        # Wait for TUI to boot and discover services.
        # Zenoh discovery might take some time.
        time.sleep(8.0)

        # Switch to Service Mode
        os.write(master, b's')
        time.sleep(1.0)

        # Vim-style navigation test: Move down using 'j', then back up using 'k'
        os.write(master, b'j')
        time.sleep(0.5)
        os.write(master, b'k')
        time.sleep(0.5)

        # We expect "/add_two_ints" to be the only/first service.
        # Press Space to select it.
        os.write(master, b' ')
        time.sleep(1.0)

        # Press 'w' to write config.
        os.write(master, b'w')
        time.sleep(1.0)

        # The TUI should have written aquarium.yaml and exited.
        # Wait for process to exit.
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            # If it didn't exit, force kill it
            proc.terminate()
            proc.wait()
            self.fail("TUI did not exit after generating config.")

        os.close(master)

        # Verify aquarium.yaml was generated
        yaml_path = os.path.join(tmpdir, 'aquarium.yaml')
        self.assertTrue(
            os.path.exists(yaml_path),
            f'aquarium.yaml was not generated in {tmpdir}',
        )

        # Verify content of aquarium.yaml
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        self.assertIsNotNone(config)
        self.assertIn('target', config)
        self.assertEqual(config['target']['type'], 'node')
        self.assertEqual(config['target']['package'], 'demo_nodes_cpp')
        self.assertEqual(config['target']['name'], 'add_two_ints_server')

        self.assertIn('interfaces', config)
        interfaces = config['interfaces']
        self.assertEqual(len(interfaces), 1)

        iface = interfaces[0]
        self.assertEqual(iface['internal_interface'], 'ros_service')
        self.assertEqual(iface['internal_name'], '/add_two_ints')
        self.assertEqual(
            iface['internal_type'], 'example_interfaces/srv/AddTwoInts'
        )
        self.assertEqual(iface['external_interface'], 'zenoh_queryable_proto')
        self.assertEqual(iface['external_name'], 'add_two_ints')
        self.assertEqual(
            iface['external_type'], 'example_interfaces.srv.AddTwoInts'
        )
