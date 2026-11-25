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
import sys
import time
import unittest
import subprocess
import tempfile
import shutil

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import grpc
from concurrent import futures


def start_dummy_grpc_server(port):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    server.add_insecure_port(f'[::]:{port}')
    server.start()
    return server


def _find_workspace_dir(test_dir, aquarium_prefix):
    cur = test_dir
    while cur != os.path.dirname(cur):
        if os.path.basename(cur) == 'src':
            return os.path.dirname(cur)
        cur = os.path.dirname(cur)
    return os.path.dirname(aquarium_prefix)


# Global variable to store temp directory so we can clean it up after test
test_context = {}


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    test_dir = os.path.dirname(os.path.realpath(__file__))
    aquarium_prefix = get_package_prefix('aquarium')
    aquarium_share = get_package_share_directory('aquarium')
    workspace_dir = _find_workspace_dir(test_dir, aquarium_prefix)
    test_tmp_dir = os.path.join(workspace_dir, 'build', 'aquarium', 'tmp')
    os.makedirs(test_tmp_dir, exist_ok=True)

    tmpdir = tempfile.mkdtemp(
        prefix='aquarium_flowstate_test_', dir=test_tmp_dir
    )
    test_context['tmpdir'] = tmpdir

    # Start dummy gRPC server to satisfy Intrinsic SDK connection checks
    dummy_port = 8089
    print(f"TEST_SETUP: Starting dummy gRPC server on port {dummy_port}")
    dummy_server = start_dummy_grpc_server(dummy_port)
    test_context['dummy_server'] = dummy_server

    # 1. Run 'aquarium generate'
    aquarium_bin = os.path.join(
        aquarium_prefix, 'lib', 'aquarium', 'aquarium'
    )
    config_path = os.path.join(
        aquarium_share, 'test', 'test_aquarium_add_two_ints.yaml'
    )

    print(f"TEST_SETUP: Running aquarium generate with config {config_path}")
    gen_cmd = [aquarium_bin, 'generate', 'intrinsic', config_path, tmpdir]
    try:
        subprocess.run(gen_cmd, check=True, cwd=workspace_dir)
    except subprocess.CalledProcessError as e:
        print(f"TEST_SETUP ERROR: Failed to run aquarium generate: {e}")
        shutil.rmtree(tmpdir)
        sys.exit(1)

    # 2. Build the generated skill package
    skill_name = 'add_two_ints_skill'
    skill_src_path = os.path.join(tmpdir, 'skills', skill_name)
    print(f"TEST_SETUP: Building generated skill package at {skill_src_path}")

    build_base = os.path.join(tmpdir, 'build')
    install_base = os.path.join(tmpdir, 'install')

    build_cmd = [
        'colcon', 'build',
        '--paths', skill_src_path,
        '--build-base', build_base,
        '--install-base', install_base,
    ]
    ros_distro = os.environ.get('ROS_DISTRO', 'lyrical')
    build_env = os.environ.copy()
    current_prefix = build_env.get('CMAKE_PREFIX_PATH', '')
    ros_prefix = f'/opt/ros/{ros_distro}'
    if ros_prefix not in current_prefix:
        build_env['CMAKE_PREFIX_PATH'] = (
            f'{ros_prefix}:{current_prefix}' if current_prefix else ros_prefix
        )

    try:
        subprocess.run(build_cmd, check=True, cwd=workspace_dir, env=build_env)
    except subprocess.CalledProcessError as e:
        print(f"TEST_SETUP ERROR: Failed to build generated skill: {e}")
        shutil.rmtree(tmpdir)
        sys.exit(1)

    # Prepare environment variables
    env_vars = os.environ.copy()
    env_vars['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'
    env_vars['RUST_LOG'] = 'info'

    # 3. Boot Zenoh Router
    zenoh_router = ExecuteProcess(
        cmd=[f'/opt/ros/{ros_distro}/lib/rmw_zenoh_cpp/rmw_zenohd'],
        name='rmw_zenohd',
        env=env_vars,
        output='screen'
    )

    # 4. Boot ROS Server (add_two_ints_server)
    ros_server = ExecuteProcess(
        cmd=['ros2', 'run', 'demo_nodes_cpp', 'add_two_ints_server'],
        name='add_two_ints_server',
        env=env_vars,
        output='screen'
    )

    # 5. Boot aquarium_gateway with generated config
    proxy_server_bin = os.path.join(
        aquarium_prefix, 'lib', 'aquarium', 'aquarium_gateway'
    )
    generated_config_path = os.path.join(tmpdir, 'aquarium.yaml')

    proxy_server = ExecuteProcess(
        cmd=[
            proxy_server_bin,
            '--ros-args',
            '-p', f'config_file:={generated_config_path}'
        ],
        name='dynamic_proxy_server',
        env=env_vars,
        output='screen'
    )

    # 6. Boot the generated Skill (gRPC server on port 8003)
    skill_bin = os.path.join(
        install_base, skill_name, 'lib', skill_name, f'{skill_name}_main'
    )
    skill_config = os.path.join(
        install_base,
        skill_name,
        'share',
        skill_name,
        f'{skill_name}_config.pbbin',
    )

    skill_runner = ExecuteProcess(
        cmd=[
            skill_bin,
            '--port', '8003',
            '--skill_service_config_filename', skill_config,
            '--world_service_address', f'localhost:{dummy_port}',
            '--geometry_service_address', f'localhost:{dummy_port}',
            '--motion_planner_service_address', f'localhost:{dummy_port}',
            '--skill_registry_service_address', f'localhost:{dummy_port}'
        ],
        name='skill_runner',
        env=env_vars,
        output='screen'
    )

    # 7. Run the generated C++ client
    client_bin = os.path.join(
        install_base, skill_name, 'lib', skill_name, f'{skill_name}_client'
    )
    skill_client = ExecuteProcess(
        cmd=[client_bin, 'localhost:8003'],
        name='skill_client',
        env=env_vars,
        output='screen'
    )

    # Delay processes to allow dependencies to boot
    delayed_proxy = TimerAction(period=2.0, actions=[proxy_server])
    delayed_skill = TimerAction(period=4.0, actions=[skill_runner])
    delayed_client = TimerAction(period=6.0, actions=[skill_client])

    return LaunchDescription([
        zenoh_router,
        ros_server,
        delayed_proxy,
        delayed_skill,
        delayed_client,
        launch_testing.actions.ReadyToTest()
    ]), {
        'zenoh_router': zenoh_router,
        'ros_server': ros_server,
        'proxy_server': proxy_server,
        'skill_runner': skill_runner,
        'skill_client': skill_client
    }


class TestAddTwoIntsSkillIntegration(unittest.TestCase):

    def test_client_terminates(self, proc_info, skill_client):
        # Await test execution and verify client shuts down within timeout
        proc_info.assertWaitForShutdown(process=skill_client, timeout=20.0)


@launch_testing.post_shutdown_test()
class TestAddTwoIntsSkillIntegrationPostShutdown(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        # Stop dummy gRPC server if running
        dummy_server = test_context.get('dummy_server')
        if dummy_server:
            print("TEST_TEARDOWN: Stopping dummy gRPC server")
            dummy_server.stop(grace=None)

        # Clean up temp directory after all processes shut down
        tmpdir = test_context.get('tmpdir')
        if tmpdir and os.path.exists(tmpdir):
            print(f"TEST_TEARDOWN: Cleaning up {tmpdir}")
            shutil.rmtree(tmpdir)

    def test_client_exit_code(self, proc_info, proc_output, skill_client):
        # Verify that the client exited with 0 and computed 42 + 42 == 84
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process=skill_client
        )
        launch_testing.asserts.assertInStdout(
            proc_output,
            'sum: 84',
            skill_client
        )

