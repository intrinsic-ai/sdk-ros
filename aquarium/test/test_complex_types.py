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

import glob
import os
import time
import unittest
from google.protobuf import descriptor_pb2
from google.protobuf import descriptor_pool
from google.protobuf import message_factory

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

# Start the Zenoh router, the dummy ROS 2 server, and aquarium_gateway,
# then make a gRPC call to /test_capture and verify the response.


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    aquarium_prefix = get_package_prefix('aquarium')
    aquarium_share = get_package_share_directory('aquarium')
    ros_distro = os.environ.get('ROS_DISTRO', 'lyrical')

    env_vars = os.environ.copy()
    env_vars['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'
    env_vars['RCUTILS_LOG_SEVERITY_THRESHOLD'] = 'DEBUG'

    # Ensure installed aquarium Python site-packages (containing generated
    # aquarium.msg / aquarium.srv) precedes any editable venv install.
    py_dirs = (
        glob.glob(
            os.path.join(aquarium_prefix, '**', 'site-packages'),
            recursive=True,
        )
        + glob.glob(
            os.path.join(aquarium_prefix, '**', 'dist-packages'),
            recursive=True,
        )
    )
    if py_dirs:
        existing_py = env_vars.get('PYTHONPATH', '')
        prefix_py = os.pathsep.join(py_dirs)
        env_vars['PYTHONPATH'] = (
            f'{prefix_py}{os.pathsep}{existing_py}'
            if existing_py
            else prefix_py
        )

    # 1. Boot Zenoh Router
    zenoh_router = ExecuteProcess(
        cmd=[f'/opt/ros/{ros_distro}/lib/rmw_zenoh_cpp/rmw_zenohd'],
        name='rmw_zenohd_complex',
        env=env_vars,
        output='screen'
    )

    # 2. Boot dummy ROS 2 server
    dummy_server_path = os.path.join(
        aquarium_share, 'test', 'dummy_camera_server.py'
    )
    ros_server = ExecuteProcess(
        cmd=['python3', '-u', dummy_server_path],
        name='dummy_camera_server',
        env=env_vars,
        output='screen'
    )

    # 3. Boot aquarium_gateway with test_complex_types.yaml
    gateway_bin = os.path.join(
        aquarium_prefix, 'lib', 'aquarium', 'aquarium_gateway'
    )
    config_path = os.path.join(
        aquarium_share, 'test', 'test_complex_types.yaml'
    )

    gateway = ExecuteProcess(
        cmd=[
            gateway_bin,
            '--ros-args',
            '-p', f'config_file:={config_path}'
        ],
        name='aquarium_gateway_complex',
        env=env_vars,
        output='screen'
    )

    # Delay gateway to allow Zenoh and ROS server to boot
    delayed_gateway = TimerAction(period=2.0, actions=[gateway])

    return LaunchDescription([
        zenoh_router,
        ros_server,
        delayed_gateway,
        launch_testing.actions.ReadyToTest()
    ]), {
        'zenoh_router': zenoh_router,
        'ros_server': ros_server,
        'gateway': gateway
    }


class TestComplexTypes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Build FileDescriptorProto manually in Python without grpc_tools
        file_proto = descriptor_pb2.FileDescriptorProto()
        file_proto.name = "test_capture.proto"
        file_proto.package = "ai.intrinsic"
        file_proto.syntax = "proto3"

        # 1. TestCameraConfig
        msg = file_proto.message_type.add()
        msg.name = "aquarium_msg_TestCameraConfig"
        f = msg.field.add()
        f.name = "trigger_mode"
        f.number = 1
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_STRING
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        f = msg.field.add()
        f.name = "exposure_time"
        f.number = 2
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_DOUBLE
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        # 2. TestStorageLocation
        msg = file_proto.message_type.add()
        msg.name = "aquarium_msg_TestStorageLocation"
        f = msg.field.add()
        f.name = "store"
        f.number = 1
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_STRING
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
        f = msg.field.add()
        f.name = "key"
        f.number = 2
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_STRING
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        # 3. TestSensorImage
        msg = file_proto.message_type.add()
        msg.name = "aquarium_msg_TestSensorImage"
        f = msg.field.add()
        f.name = "sensor_id"
        f.number = 1
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_INT64
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
        f = msg.field.add()
        f.name = "frame_id"
        f.number = 2
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_STRING
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
        f = msg.field.add()
        f.name = "data"
        f.number = 3
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_BYTES
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        # 4. TestCaptureResult (nested)
        msg = file_proto.message_type.add()
        msg.name = "aquarium_msg_TestCaptureResult"
        f = msg.field.add()
        f.name = "images"
        f.number = 1
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE
        f.type_name = ".ai.intrinsic.aquarium_msg_TestSensorImage"
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED

        # 5. TestCaptureParams (Request)
        msg = file_proto.message_type.add()
        msg.name = "TestCaptureParams"
        f = msg.field.add()
        f.name = "camera_config"
        f.number = 1
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE
        f.type_name = ".ai.intrinsic.aquarium_msg_TestCameraConfig"
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        f = msg.field.add()
        f.name = "sensor_ids"
        f.number = 2
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_INT64
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED

        f = msg.field.add()
        f.name = "capture_result_location"
        f.number = 3
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE
        f.type_name = ".ai.intrinsic.aquarium_msg_TestStorageLocation"
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED

        # 6. TestCaptureResult (Response)
        msg = file_proto.message_type.add()
        msg.name = "TestCaptureResult"
        f = msg.field.add()
        f.name = "aquarium_skill_status"
        f.number = 1
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_STRING
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        f = msg.field.add()
        f.name = "capture_result"
        f.number = 2
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE
        f.type_name = ".ai.intrinsic.aquarium_msg_TestCaptureResult"
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        f = msg.field.add()
        f.name = "capture_result_location"
        f.number = 3
        f.type = descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE
        f.type_name = ".ai.intrinsic.aquarium_msg_TestStorageLocation"
        f.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        pool = descriptor_pool.Default()
        pool.Add(file_proto)
        factory = message_factory.MessageFactory(pool)

        cls.req_class = factory.GetPrototype(
            pool.FindMessageTypeByName("ai.intrinsic.TestCaptureParams")
        )
        cls.resp_class = factory.GetPrototype(
            pool.FindMessageTypeByName("ai.intrinsic.TestCaptureResult")
        )

    def test_complex_types_service_call(self):
        # Create gRPC channel
        channel = grpc.insecure_channel('localhost:50051')

        # Build request
        req = self.req_class()
        req.camera_config.trigger_mode = "EXTERNAL"
        req.camera_config.exposure_time = 0.042
        req.sensor_ids.extend([10, 20, 30])

        loc = req.capture_result_location.add()
        loc.store = "my_store"
        loc.key = "my_key"

        # Wait for gateway to be ready
        time.sleep(3.0)

        # Make call using raw bytes
        print("Sending gRPC request...")
        request_bytes = req.SerializeToString()
        try:
            response_bytes = channel.unary_unary(
                '/ai.intrinsic.TestCapture_Service/Call',
                request_serializer=None,
                response_deserializer=None,
            )(request_bytes, timeout=10.0)

            resp = self.resp_class()
            resp.ParseFromString(response_bytes)
            print("Received and parsed gRPC response:")
            print(resp)
        except grpc.RpcError as e:
            self.fail(f"gRPC call failed: {e}")

        # Verify response
        self.assertEqual(resp.aquarium_skill_status, "SUCCEEDED")
        self.assertTrue(resp.HasField('capture_result'))
        self.assertEqual(len(resp.capture_result.images), 3)

        for i, sensor_id in enumerate([10, 20, 30]):
            img = resp.capture_result.images[i]
            self.assertEqual(img.sensor_id, sensor_id)
            self.assertEqual(img.frame_id, f"camera_frame_{sensor_id}")
            self.assertEqual(img.data, b'\x00\x01\x02\x03\x04')

        self.assertTrue(resp.HasField('capture_result_location'))
        self.assertEqual(resp.capture_result_location.store, "my_store")
        self.assertEqual(resp.capture_result_location.key, "my_key")
