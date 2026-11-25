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

from aquarium.msg import TestCaptureResult, TestSensorImage
from aquarium.srv import TestCapture
import rclpy
from rclpy.node import Node

class DummyCameraServer(Node):

    def __init__(self):
        super().__init__('dummy_camera_server')
        self.srv = self.create_service(
            TestCapture, '/test_capture', self.capture_callback
        )
        self.get_logger().info("Dummy Camera Server ready on /test_capture")

    def capture_callback(self, request, response):
        self.get_logger().info("Received Capture Request:")
        self.get_logger().info(
            f"  trigger_mode: {request.camera_config.trigger_mode}"
        )
        self.get_logger().info(
            f"  exposure_time: {request.camera_config.exposure_time}"
        )
        self.get_logger().info(f"  sensor_ids: {request.sensor_ids}")
        loc_len = len(request.capture_result_location)
        self.get_logger().info(f"  capture_result_location size: {loc_len}")

        # Build response
        response.capture_result = TestCaptureResult()
        for s_id in request.sensor_ids:
            img = TestSensorImage()
            img.sensor_id = s_id
            img.frame_id = f"camera_frame_{s_id}"
            img.data = bytes([0, 1, 2, 3, 4])
            response.capture_result.images.append(img)

        if len(request.capture_result_location) > 0:
            loc = request.capture_result_location[0]
            self.get_logger().info(
                f"  capture_result_location: {loc.store} / {loc.key}"
            )
            response.capture_result_location.store = loc.store
            response.capture_result_location.key = loc.key
        else:
            response.capture_result_location.store = ""
            response.capture_result_location.key = ""

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DummyCameraServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
