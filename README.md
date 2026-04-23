# Intrinsic SDK for ROS

The Intrinsic SDK for ROS is a wrapper of the [Intrinsic
SDK](https://github.com/intrinsic-dev/sdk) that allows for software developers
to write ROS-based workflows that work with
[Flowstate](https://intrinsic.ai/flowstate), a web-based tool to build robot
solutions from concept to deployment.

The Intrinsic SDK for ROS is compatible with [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html).

In addition to this [Intrinsic SDK for
ROS repository](https://github.com/intrinsic-dev/intrinsic_sdk_ros), there are
also:
 * [ROS-based SDK Examples](https://github.com/intrinsic-dev/sdk_examples_ros)
 * [The Intrinsic SDK](https://github.com/intrinsic-dev/sdk)
 * [Intrinsic SDK examples](https://github.com/intrinsic-dev/sdk-examples)
 * [Dev container project template](https://github.com/intrinsic-dev/project-template)

## Getting Started

Clone this repository into your ROS workspace.

```bash
cd ~/intrinsic_ws/src # Replace with source directory to your workspace.
git clone https://github.com/intrinsic-ai/sdk-ros.git
```

Source ROS and build the SDK.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/intrinsic_ws/
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers=console_direct+
```

### Using the SDK in Python

A recent version of gRPC is vendored with the SDK. From within a solution, you can connect to cloud gRPC services.

For example:

```python
from intrinsic.frontend.solution_service.proto.solution_service_pb2 import (
    GetStatusRequest,
)
from intrinsic.frontend.solution_service.proto.solution_service_pb2_grpc import (
    SolutionServiceStub,
)
from intrinsic.frontend.solution_service.proto.status_pb2 import Status
from intrinsic.util.grpc.grpc_utils import create_cloud_channel


def main():
    with create_cloud_channel() as channel:
        stub = SolutionServiceStub(channel)
        response = stub.GetStatus(GetStatusRequest())
```

Connecting to gRPC services from outside the solution (i.e. from your laptop) is not supported yet.

<!-- TODO: implement authentication for out of cluster connections -->

## Building and packaging the flowstate_ros_bridge

See [Building the flowstate_ros_bridge bundle](flowstate_ros_bridge/README.md) for more details on how to build and package the bridge.
