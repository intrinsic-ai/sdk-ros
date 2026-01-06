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

To use the SDK in Python, you must additionally create a virtualenv and install a few dependencies which are not provided by the SDK, nor are the ones available in Ubuntu's apt new enough.

For example, you could:

```bash
# Setup the venv and activate it
python3 -m venv --system-site-packages venv
source ./venv/bin/activate
# Install the new dependencies
# (venv)
pip install -U grpcio protobuf retrying
# Test that it is working
# (venv)
python3 -c 'from intrinsic.world.python.object_world_client import ObjectWorldClient'
```

## Building the flowstate_ros_bridge and uploading it to FlowState

Similar to how a Flowstate service that uses ROS is built, a couple of scripts are provided to build and bundle the flowstate_ros_bridge. The following steps will assume the `sdk-ros` repository is in the `src` folder of your workspace.

In the root of a colcon workspace first set up the docker engine:

```bash
./src/sdk-ros/scripts/setup_docker.sh
```

Then, create the bundle with the `build_service_bundle.sh` script. This will compile the packages in a docker container and bundle that container in a tarball.

```bash
./src/sdk-ros/flowstate_ros_bridge/scripts/build_service_bundle.sh
```

The output of this command will be a tarball inside the `images` directory of the colcon workspace which can be pushed to FlowState as a new service.
