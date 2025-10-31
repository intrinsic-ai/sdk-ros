# Intrinsic SDK for ROS

The Intrinsic SDK for ROS is a wrapper of the [Intrinsic
SDK](https://github.com/intrinsic-dev/sdk) that allows for software developers
to write ROS-based workflows that work with
[Flowstate](https://intrinsic.ai/flowstate), a web-based tool to build robot
solutions from concept to deployment.

The Intrinsic SDK for ROS is compatible with [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html).

In addition to the this [Intrinsic SDK for
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
source /opt/ros/jazzy.setup.bash
cd ~/intrinsic_ws/
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers=console_direct
```

## Disclaimer

As Flowstate and the SDK are in beta, the contents of this repository are
subject to change.
Use of this repository requires participation in the beta for Intrinsic
Flowstate, which is accepting [applications](https://intrinsic.ai/beta).
Access to this repository is subject to the associated [LICENSE](LICENSE).
