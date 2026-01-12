# Flowstate ROS Bridge

## Running in a local environment

### Building the flowstate_ros_bridge locally

Before we can build the `flowstate_ros_bridge`, you must follow the [Getting Started](https://github.com/intrinsic-ai/sdk-ros/blob/main/README.md#getting-started) instructions to set up your local SDK-ROS environment. 

Source ROS and build the `flowstate_ros_bridge` within your workspace:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/intrinsic_ws/
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers=console_direct+ \
  --packages-up-to flowstate_ros_bridge
```

### Running the flowstate_ros_bridge locally



## Sideloading to Flowstate as a service

### Building the flowstate_ros_bridge bundle (Docker)

Similar to how a Flowstate service that uses ROS is built, a couple of scripts are provided to build and bundle the flowstate_ros_bridge. The following steps will assume the `sdk-ros` repository is in the `src` folder of your workspace.

In the root of a colcon workspace first set up the docker engine:

```bash
./src/sdk-ros/scripts/setup_docker.sh
```

Then, create the bundle with the `build_service_bundle.sh` script. This will compile the packages in a docker container and bundle that container in a tarball.

```bash
./src/sdk-ros/flowstate_ros_bridge/scripts/build_service_bundle.sh
```

The output of this command will be a tarball inside the `images` directory of the colcon workspace which can be pushed to Flowstate as a new service.

### Sideloading the flowstate_ros_bridge to a running Flowstate solution

With a solution open in Flowstate, the generated service bundle can be sideloaded with `inctl`.

```bash
./inctl service install images/flowstate_ros_bridge.bundle.tar --org ekumen --cluster vmp-8c31-qpfxh4gp
```
