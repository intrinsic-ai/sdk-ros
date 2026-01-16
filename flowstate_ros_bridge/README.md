# Flowstate ROS Bridge

## Building the flowstate_ros_bridge bundle

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

## Documentation

* [Flowstate Diagnostics ROS Bridge:](docs/diagnostics_ros_bridge.md) Details the operational logic of the diagnostics polling feature, including architecture, configuration, and verification procedures.