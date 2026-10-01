# Flowstate ROS GZ Bridge

This node helps bridge Flowstate Gazebo simulation to ROS. It needs to be built and sideloaded to the solution to interface with Gazebo.

### Building and sideloading the flowstate_ros_gz_bridge

Building and sideloading the bridge is similar to how other services are installed, from the root of your colcon workspace. Follow the instructions [to build the Intrinsic SDK Bundle Library](https://github.com/intrinsic-ai/sdk-ros/tree/main/intrinsic_sdk_bundle_library_py), then: 

```bash
colcon intrinsic_bundle --packages-select flowstate_ros_gz_bridge --ros-distro $ROS_DISTRO --default-config src/sdk-ros/flowstate_ros_gz_bridge/flowstate_ros_gz_bridge_service_default_config.pbtxt

# Replace with your organization and cluster
inctl asset install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT intrinsic_asset_bundles/flowstate_ros_gz_bridge_service/flowstate_ros_gz_bridge_service.bundle.tar
```
### Instantiate the service

From the flowstate UI, add the service to your solution. Its default config bridges the `/clock` topic for simulated clock.
To test locally with inctl port forwarding over zenoh:

```bash
# SSH and port forward
inctl ssh --project $INTRINSIC_ORGANIZATION --context $INTRINSIC_CONTEXT --no-tofu -- -N -L 17080:localhost:17080 -L 17447:localhost:17447

# Make sure all the ROS shells are running with RMW_IMPLEMENTATION=rmw_zenoh_cpp
# In another terminal, run the zenoh router
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:17447"]' ros2 run rmw_zenoh_cpp rmw_zenohd

# In another terminal
ros2 topic echo /clock --no-daemon
```

You should see the clock being published.
To add bridges update the configuration proto. The fields follow `ros_gz_bridge` YAML configuration as shown in its [README](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge#example-5-configuring-the-bridge-via-yaml).
