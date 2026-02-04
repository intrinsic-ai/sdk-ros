# Flowstate Robot State and Sensor Publishing

## 1. Overview

This bridge facilitates the translation of internal Flowstate data—specifically joint states, gripper and force/torque readings into standard ROS 2 messages. It provides a configurable interface to expose robot hardware states and sensor feedback to the ROS ecosystem.


**How it works:**

* **Configurable Filtering:** The bridge utilizes a `SensorConfig` Protobuf message to manage data streams. Users can toggle specific sensor topics (robot state, gripper or force/torque) on or off via Flowstate `ROS bridge` service configuration, which prevents the instantiation of unnecessary ROS publishers and reduces network overhead.
* **Zenoh-Based Subscriptions:** The bridge uses the `intrinsic::platform::pubsub` library to establish direct Zenoh subscriptions to Flowstate's internal telemetry topics, such as `/icon/robot_controller/robot_status_throttle`.
* **Callback-Driven Translation:** The bridge registers dedicated callback functions (e.g., `RobotStateCallback`) that are triggered whenever new data arrives on the subscribed topics. These handlers parse the high-frequency Protobuf payloads and map them into ROS-native messages `JointState` and `WrenchStamped`.

The implementation is integrated into the following components:

* [`include/flowstate_ros_bridge/world.hpp`](../include/flowstate_ros_bridge/world.hpp) & [`src/world.cpp`](../src/world.cpp): Logic for creating specialized Flowstate subscriptions.
* [`src/bridges/world_bridge.cpp`](../src/bridges/world_bridge.cpp) & [`.hpp`](../src/bridges/world_bridge.hpp): Management of ROS publishers and the core data translation logic.

---

## 2. Interface Updates

### 2.1 ROS Topic Interface

The following topics are now available for publishing sensor data:

| Topic | Type | Description |
| :--- | :--- | :--- |
| `robot_state` | `sensor_msgs/msg/JointState` | Robot joint positions, velocities, and effort. |
| `gripper_states` | `sensor_msgs/msg/JointState` | Current sensed state or width of the gripper. |
| `force_torque_sensors` | `geometry_msgs/msg/WrenchStamped` | Force and torque (wrench) data in the part's frame. |

### 2.2 Logic Mapping Details (Proto → ROS)

The bridge implements a direct field-level mapping to ensure that high-frequency data from Flowstate is accurately represented in standard ROS messages.

#### **Joint State Mapping**
The `sensor_msgs/msg/JointState` message is populated by iterating through each joint defined in the Flowstate `PartStatus`.

| ROS `JointState` Field | Flowstate Proto Source (`PartStatus`) | Mapping Logic / Default |
| :--- | :--- | :--- |
| `header.stamp` | Provided `rclcpp::Time` | Synchronized to the callback trigger time. |
| `header.frame_id` | N/A | Typically left empty or defined by global prefix. |
| `name` | `part_name` + `_joint_` + `index` | Generated as `absl::StrFormat("%s_joint_%d", part_name, i)`. |
| `position` | `joint_state.position_sensed()` | Defaults to `0.0` if position is not sensed. |
| `velocity` | `joint_state.velocity_sensed()` | Defaults to `0.0` if velocity is not sensed. |
| `effort` | `joint_state.torque_sensed()` | Defaults to `0.0` if torque is not sensed . |

#### **Wrench (Force/Torque) Mapping**
The `geometry_msgs/msg/WrenchStamped` message maps 6-axis force and torque data directly from the `wrench_at_ft` field.

| ROS `WrenchStamped` Field | Flowstate Proto Source (`wrench_at_ft`) | Description |
| :--- | :--- | :--- |
| `header.stamp` | Provided `rclcpp::Time` | Reference time of the sensor reading. |
| `header.frame_id` | `part_name` | The link or part where the sensor is mounted. |
| `wrench.force.x` | `w.x()` | Sensed force along the X-axis. |
| `wrench.force.y` | `w.y()` | Sensed force along the Y-axis. |
| `wrench.force.z` | `w.z()` | Sensed force along the Z-axis. |
| `wrench.torque.x` | `w.rx()` | Sensed torque about the X-axis. |
| `wrench.torque.y` | `w.ry()` | Sensed torque about the Y-axis. |
| `wrench.torque.z` | `w.rz()` | Sensed torque about the Z-axis. |

### 2.3 Configuration Parameters

These parameters are defined in the `SensorConfig` message and can be modified in Flowstate `ROS Bridge` service or in `.pbtxt` configuration:

| Parameter Name | Default | Description |
| :--- | :--- | :--- |
| `enable_robot_state_topic` | `true` | Enables publishing of robot joint states. |
| `enable_gripper_state_topic` | `true` | Enables publishing of gripper status. |
| `enable_force_torque_topic` | `true` | Enables publishing of F/T sensor data. |

---

## 3. Usage & Verification

### 3.1 Configuration Check

Verify your settings in Flowstate `ROS bridge` or in `flowstate_ros_bridge_default_config.pbtxt`. The bridge logs the status of each sensor stream during the `initialize` phase:

> `"Robot State Bridge Enabled: true"`

### 3.2 Deployment & Testing Steps

1. **Verify Connection:** Monitor the bridge logs for successful Flowstate subscriptions:

   > `"Subscribed to Flowstate Robot State topic"`

2. **Performance Monitoring:** The bridge reports translation latency every 1000 messages to ensure real-time performance is maintained:
   ```text
   Robot state translation time: 0.045 ms
    ```
3. **Verify ROS Output:** Use standard ROS 2 tools to inspect the sensor data:

    ```bash
    ros2 topic echo /robot_state
    ros2 topic echo /force_torque_sensors
    ```