# `aquarium.yaml` Configuration Reference

`aquarium.yaml` is the declarative configuration file consumed by
`aquarium build`, `aquarium generate intrinsic`, and `aquarium_gateway`. It
specifies the ROS 2 target to run, any nested message definitions, and the ROS
2 interfaces to bridge.

## Schema Reference

```yaml
# ------------------------------------------------------------------------------
# 1. TARGET SPECIFICATION ('target' or 'targets')
# Use 'target' when simulation and real hardware run the same package/target
# (builds a single service image shared by sim_spec and real_spec).
# Use 'targets' with 'sim' and 'real' sub-keys when they differ.
# ------------------------------------------------------------------------------
target:
  # Required: "launch" (runs ros2 launch) or "node" (runs ros2 run)
  type: "launch"

  # Required: ROS 2 package containing the launch file or node executable
  package: "aquarium"

  # Required: Launch file name (for "launch") or executable name (for "node")
  name: "robotiq.launch.xml"

  # Optional: Extra ROS 2 packages to include in the built container closure
  # (useful when a launch file includes nodes from other packages)
  dependencies:
    - "robotiq_driver"
    - "robotiq_description"

  # Optional: Arguments passed to ros2 launch ("arg:=val") or ros2 run
  arguments:
    - "use_fake_hardware:=true"

  # Optional: rosdep keys to skip during container dependency installation
  rosdep:
    skip:
      - "rviz2"

  # Optional: File/glob paths removed from runtime container images.
  # System paths (/usr/..., /opt/ros/...) apply to both service and skill
  # images; /workspace/install/... paths apply only to the service image.
  prune:
    patterns:
      - "/usr/include"
      - "/usr/share/doc"
      - "/usr/share/man"

# Distinct simulation and hardware targets (use instead of 'target'):
# targets:
#   sim:
#     type: "launch"
#     package: "my_robot_bringup"
#     name: "robot.launch.py"
#     arguments: ["sim:=true"]
#   real:
#     type: "launch"
#     package: "my_robot_bringup"
#     name: "robot.launch.py"
#     arguments: ["sim:=false"]

# ------------------------------------------------------------------------------
# 2. NESTED MESSAGE TYPES (Optional)
# Required when request_fields or response_fields reference nested ROS messages.
# Each field entry is formatted as "<type> <field_name>", supporting arrays
# ("double[]", "double[3]") and nested message names ("std_msgs/msg/Header").
# ------------------------------------------------------------------------------
types:
  - name: "builtin_interfaces/msg/Time"
    fields:
      - "int32 sec"
      - "uint32 nanosec"
  - name: "std_msgs/msg/Header"
    fields:
      - "builtin_interfaces/msg/Time stamp"
      - "string frame_id"

# ------------------------------------------------------------------------------
# 3. INTERFACES
# ROS 2 endpoints bridged by aquarium_gateway and compiled into Skills.
# ------------------------------------------------------------------------------
interfaces:
  - # Required: "ros_service", "ros_topic_subscription", or "ros_action_client"
    internal_interface: "ros_service"

    # Required: ROS 2 graph name of the service, topic, or action
    internal_name: "/add_two_ints"

    # Required: ROS 2 type ("<pkg>/srv/<Type>", "<pkg>/msg/<Type>", or
    # "<pkg>/action/<Type>")
    internal_type: "example_interfaces/srv/AddTwoInts"

    # Optional: RIHS01 type hash captured during introspection
    type_hash: "RIHS01_e118de6bf5eeb66a2491b5bda11202e7b68f198d6f67922cf30364858239c81a"

    # Required: "zenoh_queryable_proto" (for Flowstate Skills) or "grpc"
    # (for proto-defined gRPC services generated via 'aquarium generate ros')
    external_interface: "zenoh_queryable_proto"

    # Required: Zenoh queryable key (for "zenoh_queryable_proto") or full gRPC
    # method path "/<pkg>.<Service>/<Method>" (for "grpc")
    external_name: "add_two_ints"

    # Optional: Protobuf type prefix; defaults to internal_type with '/' -> '.'
    external_type: "example_interfaces.srv.AddTwoInts"

    # Message field layouts ("<type> <name>"):
    # - ros_service: request_fields and response_fields
    # - ros_action_client: request_fields (Goal) and response_fields (Result)
    # - ros_topic_subscription: response_fields only
    request_fields:
      - "int64 a"
      - "int64 b"
    response_fields:
      - "int64 sum"
```

---

## Interface Modes

* **`ros_service`:** Synchronous request/response call bridged to a ROS 2
  service server.
* **`ros_topic_subscription`:** Subscribes to a ROS 2 topic in the background
  and caches the latest message. Generated skills automatically expose optional
  polling parameters `wait_for_next_message` (`bool`, default `false`) and
  `timeout_seconds` (`double`); if no message has arrived, the gateway returns
  `NO_DATA` (or `TIMEOUT` when waiting).
* **`ros_action_client`:** Sends a ROS 2 action goal (`request_fields`), waits
  for terminal completion, and maps the action result (`response_fields`). An
  optional `string action_name` request field overrides `internal_name` per
  invocation when present.

---

## Examples

### ROS 2 Topic Subscription

```yaml
interfaces:
  - internal_interface: "ros_topic_subscription"
    internal_name: "/chatter"
    internal_type: "example_interfaces/msg/String"
    external_interface: "zenoh_queryable_proto"
    external_name: "chatter"
    response_fields:
      - "string data"

target:
  type: "node"
  package: "demo_nodes_cpp"
  name: "talker"
```

### ROS 2 Action Client with Nested Types

```yaml
types:
  - name: "sensor_msgs/msg/JointState"
    fields:
      - "std_msgs/msg/Header header"
      - "string[] name"
      - "double[] position"
      - "double[] velocity"
      - "double[] effort"
  - name: "std_msgs/msg/Header"
    fields:
      - "builtin_interfaces/msg/Time stamp"
      - "string frame_id"
  - name: "builtin_interfaces/msg/Time"
    fields:
      - "int32 sec"
      - "uint32 nanosec"

interfaces:
  - internal_interface: "ros_action_client"
    internal_name: "/robotiq_gripper_controller/gripper_cmd"
    internal_type: "control_msgs/action/ParallelGripperCommand"
    external_interface: "zenoh_queryable_proto"
    external_name: "gripper_cmd"
    request_fields:
      - "sensor_msgs/msg/JointState command"
      - "string action_name"
    response_fields:
      - "sensor_msgs/msg/JointState state"
      - "bool stalled"
      - "bool reached_goal"

target:
  type: "launch"
  package: "aquarium"
  name: "robotiq.launch.xml"
  dependencies:
    - "robotiq_driver"
    - "robotiq_description"
```
