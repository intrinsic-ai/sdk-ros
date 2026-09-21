# Generic ROS 2 Action Skill for Intrinsic

This project provides a Generic Intrinsic Skill that can interface with any ROS 2 Action Server.
It serves as a bridge between Intrinsic Flowstate processes and the ROS 2 ecosystem using Zenoh for high-performance communication.

## 1. Architectural Overview

### The Python Node ([`generic_action_skill.py`](/generic_action_skill/generic_action_skill/generic_action_skill.py))

This is the core translator. It is designed to be completely decoupled from any specific ROS 2 message types at compile time.

* Dynamic Loading: It uses `rosidl_runtime_py.utilities.get_action` to load ROS 2 Action classes dynamically based on a string input.

* YAML Mapping: It converts Intrinsic string inputs (YAML) into structured ROS 2 Action Goal messages.

* Status Mapping: It translates ROS 2 execution states (Succeeded, Aborted, Canceled) into strings visible in Flowstate.

* SDK Compatibility: It implements a manual runner with dynamic gRPC discovery to ensure it works across different versions of the Intrinsic SDK (Jazzy/Rolling) and different environments (Local Docker vs. K8s Cluster).

### The Manifest ([`generic_action_skill.manifest.textproto`](./src/generic_action_skill.manifest.textproto))

The manifest defines how the Intrinsic platform perceives the skill:

* Metadata: Links the skill to the unique ID `com.example.generic_action_skill`.

* Interfaces: Specifies the gRPC ports and the logic entry points.

* Asset Linkage: Maps the skill to the specific container image built by the Dockerfile.

### The Protobuf ([`generic_action_skill.proto`](./src/generic_action_skill.proto))

* Inputs: `action_name`, `action_type`, `goal_yaml`, and `timeout_sec`.

* Outputs: `status` and `result_yaml`.

## 2. Prerequisites

1. `inctl` configured for your organization.

2. Docker installed locally.

3. `kubectl` and `k9s` for cluster monitoring.

4. An active Intrinsic Solution/Workcell.

## 3. Step by Step Execution Workflow

### Step 1: Build the Skill Bundle

This command build the code, the Intrinsic SDK, and the necessary ROS dependencies into a tar image.

```bash
./src/sdk-ros/scripts/build_container.sh \
  --ros_distro "jazzy" \
  --skill_package generic_action_skill \
  --skill_name generic_action_skill \
  --dependencies "ros-jazzy-common-interfaces ros-jazzy-action-tutorials-cpp ros-jazzy-action-tutorials-interfaces python3-absl python3-retrying python3-yaml"
```

_Note: You will need to add to the `dependencies` tag the packages required for the action being used. Those will be installed during the build process._

The next command then wraps the tar image with additiona Flowstate metadata (manifest and proto descriptors) to make it into a deployable tarball.

```bash
./src/sdk-ros/scripts/build_bundle.sh \
  --skill_package generic_action_skill \
  --skill_name generic_action_skill \
  --manifest_path src/sdk-ros/generic_action_skill/src/generic_action_skill.manifest.textproto
```

### Step 2: Install to the Cluster

Upload the bundle to your specific solution branch.

```bash
inctl skill install images/generic_action_skill/generic_action_skill.bundle.tar \
  --org=your-org-name \
  --solution=your-solution-id
```

### Step 3: Start the ROS 2 Action Server (e.g., Fibonacci)

The skill is a Client; it needs a Server to talk to.
Launch a manual server pod in your cluster.
For this example, the [Fibonacci action](https://docs.ros.org/en/jazzy/p/action_tutorials_interfaces/action/Fibonacci.html) and [action_tutorials_cpp](https://docs.ros.org/en/jazzy/p/action_tutorials_cpp/) will be used.

For further information on how to setup `k9s`, click [here](go/intrinsic-k9s).

```bash
kubectl --context vm run fibonacci-server-manual \
  --image=direct.upload.local/com.example.generic_action_skill.generic_action_skill:dev.user \
  --env="RMW_IMPLEMENTATION=rmw_zenoh_cpp" \
  --env="ROS_HOME=/tmp" \
  --env="ZENOH_CONFIG_OVERRIDE=connect/endpoints=[\"tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447\"]" \
  --overrides='{"spec": {"template": {"spec": {"imagePullPolicy": "IfNotPresent"}}}}' \
  -- ros2 run action_tutorials_cpp fibonacci_action_server
```

_Note: You can check the `image` tag from the previous run output with `inctl install` and replace it there._

### Step 4: Configure Flowstate

1. Add the Generic Action Skill node to your process.

2. Inputs:

    * `action_name`: /fibonacci

    * `action_type`: action_tutorials_interfaces/action/Fibonacci

    * `goal_yaml`: order: 5

    * `timeout_sec`: 10.0

3. Outputs:

    * Map `result_yaml` to a Blackboard variable (e.g., `fibonacci_result`).

### Step 5: Run and Verify

* Execute the process in Flowstate.

* Check the Blackboard tab in the Executive.

* You should see `fibonacci_result` populated with: `sequence: [0, 1, 1, 2, 3, 5]`.

## 4. Local Development & Debugging

To iterate on the Python code without rebuilding the whole bundle:

```bash
docker run --rm -it \
  -v $(pwd)/src/sdk-ros/generic_action_skill/generic_action_skill/generic_action_skill.py:/opt/ros/overlay/install/lib/generic_action_skill/generic_action_skill_main \
  --entrypoint /run_skill.sh \
  generic_action_skill:generic_action_skill --port=8003
```

This mounts the local `.py` file directly into the container in order to see code changes immediately upon restarting the container.
