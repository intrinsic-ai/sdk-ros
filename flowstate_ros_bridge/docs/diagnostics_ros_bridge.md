# Flowstate Diagnostics ROS Bridge

## 1. Overview

This feature introduces a bridge plugin that exposes Flowstate system health to the ROS ecosystem. Instead of complex event handling, it uses a simple polling loop to fetch status updates and publish them to the standard `/diagnostics` topic.

**How it works:**

* **Connection:** The bridge connects to the Flowstate gRPC service (defaulting to `istio-ingressgateway.app-ingress.svc.cluster.local:80`).

* **Polling:** A timer triggers the bridge to fetch the latest system state at a configurable rate (default 1Hz).

* **Translation:** It automatically converts the internal [`InstanceState`](https://github.com/intrinsic-ai/sdk/blob/main/intrinsic/assets/services/proto/v1/system_service_state.proto#L84) (Protobuf) into standard ROS `DiagnosticArray` messages and publishes them.


The architecture described above is accomplished through the following files:

* [`src/diagnostics.cpp`](../src/diagnostics.cpp) & [`.hpp`](../include/flowstate_ros_bridge/diagnostics.hpp): Contains the **Source** logic (the gRPC client).

* [`src/bridges/diagnostics_bridge.cpp`](../src/bridges/diagnostics_bridge.cpp) & [`.hpp`](../src/bridges/diagnostics_bridge.hpp): Contains the **Bridge** and **Translation** logic (the ROS plugin).


## 2. Interface Updates

### 2.1 ROS Topic Interface

| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Publisher | Aggregated system status messages. |

### 2.2 Logic Mapping (Proto → ROS)
A helper function `to_diagnostic_level` maps Flowstate enums, derived from [`InstanceState`](https://github.com/intrinsic-ai/sdk/blob/main/intrinsic/assets/services/proto/v1/system_service_state.proto#L84) protobuf messages, to ROS diagnostic levels:
| Flowstate State Code | ROS Diagnostic Level | Meaning |
| :--- | :--- | :--- |
| `STATE_CODE_ENABLED` | **OK** (0) | System is healthy. |
| `STATE_CODE_DISABLED` | **WARN** (1) | System is idle/disabled but functional. |
| `STATE_CODE_STOPPED` | **WARN** (1) | System is stopped. |
| `STATE_CODE_ERROR` | **ERROR** (2) | System is in failure state. |
| *(Default/Other)* | **STALE** (3) | Unknown state. |


### 2.3 Configuration Parameters
The following parameters are defined in `flowstate_ros_bridge_default_config.pbtxt` and processed by the main node:

| Parameter Name | Default Value | Description |
| :--- | :--- | :--- |
| `diagnostics_service_address` | `istio-ingressgateway...` | gRPC endpoint for the system service. |
| `diagnostics_deadline_seconds` | `5` | Timeout for gRPC connection attempts. |

---

## 3. Usage & Verification

### 3.1 Configuration Check
As a safety check, ensure that the **`flowstate_ros_bridge::DiagnosticsBridge`** plugin is included in the `bridge_plugins` list within your configuration parameters. This is typically enabled by default, but verify it if the topic does not appear.

### 3.2 Deployment & Testing Steps

1.  **Upload Bundle:** Upload the `.tar` bundle to the platform.
    * *(For specific upload instructions, please refer to: [flowstate_ros_bridge/README.md](flowstate_ros_bridge/README.md))*.

2.  **Verify Connection (Flowstate Logs):** Check the Flowstate Platform logs to confirm the bridge has initialized correctly. Look for the success message:
    > `"Successfully connected to diagnostics service!"`

3.  **Verify ROS Output:** Switch to your ROS environment and echo the diagnostics topic to validate the data:
    ```bash
    ros2 topic echo /diagnostics
    ```
    * **Expected Result:** You should see `DiagnosticStatus` messages listing services (e.g., `service:perception`, `service:robot_controller`) with their current `Platform State`, as shown below:
        ```yaml
        header:
        stamp: ...
        status:
        - level: "\0"
            name: "service:perception"
            message: "STATE_CODE_ENABLED"
            hardware_id: "platform"
            values:
            - key: "Platform State"
              value: "STATE_CODE_ENABLED"
        - level: "\0"
            name: "service:robot_controller"
            ...
        ```