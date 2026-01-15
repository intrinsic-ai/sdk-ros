# Flowstate Diagnostics ROS Bridge

## 1. Executive Summary & Architecture

**Scope:** Adds a new bridge plugin that polls the Flowstate System Service via gRPC and publishes system health statuses to the standard ROS `/diagnostics` topic.

**Architecture & Data Flow:** The system follows a **Polling Bridge** pattern designed to translate internal system states into standard ROS messages:

* **Source (Flowstate):** The `Diagnostics` class connects to the `SystemServiceState` gRPC service (default: `istio-ingressgateway.app-ingress.svc.cluster.local:80`).

* **Bridge (ROS Node):** The `DiagnosticsBridge` plugin is loaded by the main node. It holds a timer that triggers an update loop at a configurable rate (default 1Hz).

* **Translation:** The system retrieves `InstanceState` protobufs and converts them into `diagnostic_msgs::msg::DiagnosticArray` messages.

* **Output:** The aggregated data is published to the `/diagnostics` topic.



**Implementation Manifest (New Components):** The architecture described above is realized through the following new files:


* `src/diagnostics.cpp` & `.hpp`: Contains the **Source** logic (the gRPC client).


* `src/bridges/diagnostics_bridge.cpp`: Contains the **Bridge** and **Translation** logic (the ROS plugin).

---


## 2. Interface Updates

### 2.1 ROS Topic Interface

| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Publisher | Aggregated system status messages. |

### 2.2 Logic Mapping (Proto → ROS)
A helper function `to_diagnostic_level` maps Flowstate enums to ROS diagnostic levels:

| Flowstate State Code | ROS Diagnostic Level | Meaning |
| :--- | :--- | :--- |
| `STATE_CODE_ENABLED` | **OK** (0) | System is healthy. |
| `STATE_CODE_DISABLED` | **WARN** (1) | System is idle/disabled but functional. |
| `STATE_CODE_STOPPED` | **WARN** (1) | System is stopped. |
| `STATE_CODE_ERROR` | **ERROR** (2) | System is in failure state. |
| *(Default/Other)* | **STALE** (3) | Unknown state. |


### 2.3 Configuration Parameters
New parameters were added to `flowstate_ros_bridge_default_config.pbtxt` and handled in `flowstate_ros_bridge_main.cpp`:

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