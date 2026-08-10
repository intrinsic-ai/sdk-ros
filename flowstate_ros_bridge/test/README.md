# Rapid Object Visualization and Integration Test

This directory contains integration test tools for verifying the stability, timestamp synchronization, and high concurrency mesh resource delivery of `flowstate_ros_bridge` when rendering large numbers of 3D objects in RViz.

---

## Overview

The primary test script, [`test_rapid_object_visualization.py`](test_rapid_object_visualization.py), simulates RViz's runtime marker rendering and TF transform resolution under high-stress conditions (e.g. 400+ dynamically spawned objects).

### What the Test Verifies

1. **Marker Delivery Completeness**: Monitors incoming `visualization_msgs/msg/MarkerArray` topics to ensure no object marker messages are dropped during rapid spawning.
2. **Parallel HTTP Mesh Downloads**: Replicates RViz's parallel mesh downloading behavior (using 4 worker threads to query `http://localhost:8123/...`) to verify that `rviz_http_proxy` can handle concurrent requests without socket queuing timeouts.
3. **TF Frame Transform Validity**: Queries ROS 2 `tf2_ros::Buffer` to ensure all object frame transforms relative to `root` exist and can be resolved.
4. **Temporal Stability**: Holds an active monitoring window after all objects are spawned to detect spurious marker deletions or frame lock stamp expirations.

---

## Environment & Setup

Ensure the ROS 2 environment and built workspace are sourced prior to running the test:

```bash
# Source ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Source workspace install directory
source install/setup.bash

# Ensure Zenoh RMW is configured
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Make sure `flowstate_ros_bridge` is running before launching the test.

---

## Running the Test

### Standard 400 Object Stress Test

To execute a full stress test with 400 dynamically spawned objects and a 120-second timeout:

```bash
python3 /path/to/workspace/src/sdk-ros/flowstate_ros_bridge/test/test_rapid_object_visualization.py --spawn-count 400 --timeout 120
```

### Quick Verification Run (50 Objects)

For quick smoke testing after code modifications:

```bash
python3 /path/to/workspace/src/sdk-ros/flowstate_ros_bridge/test/test_rapid_object_visualization.py --spawn-count 50 --timeout 30
```

---

## CLI Options Reference

| Argument | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `--spawn-count` | `int` | `400` | Number of newly spawned stress objects expected during the test run. |
| `--expected-total` | `int` | `None` | Overrides total expected objects count (ignoring initial baseline auto-detection). |
| `--baseline-window` | `float` | `2.0` | Time (in seconds) spent sampling baseline pre-existing scene objects before testing. |
| `--stability-window` | `float` | `5.0` | Time (in seconds) to continuously monitor marker stability after reaching target count. |
| `--fixed-frame` | `str` | `"root"` | World fixed frame ID used for TF 3D world pose lookups. |
| `--timeout` | `float` | `120.0` | Total maximum duration (in seconds) to wait for target object count to be reached. |
| `--no-verify-http` | `flag` | `False` | Disables HTTP GET download verification of GLTF mesh resources. |

---

## Interpreting Test Results

At the end of execution, the script prints a detailed summary block:

```text
==================================================
        VISUALIZATION INTEGRATION TEST RESULTS     
==================================================
Elapsed Time to Target: 23.93 seconds
Total Test Duration:    29.45 seconds
Initial Baseline Objects:21
Target Total Objects:   421
Final Unique Objects:   421
Final Marker Elements:  421
Valid TF 3D World Poses:414
Non-Zero/Expired TF Stamps:0
Missing TF Frames:      0
Spurious Deletions:     0
Total HTTP Mesh Requests:821
Successful Mesh HTTPs:  821
Failed Mesh HTTP URLs:  0
STATUS: SUCCESS (All objects delivered, TF transforms valid, all mesh HTTP requests succeeded)
==================================================
```

### Key Failure Signals

- **`Failed Mesh HTTP URLs > 0`**: Single-threaded `rviz_http_proxy` bottlenecking on concurrent GET requests, causing mesh download timeouts in RViz.
- **`Missing TF Frames > 0`**: Objects published in `MarkerArray` whose frame transforms are absent or unresolvable in `tf2`.
- **`Spurious Deletions > 0`**: Markers unexpectedly deleted or cleared while the test is monitoring stability.
