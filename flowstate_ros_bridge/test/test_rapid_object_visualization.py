#!/usr/bin/env python3
# Copyright 2025 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
from concurrent.futures import ThreadPoolExecutor
import sys
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


class MarkerMonitorNode(Node):

  def __init__(
      self,
      spawn_count: int,
      expected_total: int | None,
      timeout_sec: float,
      baseline_window_sec: float = 2.0,
      fixed_frame: str = "root",
      verify_http_meshes: bool = True,
  ):
    super().__init__("marker_integration_test_monitor")
    self.spawn_count = spawn_count
    self.user_expected_total = expected_total
    self.timeout_sec = timeout_sec
    self.baseline_window_sec = baseline_window_sec
    self.fixed_frame = fixed_frame
    self.verify_http_meshes = verify_http_meshes

    # Replicate RViz's exact 10-second TF cache buffer duration
    self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ThreadPool simulating RViz's concurrent mesh downloads (4 parallel workers matching Assimp/OGRE)
    self.http_executor = ThreadPoolExecutor(max_workers=4)
    self.http_lock = threading.Lock()

    # QoS matching RViz setup (Transient Local + Reliable)
    qos_profile = QoSProfile(
        depth=100,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    # Dictionary mapping (ns, id) -> (frame_id, stamp_time, frame_locked)
    self.active_markers = {}

    # Track mesh HTTP fetches per marker: total, succeeded, failed (protected by http_lock)
    self.total_mesh_requests = 0
    self.successful_mesh_requests = 0
    self.failed_mesh_requests = []

    self.baseline_count = 0
    self.target_total = 0
    self.start_time = time.time()
    self.baseline_established = False

    # Track unexpected deletion events
    self.spurious_deletions_count = 0

    self.subscription = self.create_subscription(
        MarkerArray, "/workcell_markers", self._marker_callback, qos_profile
    )

  def get_unique_object_namespaces(self) -> set:
    """Returns set of unique object namespaces (e.g. root/box_0/base)."""
    return {ns for (ns, _id) in self.active_markers.keys()}

  def analyze_tf_and_poses(self):
    """Uses TF2 buffer to lookup actual 3D world poses relative to fixed frame (root).

    Replicates RViz's frame-locked marker timestamp lookup and 10s buffer limit.
    """
    valid_world_poses = {}
    missing_tf_frames = []
    invalid_stamp_frames = []

    for (ns, _id), (frame_id, stamp_time, frame_locked) in self.active_markers.items():
      # 1. Test latest TF transform (Time(0))
      try:
        t_latest = self.tf_buffer.lookup_transform(
            self.fixed_frame, frame_id, Time()
        )
        pos = (
            round(t_latest.transform.translation.x, 2),
            round(t_latest.transform.translation.y, 2),
            round(t_latest.transform.translation.z, 2),
        )
        valid_world_poses[pos] = valid_world_poses.get(pos, 0) + 1
      except (
          tf2_ros.LookupException,
          tf2_ros.ConnectivityException,
          tf2_ros.ExtrapolationException,
      ):
        missing_tf_frames.append(frame_id)

      # 2. Replicate RViz timestamp check:
      # If marker is NOT frame_locked and has non-zero stamp, RViz attempts exact stamp lookup.
      # If marker IS frame_locked, RViz uses latest transform Time(0) dynamically on every frame.
      if not frame_locked and stamp_time.nanoseconds != 0:
        try:
          self.tf_buffer.lookup_transform(
              self.fixed_frame, frame_id, stamp_time
          )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
          invalid_stamp_frames.append(frame_id)

    return len(valid_world_poses), missing_tf_frames, invalid_stamp_frames

  def establish_baseline(self):
    """Establishes baseline active scene objects present before spawning stress objects."""
    unique_objects = self.get_unique_object_namespaces()
    self.baseline_count = len(unique_objects)

    if self.user_expected_total is not None:
      self.target_total = self.user_expected_total
    else:
      self.target_total = self.baseline_count + self.spawn_count
    self.baseline_established = True

    self.get_logger().info(
        f"Baseline established: {self.baseline_count} unique active objects"
        f" ({len(self.active_markers)} total marker elements)."
    )
    self.get_logger().info(
        f"Target total objects: {self.target_total} (Baseline:"
        f" {self.baseline_count} + Spawned: {self.spawn_count})"
    )

  def _fetch_mesh_http(self, marker_ns: str, url: str):
    """Worker function to test HTTP download matching RViz's timeout."""
    if not url.startswith("http://") and not url.startswith("https://"):
      return

    with self.http_lock:
      self.total_mesh_requests += 1

    try:
      req = urllib.request.Request(url, method="GET")
      # RViz's resource retriever times out on slow/blocked single-threaded HTTP responses
      with urllib.request.urlopen(req, timeout=1.5) as resp:
        content = resp.read()
        if resp.status == 200 and len(content) > 0:
          with self.http_lock:
            self.successful_mesh_requests += 1
        else:
          err_msg = f"HTTP status {resp.status}, size {len(content)}"
          with self.http_lock:
            self.failed_mesh_requests.append((marker_ns, url, err_msg))
          self.get_logger().error(
              f"HTTP mesh fetch failed for {marker_ns} ({url}): {err_msg}"
          )
    except Exception as e:
      with self.http_lock:
        self.failed_mesh_requests.append((marker_ns, url, str(e)))
      self.get_logger().error(
          f"HTTP mesh fetch failed for {marker_ns} ({url}): {e}"
      )

  def _marker_callback(self, msg: MarkerArray):
    for marker in msg.markers:
      key = (marker.ns, marker.id)
      stamp_time = Time.from_msg(marker.header.stamp)

      if marker.action in (Marker.ADD, Marker.MODIFY):
        self.active_markers[key] = (
            marker.header.frame_id,
            stamp_time,
            marker.frame_locked,
        )

        if self.verify_http_meshes and marker.mesh_resource:
          self.http_executor.submit(
              self._fetch_mesh_http, marker.ns, marker.mesh_resource
          )
      elif marker.action == Marker.DELETE:
        if key in self.active_markers:
          self.spurious_deletions_count += 1
          self.get_logger().warning(
              f"Received DELETE marker for active object {marker.ns}"
          )
        self.active_markers.pop(key, None)
      elif marker.action == Marker.DELETEALL:
        if marker.ns:
          keys_to_remove = [k for k in self.active_markers if k[0] == marker.ns]
          if keys_to_remove:
            self.spurious_deletions_count += len(keys_to_remove)
            self.get_logger().warning(
                f"Received DELETEALL marker for active namespace {marker.ns}"
            )
          for k in keys_to_remove:
            self.active_markers.pop(k, None)
        else:
          if self.active_markers:
            self.spurious_deletions_count += len(self.active_markers)
            self.get_logger().warning(
                "Received global DELETEALL marker wiping active markers"
            )
          self.active_markers.clear()

    if self.baseline_established:
      elapsed = time.time() - self.start_time
      unique_objects_count = len(self.get_unique_object_namespaces())
      self.get_logger().info(
          f"[{elapsed:.1f}s] Active objects: {unique_objects_count} /"
          f" {self.target_total} (Valid HTTP:"
          f" {self.successful_mesh_requests}/{self.total_mesh_requests}, Failed HTTP:"
          f" {len(self.failed_mesh_requests)})"
      )

  def is_target_reached(self) -> bool:
    unique_objects_count = len(self.get_unique_object_namespaces())
    return (
        self.baseline_established and unique_objects_count >= self.target_total
    )

  def is_timed_out(self) -> bool:
    return (time.time() - self.start_time) > self.timeout_sec


def main():
  parser = argparse.ArgumentParser(
      description="Integration test for WorldBridge rapid object visualization"
  )
  parser.add_argument(
      "--spawn-count",
      type=int,
      default=400,
      help=(
          "Number of NEW objects spawned during stress test (default: 400)."
          " Added to initial scene baseline count."
      ),
  )
  parser.add_argument(
      "--expected-total",
      type=int,
      default=None,
      help="Explicit total expected objects (overrides baseline + spawn_count)",
  )
  parser.add_argument(
      "--baseline-window",
      type=float,
      default=2.0,
      help="Seconds to sample initial scene markers before spawning (default: 2.0)",
  )
  parser.add_argument(
      "--stability-window",
      type=float,
      default=5.0,
      help=(
          "Seconds to hold and verify stability after target objects are"
          " reached (default: 5.0)"
      ),
  )
  parser.add_argument(
      "--fixed-frame",
      type=str,
      default="root",
      help="World fixed frame ID for TF pose lookups (default: 'root')",
  )
  parser.add_argument(
      "--timeout",
      type=float,
      default=120.0,
      help="Total timeout in seconds to wait for objects (default: 120.0)",
  )
  parser.add_argument(
      "--no-verify-http",
      action="store_true",
      help="Skip HTTP GET download verification for GLTF mesh resources",
  )
  args, _ = parser.parse_known_args()

  rclpy.init()
  node = MarkerMonitorNode(
      spawn_count=args.spawn_count,
      expected_total=args.expected_total,
      timeout_sec=args.timeout,
      baseline_window_sec=args.baseline_window,
      fixed_frame=args.fixed_frame,
      verify_http_meshes=not args.no_verify_http,
  )

  print(
      f"Sampling initial scene baseline objects for {args.baseline_window}s..."
  )
  baseline_start = time.time()
  while rclpy.ok() and (time.time() - baseline_start) < args.baseline_window:
    rclpy.spin_once(node, timeout_sec=0.1)

  node.establish_baseline()
  print("Beginning monitoring for spawned objects...")

  try:
    while rclpy.ok() and not node.is_target_reached() and not node.is_timed_out():
      rclpy.spin_once(node, timeout_sec=0.1)

    target_reached = node.is_target_reached()
    elapsed_to_target = time.time() - node.start_time

    if target_reached:
      print(
          f"\nTarget {node.target_total} objects reached! Monitoring"
          f" stability for {args.stability_window}s to finish concurrent HTTP"
          " mesh fetches..."
      )
      stability_start = time.time()

      while (
          rclpy.ok()
          and (time.time() - stability_start) < args.stability_window
      ):
        rclpy.spin_once(node, timeout_sec=0.1)

    node.http_executor.shutdown(wait=True)

    total_elapsed = time.time() - node.start_time
    final_unique_objects = len(node.get_unique_object_namespaces())
    final_active_markers = len(node.active_markers)
    failed_meshes_count = len(node.failed_mesh_requests)

    # Perform final TF transform analysis
    unique_world_poses, missing_tf_frames, invalid_stamp_frames = (
        node.analyze_tf_and_poses()
    )
    invalid_stamps_count = len(invalid_stamp_frames)

    print("\n==================================================")
    print("        VISUALIZATION INTEGRATION TEST RESULTS     ")
    print("==================================================")
    print(f"Elapsed Time to Target: {elapsed_to_target:.2f} seconds")
    print(f"Total Test Duration:    {total_elapsed:.2f} seconds")
    print(f"Initial Baseline Objects:{node.baseline_count}")
    print(f"Target Total Objects:   {node.target_total}")
    print(f"Final Unique Objects:   {final_unique_objects}")
    print(f"Final Marker Elements:  {final_active_markers}")
    print(f"Valid TF 3D World Poses:{unique_world_poses}")
    print(f"Non-Zero/Expired TF Stamps:{invalid_stamps_count}")
    print(f"Missing TF Frames:      {len(missing_tf_frames)}")
    print(f"Spurious Deletions:     {node.spurious_deletions_count}")
    print(f"Total HTTP Mesh Requests:{node.total_mesh_requests}")
    print(f"Successful Mesh HTTPs:  {node.successful_mesh_requests}")
    print(f"Failed Mesh HTTP URLs:  {failed_meshes_count}")

    is_successful = (
        target_reached
        and final_unique_objects >= node.target_total
        and node.spurious_deletions_count == 0
        and failed_meshes_count == 0
        and invalid_stamps_count == 0
        and len(missing_tf_frames) == 0
        and (
            node.spawn_count <= 1
            or unique_world_poses > node.baseline_count + 1
        )
    )

    if is_successful:
      print(
          "STATUS: SUCCESS (All objects delivered, TF transforms valid, all"
          " mesh HTTP requests succeeded)"
      )
      print("==================================================\n")
      sys.exit(0)
    else:
      print(
          "STATUS: FAILED (Spurious deletions, invalid frame-locked TF stamps, missing TF"
          " transforms, missing objects, or HTTP mesh errors detected)"
      )
      if invalid_stamps_count > 0:
        print(
            f"ERROR: Frame-locked markers have non-zero header stamps for {invalid_stamps_count}"
            " objects (causes RViz to hide boxes after 10s TF buffer expiration!):"
        )
        for frame in invalid_stamp_frames[:15]:
          print(f"   - {frame}")
      if failed_meshes_count > 0:
        print(
            f"ERROR: Failed/Timed out HTTP mesh downloads for {failed_meshes_count}"
            " object markers (single-threaded rviz_http_proxy socket queue bottleneck!):"
        )
        for ns, url, err in node.failed_mesh_requests[:15]:
          print(f"   - {ns} ({url}): {err}")
      if len(missing_tf_frames) > 0:
        print(
            f"ERROR: Missing TF transforms for {len(missing_tf_frames)} objects"
            " (causes missing boxes in RViz!):"
        )
        for frame in missing_tf_frames[:10]:
          print(f"   - {frame}")
      if (
          unique_world_poses <= node.baseline_count + 1
          and node.spawn_count > 1
      ):
        print(
            "ERROR: Marker TF poses are clustered at duplicate 3D coordinates!"
            f" (Unique 3D poses: {unique_world_poses} for"
            f" {final_unique_objects} objects)"
        )
      if node.spurious_deletions_count > 0:
        print(
            f"ERROR: Received {node.spurious_deletions_count} spurious"
            " DELETE/DELETEALL markers!"
        )
      if final_unique_objects < node.target_total:
        print(f"ERROR: Missing {node.target_total - final_unique_objects} objects!")
      print("==================================================\n")
      sys.exit(1)

  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
