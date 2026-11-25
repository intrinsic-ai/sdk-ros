#!/bin/bash
# Copyright 2026 Intrinsic Innovation LLC
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

set -euo pipefail

NO_CACHE=0
SKIP_BUILD=0
for arg in "$@"; do
  case "${arg}" in
    --no-cache)
      NO_CACHE=1
      ;;
    --skip-build)
      SKIP_BUILD=1
      ;;
    -h|--help)
      echo "Usage: $0 [--no-cache] [--skip-build]"
      exit 0
      ;;
    *)
      echo "Error: Unknown argument '${arg}'." >&2
      echo "Usage: $0 [--no-cache] [--skip-build]" >&2
      exit 1
      ;;
  esac
done

if [ ! -d "src" ]; then
  echo "Error: Run this script from the ROS workspace root containing src/."
  exit 1
fi

if [ -n "${XDG_RUNTIME_DIR:-}" ] && [ ! -d "${XDG_RUNTIME_DIR}" ]; then
  unset XDG_RUNTIME_DIR
fi

WORKSPACE_DIR="$(pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AQUARIUM_PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIR="build/aquarium_test_add_two_ints"
ROS_DISTRO_NAME="${ROS_DISTRO:-lyrical}"

rm -rf "${OUTPUT_DIR}"

echo "=== 1. Generating Skill and Service Config ==="
PYTHONPATH="${AQUARIUM_PKG_DIR}" python3 -m aquarium.cli generate intrinsic \
  "${AQUARIUM_PKG_DIR}/test/test_aquarium_add_two_ints.yaml" \
  "${OUTPUT_DIR}"

SDK_ROS_DIR="$(PYTHONPATH="${AQUARIUM_PKG_DIR}" python3 -c \
  "from aquarium.workspace import find_sdk_ros_dir as f; \
print(f('${WORKSPACE_DIR}'))")"

BUILDAH_COMMON_ARGS=(
  --ignorefile "${OUTPUT_DIR}/dockerignore"
  --format docker
)
if [ "${NO_CACHE}" -eq 1 ]; then
  BUILDAH_COMMON_ARGS=(--no-cache "${BUILDAH_COMMON_ARGS[@]}")
  echo "=== Pulling Latest Upstream Base Images ==="
  buildah pull --policy always docker.io/library/ros:lyrical
  buildah pull --policy always docker.io/library/ros:lyrical-ros-core
fi

if [ "${SKIP_BUILD}" -eq 0 ]; then
  echo "=== 2a. Ensuring SDK Underlay Deps Image ==="
  if [ "${NO_CACHE}" -eq 1 ] || \
     ! buildah inspect --type image aquarium-sdk-underlay-deps:latest \
         >/dev/null 2>&1; then
    buildah bud "${BUILDAH_COMMON_ARGS[@]}" \
      -t aquarium-sdk-underlay-deps:latest \
      -f "${OUTPUT_DIR}/Dockerfile.underlay_deps" .
  fi

  echo "=== 2b. Ensuring SDK Underlay Image ==="
  if [ "${NO_CACHE}" -eq 1 ] || \
     ! buildah inspect --type image aquarium-sdk-underlay:latest \
         >/dev/null 2>&1; then
    buildah bud "${BUILDAH_COMMON_ARGS[@]}" \
      -t aquarium-sdk-underlay:latest \
      --build-arg "SDK_ROS_DIR=${SDK_ROS_DIR}" \
      -f "${OUTPUT_DIR}/Dockerfile.underlay" .
  fi

  echo "=== 3. Building Service Image ==="
  buildah bud "${BUILDAH_COMMON_ARGS[@]}" \
    -t demo-nodes-cpp-add-two-ints-server \
    --build-arg "OUTPUT_DIR=${OUTPUT_DIR}" \
    --build-arg "TARGET_PACKAGE=demo_nodes_cpp" \
    --build-arg "TARGET_APT_PACKAGE=ros-${ROS_DISTRO_NAME}-demo-nodes-cpp" \
    --build-arg "CONFIG_PATH=${OUTPUT_DIR}/aquarium.yaml" \
    --build-arg "LAUNCH_PATH=${OUTPUT_DIR}/aquarium_launch.xml" \
    --build-arg "ENTRYPOINT_PATH=${OUTPUT_DIR}/entrypoint.py" \
    -f "${OUTPUT_DIR}/Dockerfile.service" .

  echo "=== 4. Building Skill Image ==="
  buildah bud "${BUILDAH_COMMON_ARGS[@]}" \
    -t add_two_ints_skill:latest \
    --build-arg "OUTPUT_DIR=${OUTPUT_DIR}" \
    --build-arg SKILL_NAME=add_two_ints_skill \
    -f "${OUTPUT_DIR}/Dockerfile.skill" .
fi

TEST_PASSED=0

cleanup() {
  echo "=== Cleaning up containers ==="
  if [ "${TEST_PASSED}" -eq 0 ]; then
    echo "=== DEBUG: Skill Container Logs ==="
    podman logs skill_container || true
    echo "=== DEBUG: Service Container Logs ==="
    podman logs service_container || true
    echo "=== DEBUG: Dummy gRPC Logs ==="
    podman logs dummy_grpc || true
    echo "=== DEBUG: Zenoh Router Logs ==="
    podman logs zenoh_router || true
  fi
  podman rm -f service_container skill_container dummy_grpc zenoh_router \
    || true
  if [ "${TEST_PASSED}" -eq 1 ]; then
    rm -rf "${OUTPUT_DIR}"
  fi
}
trap cleanup EXIT

podman rm -f service_container skill_container dummy_grpc zenoh_router \
  >/dev/null 2>&1 || true

echo "=== 5. Starting Zenoh Router in Container ==="
podman run -d --name zenoh_router --network host \
  localhost/aquarium-sdk-underlay-deps:latest \
  ros2 run rmw_zenoh_cpp rmw_zenohd
sleep 2

echo "=== 6. Starting Dummy gRPC Server in Container ==="
DUMMY_GRPC_PY="${AQUARIUM_PKG_DIR}/test/dummy_grpc_server.py"
podman run -d --name dummy_grpc --network host \
  -v "${DUMMY_GRPC_PY}:/workspace/dummy_grpc_server.py:z" \
  add_two_ints_skill:latest \
  python3 /workspace/dummy_grpc_server.py 8089
sleep 2

echo "=== 7. Running Service Container ==="
podman run -d --name service_container --network host \
  demo-nodes-cpp-add-two-ints-server:latest

echo "=== 8. Running Skill Container ==="
podman run -d --name skill_container --network host \
  add_two_ints_skill:latest \
  --world_service_address localhost:8089 \
  --geometry_service_address localhost:8089 \
  --motion_planner_service_address localhost:8089 \
  --skill_registry_service_address localhost:8089

for _ in $(seq 1 30); do
  SVC_LOGS="$(podman logs service_container 2>&1 || true)"
  SKILL_LOGS="$(podman logs skill_container 2>&1 || true)"
  if grep -q "aquarium_gateway online" <<< "${SVC_LOGS}" && \
     grep -q "Skill service listening" <<< "${SKILL_LOGS}"; then
    break
  fi
  sleep 0.5
done

echo "=== 9. Verifying Default ROS_DOMAIN_ID=1 Isolation and E2E Call ==="
SVC_LOGS="$(podman logs service_container 2>&1)"
grep -q "ENTRYPOINT: ROS_DOMAIN_ID set to: 1" <<< "${SVC_LOGS}"

ZENOH_LOCAL_ENV='connect/endpoints=["tcp/127.0.0.1:7447"]'
DOM0_SERVICES="$(podman run --rm --network host \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ROS_DOMAIN_ID=0 \
  -e "ZENOH_CONFIG_OVERRIDE=${ZENOH_LOCAL_ENV}" \
  localhost/aquarium-sdk-underlay-deps:latest \
  ros2 service list --no-daemon --spin-time 3)"
if grep -q "^/add_two_ints$" <<< "${DOM0_SERVICES}"; then
  echo "ERROR: /add_two_ints leaked onto ROS_DOMAIN_ID=0" >&2
  exit 1
fi

DOM1_SERVICES="$(podman run --rm --network host \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ROS_DOMAIN_ID=1 \
  -e "ZENOH_CONFIG_OVERRIDE=${ZENOH_LOCAL_ENV}" \
  localhost/aquarium-sdk-underlay-deps:latest \
  ros2 service list --no-daemon --spin-time 3)"
grep -q "^/add_two_ints$" <<< "${DOM1_SERVICES}"

CLIENT_BIN="/opt/add_two_ints_skill_workspace/install/add_two_ints_skill"
CLIENT_BIN="${CLIENT_BIN}/lib/add_two_ints_skill/add_two_ints_skill_client"
CLIENT_OUT="$(podman run --rm --network host add_two_ints_skill:latest \
  "${CLIENT_BIN}" localhost:8001)"
echo "${CLIENT_OUT}"
grep -q "sum: 84" <<< "${CLIENT_OUT}"

echo "=== 10. Verifying RuntimeContext ros_domain_id=7 Override ==="
podman rm -f service_container skill_container >/dev/null 2>&1 || true

RUNTIME_CFG_PB="${WORKSPACE_DIR}/${OUTPUT_DIR}/runtime_config.pb"
podman run --rm --entrypoint /usr/bin/python3 \
  demo-nodes-cpp-add-two-ints-server:latest -c \
  "import sys; sys.path.append('/workspace/config_protos'); \
import runtime_context_pb2 as rc; \
import demo_nodes_cpp_add_two_ints_server_config_pb2 as sc; \
ctx = rc.RuntimeContext(); \
ctx.config.Pack(sc.DemoNodesCppAddTwoIntsServerConfig(ros_domain_id=7)); \
sys.stdout.buffer.write(ctx.SerializeToString())" > "${RUNTIME_CFG_PB}"

podman run -d --name service_container --network host \
  -e "ZENOH_CONFIG_OVERRIDE=${ZENOH_LOCAL_ENV}" \
  -v "${RUNTIME_CFG_PB}:/etc/intrinsic/runtime_config.pb:ro,z" \
  demo-nodes-cpp-add-two-ints-server:latest

podman run -d --name skill_container --network host \
  add_two_ints_skill:latest \
  --world_service_address localhost:8089 \
  --geometry_service_address localhost:8089 \
  --motion_planner_service_address localhost:8089 \
  --skill_registry_service_address localhost:8089

for _ in $(seq 1 30); do
  SVC_LOGS_7="$(podman logs service_container 2>&1 || true)"
  SKILL_LOGS_7="$(podman logs skill_container 2>&1 || true)"
  if grep -q "aquarium_gateway online" <<< "${SVC_LOGS_7}" && \
     grep -q "Skill service listening" <<< "${SKILL_LOGS_7}"; then
    break
  fi
  sleep 0.5
done

SVC_LOGS_7="$(podman logs service_container 2>&1)"
grep -q "ENTRYPOINT: ROS_DOMAIN_ID set to: 7" <<< "${SVC_LOGS_7}"

DOM1_AFTER="$(podman run --rm --network host \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ROS_DOMAIN_ID=1 \
  -e "ZENOH_CONFIG_OVERRIDE=${ZENOH_LOCAL_ENV}" \
  localhost/aquarium-sdk-underlay-deps:latest \
  ros2 service list --no-daemon --spin-time 3)"
if grep -q "^/add_two_ints$" <<< "${DOM1_AFTER}"; then
  echo "ERROR: /add_two_ints remained on ROS_DOMAIN_ID=1 after override" >&2
  exit 1
fi

DOM7_SERVICES="$(podman run --rm --network host \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ROS_DOMAIN_ID=7 \
  -e "ZENOH_CONFIG_OVERRIDE=${ZENOH_LOCAL_ENV}" \
  localhost/aquarium-sdk-underlay-deps:latest \
  ros2 service list --no-daemon --spin-time 3)"
grep -q "^/add_two_ints$" <<< "${DOM7_SERVICES}"

CLIENT_OUT_7="$(podman run --rm --network host add_two_ints_skill:latest \
  "${CLIENT_BIN}" localhost:8001)"
echo "${CLIENT_OUT_7}"
grep -q "sum: 84" <<< "${CLIENT_OUT_7}"

echo "=== CONTAINER TEST PASSED ==="
TEST_PASSED=1
