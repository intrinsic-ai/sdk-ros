# CLI Usage Guide

`aquarium` provides a unified CLI for introspecting ROS 2 applications,
building Flowstate Service and Skill `.bundle.tar` archives, encoding service
configuration overrides, and generating ROS 2 interfaces from Protobuf service
definitions.

## Command Summary

```text
aquarium assistant launch <PACKAGE> <LAUNCH_FILE> [args...] [-s <SELECTION>]
aquarium assistant run <PACKAGE> <EXECUTABLE> [args...] [-s <SELECTION>]
aquarium build [--privileged] [--no-cache] <AQUARIUM_CONFIG_FILE>
aquarium generate intrinsic [--privileged] <AQUARIUM_CONFIG_FILE> [OUTPUT_DIR]
aquarium config <INPUT_TEXTPROTO> [OUTPUT_BINPB] [-o <OUT>] [--proto <PROTO>]
aquarium generate ros --service <SVC> --proto_path <PROTO> --output_path <DIR>
aquarium run [--sim] <AQUARIUM_CONFIG_FILE>
```

---

## Workflow 1: Wrapping a ROS 2 Launch File or Node

### Step 1: Introspect Interfaces (`aquarium assistant`)

Requires an active ROS 2 environment with `rclpy` and `rmw_zenoh_cpp`. Start a
launch file or single node in the background and open the interactive terminal
setup assistant in the foreground:

```bash
# Wrap a ROS 2 launch file
aquarium assistant launch aquarium add_two_ints.launch.xml

# Or wrap a single ROS 2 executable
aquarium assistant run demo_nodes_cpp add_two_ints_server
```

Select the topics, services, or actions to expose and press `W` to write
`aquarium.yaml` in the current working directory (or pass `--selection` / `-s`
for non-interactive selection; see the
[TUI Setup Assistant guide](setup_assistant.md)). Note that `aquarium.yaml` is
plain declarative YAML and can also be written or edited by hand—see the
[`aquarium.yaml` Reference](aquarium_yaml.md).

### Step 2: Build Flowstate Bundles (`aquarium build`)

Run `aquarium build` from your workspace environment (requires `buildah` and
`protoc`, with `sdk-ros` present in `src/`):

```bash
aquarium build aquarium.yaml
```

Useful flags:
* `--privileged`: Configure the generated Flowstate Service manifest to run
  with a privileged container security context (for hardware drivers needing
  device access).
* `--no-cache`: Force clean `buildah` container image builds without layer
  caching.

What `aquarium build` produces under `build/aquarium_images/`:
1. **Flowstate Service bundle (`<service>.bundle.tar`):** Packages the target
   ROS 2 package closure, its `rosdep` runtime dependencies, the recorded
   launch/node entrypoint, and the `aquarium_gateway` binary into an OCI image
   and Flowstate service bundle.
2. **Flowstate Skill bundles (`<skill>.bundle.tar`):** Generates a dedicated
   `.proto` schema and statically-typed C++ skill container for each interface
   listed in `aquarium.yaml`. Skill containers contain no runtime reflection;
   they forward serialized Protobuf requests as Zenoh queries to
   `aquarium_gateway` and expose standard static parameter descriptors to the
   Flowstate UI.

To stage the packaged Dockerfiles and generate the intermediate manifests,
config protos, and skill sources under `build/aquarium_images/` without running
`buildah` or `inbuild`:

```bash
aquarium generate intrinsic [--privileged] aquarium.yaml [OUTPUT_DIR]
```

### Step 3: Sideload & Override Launch Arguments (`aquarium config` & `inctl`)

When wrapping a launch file (`type: "launch"`), `aquarium` inspects top-level
launch arguments (`<arg>`, `DeclareLaunchArgument`, or `arg:`), merges them
with any explicit `arguments` in `aquarium.yaml`, and writes two files into
`build/aquarium_images/`:
* `<service_name>_config.proto`: Protobuf schema with a field per launch
  argument (`bool`, `int32`, `double`, or `string` inferred from default
  values).
* `<service_tag>_default_config.textproto`: Default `google.protobuf.Any`
  configuration embedded into the service bundle.

At container startup, the service entrypoint reads
`/etc/intrinsic/runtime_config.pb` and forwards each configured field to
`ros2 launch` as `<arg_name>:=<value>`.

To override launch arguments at deployment time without rebuilding the bundle,
edit a `.textproto` file and encode it into a binary `google.protobuf.Any`
(`.binpb`) file with `aquarium config`:

```bash
# 1. Copy and edit the generated default config
cp build/aquarium_images/<service_tag>_default_config.textproto \
  custom_config.textproto

# 2. Encode the textproto into custom_config.binpb
aquarium config custom_config.textproto -o custom_config.binpb

# 3. Install bundles and instantiate the service on a cluster
inctl asset install \
  build/aquarium_images/<service_tag>.bundle.tar \
  build/aquarium_images/<skill_name>.bundle.tar \
  --address=localhost:17080

inctl service add ai.intrinsic.<service_name> \
  --name=<service_name> \
  --config=custom_config.binpb \
  --address=localhost:17080
```

See [`examples/robotiq/README.md`](../examples/robotiq/README.md) for a
complete working example.

---

## Workflow 2: Generating ROS Interfaces from a Proto Service

For subsystems defined upstream by an Intrinsic gRPC `.proto` service (such as
`CameraService`), use `aquarium generate ros` to generate ROS 2 `.msg`/`.srv`
packages and a matching `aquarium.yaml` gateway config:

```bash
aquarium generate ros \
  --service CameraService \
  --proto_path src/sdk/intrinsic/perception/proto/v1/camera_service.proto \
  --output_path src/generated \
  --single_package intrinsic_perception_interfaces \
  --target_package my_camera_driver \
  --target_node camera_driver_node
```

Options:
* `--service`: Target Protobuf `service` name in `--proto_path`.
* `--proto_path`: Path to the `.proto` file defining the service.
* `--output_path`: Directory where generated ROS 2 interface package(s) and
  `aquarium.yaml` are written.
* `--single_package`: Optional ROS package name to consolidate all generated
  `.msg` and `.srv` definitions into one package.
* `--target_package` / `--target_node`: Optional ROS package and node names
  populated into the `target` block of the generated `aquarium.yaml`.

Implement the generated ROS 2 services in your node and launch it alongside
`aquarium_gateway`, which exposes the corresponding gRPC endpoints (`grpc_port`
parameter, default `50051`).

---

## Local Execution & Cluster Zenoh Routing

### Local Gateway Runner (`aquarium run`)

Inside a ROS 2 environment, start a local `rmw_zenohd` router, the `target:`
(or `targets:` with `--sim`) configured in `aquarium.yaml`, and
`aquarium_gateway` together with:

```bash
aquarium run aquarium.yaml
```

Or run `aquarium_gateway` directly as a ROS 2 node:

```bash
ros2 run aquarium aquarium_gateway \
  --ros-args -p config_file:=/path/to/aquarium.yaml
```

### Automatic Cluster Zenoh Routing

At startup, the generated service/skill entrypoints and `aquarium_gateway`
check whether they are running inside a Kubernetes cluster via:
1. `KUBERNETES_SERVICE_HOST` in the environment,
2. `/etc/intrinsic/runtime_config.pb` on disk, or
3. `cluster.local` in `/etc/resolv.conf`.

When running in a cluster, Zenoh connects to
`tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447`; otherwise it
defaults to `tcp/127.0.0.1:7447`. Set `ZENOH_CONFIG_OVERRIDE` explicitly to
override the router endpoint:

```bash
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7447"]'
```

### Generated Skill Query Timeout

Generated C++ skill containers wait up to `60` seconds by default for a Zenoh
reply from `aquarium_gateway`. For skills wrapping long-running ROS 2 actions or
services, set `ZENOH_QUERY_TIMEOUT_SECONDS` in the skill container environment
to extend the deadline (in whole seconds).
