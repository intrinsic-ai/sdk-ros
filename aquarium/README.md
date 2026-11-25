# aquarium

`aquarium` bridges ROS 2 Lyrical Luth (`rmw_zenoh_cpp`) and the Intrinsic
Platform by translating between ROS 2 Common Data Representation (CDR) messages
and Intrinsic Protocol Buffers (Protobuf), and packaging ROS 2 subsystems into
sideloadable Flowstate Service and Skill bundles.

> Note: `aquarium` targets **ROS 2 Lyrical Luth** (Ubuntu 26.04). Running
> `aquarium build` compiles container images via `buildah`, so the host machine
> only needs `buildah`, `python3-venv`, and `protobuf-compiler`; running
> `aquarium assistant` or `aquarium run` locally requires a ROS 2 Lyrical
> environment.

## Quickstart

Point `aquarium build` at an [`aquarium.yaml`](docs/aquarium_yaml.md) config to
compile a ROS 2 node or launch file—along with static Flowstate skills for its
selected topics, services, and actions—into sideload-ready `.bundle.tar`
archives:

```bash
aquarium build src/sdk-ros/aquarium/test/test_aquarium_add_two_ints.yaml
```

Standalone workspace setup from scratch:

```bash
sudo apt install buildah python3-venv protobuf-compiler
mkdir -p ~/aquarium_ws/src
cd ~/aquarium_ws/src
git clone https://github.com/intrinsic-ai/sdk-ros
cd ~/aquarium_ws
python3 -m venv aquarium_venv
source aquarium_venv/bin/activate
pip install -e src/sdk-ros/aquarium
aquarium build src/sdk-ros/aquarium/test/test_aquarium_add_two_ints.yaml
```

This produces two bundles in `build/aquarium_images/` ready for sideloading with
`inctl asset install`:
* `build/aquarium_images/demo-nodes-cpp-add-two-ints-server.bundle.tar`
* `build/aquarium_images/add_two_ints_skill.bundle.tar`

## Workflows

* **Wrap an existing ROS 2 subsystem (ROS-defined):** Run
  `aquarium assistant launch` or `aquarium assistant run` to inspect a live
  ROS 2 graph in the terminal UI, select which topics, services, and actions to
  expose, and write `aquarium.yaml`. Then run `aquarium build aquarium.yaml` to
  build the Flowstate Service bundle (containing your ROS workspace and
  `aquarium_gateway`) and statically-typed Flowstate Skill bundles.
* **Implement an Intrinsic proto service in ROS 2 (proto-defined):** Run
  `aquarium generate ros` against an Intrinsic `.proto` service definition
  (such as `CameraService`) to generate ROS 2 `.msg`/`.srv` packages and a
  corresponding `aquarium.yaml` gateway configuration.

## Package Layout

* [**`aquarium/`**](aquarium): Python `aquarium` CLI orchestrator, packaged
  container templates, and ROS interface generator.
* [**`src/`**](src) & [**`include/`**](include): C++ `aquarium_gateway` dynamic
  translation server and C++ `aquarium_setup_assistant` terminal UI.
* [**`examples/`**](examples): Example configurations and launch files (see
  [`examples/robotiq/README.md`](examples/robotiq/README.md)).
* [**`test/`**](test): Unit and integration test suite.
* [**`docs/`**](docs): Usage, configuration, and architecture documentation.

## Documentation

* [**CLI Usage Guide**](docs/usage.md): End-to-end commands for introspecting,
  building, configuring launch arguments, sideloading bundles, and generating
  ROS interfaces.
* [**`aquarium.yaml` Reference**](docs/aquarium_yaml.md): Schema reference and
  examples for targets, custom types, services, topics, and actions.
* [**TUI Setup Assistant**](docs/setup_assistant.md): Interactive terminal
  wizard keybindings, headless selection mode, and graph introspection.
* [**Architecture & Internals**](docs/architecture.md): Background details on
  ROS 2 Type Description Service introspection, dynamic Protobuf reflection,
  and C-struct marshalling.
