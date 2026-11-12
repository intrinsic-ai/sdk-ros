# TUI Setup Assistant

`aquarium_setup_assistant` is a zero-dependency POSIX terminal UI (using raw
`termios` and ANSI escape codes, suitable for SSH/`tmux` sessions) that scans a
live ROS 2 (`rmw_zenoh_cpp`) graph, queries interface schemas via the ROS 2
Type Description Service (TDS), and writes selected endpoints to
[`aquarium.yaml`](aquarium_yaml.md).

The assistant only performs graph discovery and config generation; container
compilation and Flowstate bundling happen separately in `aquarium build`.

## Running the Assistant

### Wrapped Launch or Node Mode (Recommended)

Use `aquarium assistant` to start your ROS 2 launch file or node in the
background (logging output to `log/aquarium/background_launch.log`), open the
TUI in the foreground, and automatically populate the `target:` block in
`aquarium.yaml`:

```bash
# Launch file target
aquarium assistant launch <PACKAGE_NAME> <LAUNCH_FILE> [launch_args...]

# Single node target
aquarium assistant run <PACKAGE_NAME> <EXECUTABLE_NAME> [node_args...]
```

When you quit the TUI, `aquarium` shuts down the background ROS process group
automatically.

### Headless Selection Mode

For scripted or automated config generation against a live node/launch target,
pass `--selection` (or `-s`) with a YAML file specifying which endpoints to
introspect and export without opening the interactive terminal UI:

```yaml
# selection.yaml (specify one or more of the following lists)
exposed_services:
  - /add_two_ints
exposed_topics:
  - /chatter
exposed_actions:
  - /robotiq_gripper_controller/gripper_cmd
```

```bash
aquarium assistant run demo_nodes_cpp add_two_ints_server -s selection.yaml
```

### Standalone Node Mode

To attach the TUI to an already-running `rmw_zenoh_cpp` ROS 2 graph:

```bash
# Terminal 1: ensure a Zenoh router is running
ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 2: launch the TUI directly
ros2 run aquarium aquarium_setup_assistant
```

## Interactive Screen & Keybindings

The assistant polls the active ROS 2 graph once per second and organizes
discovered endpoints into three tabs (`Topics`, `Services`, and `Actions`):

```text
 Aquarium Setup Assistant
 [T]opics      [S]ervices      [A]ctions
================================================================================
  [X] /add_two_ints                 example_interfaces/srv/AddTwoInts
  [ ] /lc_talker/change_state       lifecycle_msgs/srv/ChangeState
  [ ] /lc_talker/get_state          lifecycle_msgs/srv/GetState

--------------------------------------------------------------------------------
 STATUS: [Selected: 1 / Discovered: 3 (S:3, T:0, A:0)] Ready.
================================================================================
 COMMAND KEYS:
  [Arrow Up/Dn or j/k] Navigate list  [Space] Toggle Select
  [Enter] Show Introspection Detail    [W] Write Config File
  [T] Topics   [S] Services   [A] Actions     [Q] Quit Setup Wizard
================================================================================
```

| Key | Action |
| :--- | :--- |
| `Up`/`Down` or `k`/`j` | Move cursor up or down the active list |
| `Space` | Toggle selection (`[X]`) and run TDS introspection |
| `T` / `S` / `A` | Switch between **Topics**, **Services**, **Actions** |
| `P` | Toggle visibility of ROS 2 system/parameter services |
| `Enter` | Open **Interface Details** (`RIHS01` hash and fields) |
| `Escape` | Return from **Interface Details** to the catalog list |
| `W` | Write `aquarium.yaml` in the current working directory |
| `Q` | Restore terminal state and exit |
