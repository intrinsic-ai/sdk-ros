# HOWTO build

```bash
sudo apt install buildah python3-venv protobuf-compiler
mkdir -p ~/robotiq_ws/src
cd ~/robotiq_ws/src
git clone https://github.com/intrinsic-ai/sdk-ros
git clone https://github.com/robotiq/robotiq-ros
cd ~/robotiq_ws
python3 -m venv aquarium_venv
source aquarium_venv/bin/activate
pip install -e src/sdk-ros/aquarium
aquarium build src/sdk-ros/aquarium/examples/robotiq/robotiq_aquarium.yaml
```

For real hardware (so the service container can access `/dev/ttyUSB0`), pass
`--privileged`:

```bash
aquarium build --privileged \
  src/sdk-ros/aquarium/examples/robotiq/robotiq_aquarium.yaml
```

# HOWTO sideload with a custom config

To sideload the generated bundles onto a running `intrinsic-core` cluster with a
custom configuration (for example, `use_fake_hardware: true`, or overriding
`com_port` and `slave_address` on real hardware), encode a `.textproto` override
into a binary `google.protobuf.Any` (`.binpb`) file using `aquarium config`,
then install the bundles and add the service instance with `inctl`:

```bash
cat << 'EOF' > build/aquarium_images/fake_hw_config.textproto
[type.googleapis.com/ai.intrinsic.AquariumRobotiqLaunchXmlConfig] {
  use_fake_hardware: true
  gripper_model: "hand_e"
  ros_domain_id: 42
}
EOF

aquarium config \
  build/aquarium_images/fake_hw_config.textproto \
  -o build/aquarium_images/fake_hw_config.binpb

inctl asset install \
  build/aquarium_images/aquarium-robotiq.launch.xml.bundle.tar \
  build/aquarium_images/gripper_cmd_skill.bundle.tar \
  --address=localhost:17080

inctl service add ai.intrinsic.aquarium_robotiq_launch_xml \
  --name=aquarium_robotiq_launch_xml \
  --config=build/aquarium_images/fake_hw_config.binpb \
  --address=localhost:17080
```

For a real Hand-E gripper on `/dev/ttyUSB0` configured with Modbus RTU slave
address `2` (if `aquarium_robotiq_launch_xml` is already running from the fake
hardware example above, remove it first with
`inctl service remove aquarium_robotiq_launch_xml --address=localhost:17080`):

```bash
cat << 'EOF' > build/aquarium_images/real_hw_config.textproto
[type.googleapis.com/ai.intrinsic.AquariumRobotiqLaunchXmlConfig] {
  use_fake_hardware: false
  gripper_model: "hand_e"
  com_port: "/dev/ttyUSB0"
  slave_address: 2
  ros_domain_id: 42
}
EOF

aquarium config \
  build/aquarium_images/real_hw_config.textproto \
  -o build/aquarium_images/real_hw_config.binpb

inctl service add ai.intrinsic.aquarium_robotiq_launch_xml \
  --name=aquarium_robotiq_launch_xml \
  --config=build/aquarium_images/real_hw_config.binpb \
  --address=localhost:17080
```

# HOWTO close and open the gripper with SBL

Ensure a healthy solution is active on your `intrinsic-core` cluster before
connecting with the Solution Building Library (SBL). If needed, start the
minimal placeholder solution from an `intrinsic-core` checkout first, then
sideload the bundles and add the service as shown above:

```bash
bazel run //intrinsic/config:empty_application -- \
  --address=localhost:17080 \
  --operation_mode=real
```

Create a separate virtual environment (kept separate from `aquarium_venv` so
SBL's dependencies stay isolated) and install the `solutions` wheel:

```bash
python3 -m venv sbl_venv
source sbl_venv/bin/activate
pip install \
  https://github.com/intrinsic-ai/sdk/releases/latest/download/solutions-0.0.1-py3-none-any.whl
```

If `pip` pulls `protobuf` `7.x` (for example on Ubuntu 26.04) and generated
`_pb2` modules report a runtime version mismatch, pin `protobuf` below `7`:

```bash
pip install "protobuf<7"
```

Save the following script as `openclose.py` to close the gripper, pause for 3
seconds, and re-open it in a single executive run:

```python
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments

solution = deployments.connect(address="localhost:17080")
print("is_simulated:", solution.is_simulated)

gripper_cmd = solution.skills.ai.intrinsic.gripper_cmd_skill

# Hand-E travel: 0.025 m = closed, 0.0 m = open
close_gripper = gripper_cmd(
    command=gripper_cmd.ai.intrinsic.JointState(
        name=["hande_left_finger_joint"],
        position=[0.025],
        effort=[40.0],
    )
)
# Pause on the cluster via code-execution (or use
# solution.skills.ai.intrinsic.sleep_for(timeout=3.0) if installed).
sleep_3s = bt.PythonScript(function_body="import time\ntime.sleep(3.0)")
open_gripper = gripper_cmd(
    command=gripper_cmd.ai.intrinsic.JointState(
        name=["hande_left_finger_joint"],
        position=[0.0],
        effort=[40.0],
    )
)

solution.executive.run([close_gripper, sleep_3s, open_gripper])
blackboard = solution.executive.operation.blackboard
print("close result:", blackboard.get_value(close_gripper.result))
print("open result:", blackboard.get_value(open_gripper.result))
```
