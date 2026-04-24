import os

file_path = "intrinsic/icon/control/algorithms/joint_position_pid_velocity_controller.cc"

if os.path.exists(file_path):
    with open(file_path, "r") as f:
        content = f.read()

    old_code = """  eigenmath::VectorNd is_not_saturated =
      state_.previous_velocity_command.cwiseAbs()
          .cwiseLess(params_.max_velocity_command)
          .cast<double>();"""

    new_code = """  eigenmath::VectorNd is_not_saturated =
      (state_.previous_velocity_command.cwiseAbs().array() < params_.max_velocity_command.array())
          .cast<double>()
          .matrix();"""

    if old_code in content:
        content = content.replace(old_code, new_code)
        with open(file_path, "w") as f:
            f.write(content)
        print("Successfully fixed cwiseLess in joint_position_pid_velocity_controller.cc")
    else:
        print("Old code not found in joint_position_pid_velocity_controller.cc")
else:
    print(f"File not found: {file_path}")
