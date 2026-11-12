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

"""Local process orchestration for aquarium assistant and aquarium run."""

import os
import signal
import subprocess
import sys
import time
import yaml

from .config import parse_targets_config
from .config import resolve_target_arguments
from .workspace import find_workspace_dir


def terminate_process_group(proc, label, sig=signal.SIGINT, timeout=5.0):
  """Gracefully signal a subprocess group, escalating to SIGKILL on timeout."""
  if proc is None or proc.poll() is not None:
    return
  try:
    os.killpg(os.getpgid(proc.pid), sig)
  except ProcessLookupError:
    return

  try:
    proc.wait(timeout=timeout)
  except subprocess.TimeoutExpired:
    print(f'LAUNCHER: Force-killing {label} process group...')
    try:
      os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
      pass
    proc.wait()


def build_target_ros2_command(config, workspace_dir=None, use_sim=False):
  """Construct the ros2 run/launch command list for an aquarium.yaml config."""
  if 'target' in config or 'targets' in config:
    real_target, sim_target = parse_targets_config(config)
    selected = sim_target if use_sim else real_target
    target_type = selected.get('type')
    package = selected.get('package')
    name = selected.get('name')
    resolved_args = resolve_target_arguments(selected, workspace_dir)
    launch_args = [
        arg for arg in resolved_args if ':=' in arg or arg.startswith('-')
    ]
    if target_type == 'launch':
      return ['ros2', 'launch', package, name] + launch_args
    if target_type == 'node':
      return ['ros2', 'run', package, name] + resolved_args
    print(f"Error: Unsupported target type '{target_type}'.")
    sys.exit(1)

  legacy_launch = config.get('launch_file')
  if isinstance(legacy_launch, str) and legacy_launch.strip():
    return ['ros2', 'launch'] + legacy_launch.split()

  print("Error: Config must define either 'target' or 'targets' section.")
  sys.exit(1)


def run_assistant(mode, package_name, target, remaining_args):
  """Launch a ROS 2 target in the background and run setup assistant TUI."""
  selection_file = None
  node_args = []
  idx = 0
  while idx < len(remaining_args):
    arg = remaining_args[idx]
    if arg in ('--selection', '-s'):
      if idx + 1 < len(remaining_args):
        selection_file = remaining_args[idx + 1]
        idx += 2
        continue
      print('Error: Missing argument for --selection')
      sys.exit(1)
    node_args.append(arg)
    idx += 1

  env = os.environ.copy()
  env['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'

  workspace_dir = find_workspace_dir() or os.getcwd()
  log_dir = os.path.join(workspace_dir, 'log', 'aquarium')
  os.makedirs(log_dir, exist_ok=True)
  log_file_path = os.path.join(log_dir, 'background_launch.log')

  print(f'LAUNCHER: Logging background server output to: {log_file_path}')
  # Keep file descriptor open until background subprocess terminates.
  log_file = open(log_file_path, 'w', encoding='utf-8')

  cmd_args = [package_name, target] + node_args
  cmd_str = ' '.join(cmd_args)
  if mode == 'launch':
    print(f'LAUNCHER: Launching background: ros2 launch {cmd_str}')
    sub_cmd = ['ros2', 'launch'] + cmd_args
  elif mode == 'run':
    print(f'LAUNCHER: Launching background: ros2 run {cmd_str}')
    sub_cmd = ['ros2', 'run'] + cmd_args
  else:
    log_file.close()
    print(f"LAUNCHER ERROR: Unknown mode '{mode}'")
    sys.exit(1)

  node_proc = subprocess.Popen(
      sub_cmd,
      env=env,
      stdout=log_file,
      stderr=subprocess.STDOUT,
      text=True,
      process_group=0,
  )

  time.sleep(1.5)
  if node_proc.poll() is not None:
    print(f'LAUNCHER ERROR: Background ros2 {mode} exited immediately.')
    log_file.close()
    with open(log_file_path, 'r', encoding='utf-8') as f:
      print(f.read())
    sys.exit(1)

  tui_proc = None
  try:
    if selection_file:
      print(
          'LAUNCHER: Executing aquarium_setup_assistant in headless mode with'
          f' selection: {selection_file}...'
      )
    else:
      print('LAUNCHER: Executing aquarium_setup_assistant TUI...')
    target_type = 'node' if mode == 'run' else 'launch'
    tui_cmd = [
        'ros2',
        'run',
        'aquarium',
        'aquarium_setup_assistant',
        '--ros-args',
        '-p',
        f'target_type:={target_type}',
        '-p',
        f'target_package:={package_name}',
        '-p',
        f'target_name:={target}',
    ]
    if selection_file:
      tui_cmd.extend(
          ['-p', f'selection_file:={os.path.abspath(selection_file)}']
      )
    if node_args:
      args_yaml = '[' + ', '.join(f'"{a}"' for a in node_args) + ']'
      tui_cmd.extend(['-p', f'target_arguments:={args_yaml}'])

    tui_proc = subprocess.Popen(tui_cmd, env=env)
    tui_proc.wait()
  except KeyboardInterrupt:
    if tui_proc and tui_proc.poll() is None:
      print('LAUNCHER: Terminating TUI...')
      tui_proc.send_signal(signal.SIGINT)
      try:
        tui_proc.wait(timeout=2.0)
      except subprocess.TimeoutExpired:
        print('LAUNCHER: Force-killing TUI...')
        tui_proc.kill()
        tui_proc.wait()
  finally:
    print('\nLAUNCHER: Terminating background servers...')
    terminate_process_group(node_proc, 'background target', signal.SIGINT)
    log_file.close()
    print('LAUNCHER: Shutdown completed successfully.')


def run_gateway(config_path, use_sim=False):
  """Launch rmw_zenohd, the configured ROS 2 target, and aquarium_gateway."""
  if not os.path.exists(config_path):
    print(f"Error: Configuration file '{config_path}' not found.")
    sys.exit(1)

  try:
    with open(config_path, 'r', encoding='utf-8') as f:
      config = yaml.safe_load(f)
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f'Error: Failed to parse YAML configuration: {e}')
    sys.exit(1)

  if not config or not isinstance(config, dict):
    print('Error: Config file is empty or invalid.')
    sys.exit(1)

  workspace_dir = find_workspace_dir() or os.getcwd()
  target_cmd = build_target_ros2_command(
      config, workspace_dir=workspace_dir, use_sim=use_sim
  )

  env = os.environ.copy()
  env['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'

  print('LAUNCHER: Launching rmw_zenohd background router...')
  router_proc = subprocess.Popen(
      ['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
      env=env,
      stdout=subprocess.DEVNULL,
      stderr=subprocess.DEVNULL,
      process_group=0,
  )
  time.sleep(1.0)

  log_dir = os.path.join(workspace_dir, 'log', 'aquarium')
  os.makedirs(log_dir, exist_ok=True)
  log_file_path = os.path.join(log_dir, 'background_launch.log')

  print(f'LAUNCHER: Logging background server output to: {log_file_path}')
  log_file = open(log_file_path, 'w', encoding='utf-8')

  print(f"LAUNCHER: Launching background: {' '.join(target_cmd)}")
  target_proc = subprocess.Popen(
      target_cmd,
      env=env,
      stdout=log_file,
      stderr=subprocess.STDOUT,
      text=True,
      process_group=0,
  )

  time.sleep(1.5)
  if target_proc.poll() is not None:
    print('LAUNCHER ERROR: Background target exited immediately.')
    log_file.close()
    with open(log_file_path, 'r', encoding='utf-8') as f:
      print(f.read())
    terminate_process_group(router_proc, 'rmw_zenohd', signal.SIGTERM)
    sys.exit(1)

  try:
    print('LAUNCHER: Starting dynamic reflection proxy (aquarium_gateway)...')
    config_abs = os.path.abspath(config_path)
    gateway_cmd = [
        'ros2',
        'run',
        'aquarium',
        'aquarium_gateway',
        '--ros-args',
        '-p',
        f'config_file:={config_abs}',
    ]
    subprocess.run(gateway_cmd, env=env, check=True)
  except KeyboardInterrupt:
    pass
  finally:
    print('\nLAUNCHER: Terminating background servers...')
    terminate_process_group(target_proc, 'background target', signal.SIGINT)
    terminate_process_group(router_proc, 'rmw_zenohd', signal.SIGTERM)
    log_file.close()
    print('LAUNCHER: Shutdown completed successfully.')
