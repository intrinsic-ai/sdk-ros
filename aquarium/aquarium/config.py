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

"""Aquarium YAML target configuration parsing and service config encoding."""

import ast
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

from .workspace import find_workspace_dir
from .workspace import validate_target_package


def sanitize_skill_stem(external_name):
  """Convert an interface external_name into a valid snake_case skill stem."""
  if not isinstance(external_name, str) or not external_name.strip():
    print("BUILDER ERROR: Interface is missing a non-empty 'external_name'.")
    sys.exit(1)
  stem = re.sub(r'[^a-zA-Z0-9]+', '_', external_name.strip()).strip('_')
  stem = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', stem).lower()
  stem = re.sub(r'_+', '_', stem)
  if not stem:
    print(
        f"BUILDER ERROR: Interface external_name '{external_name}' contains"
        ' no alphanumeric characters.'
    )
    sys.exit(1)
  if stem[0].isdigit():
    stem = f's_{stem}'
  return stem


def infer_proto_type(val_str):
  """Infer a scalar Protobuf field type from a launch argument value string."""
  if val_str.lower() in ('true', 'false'):
    return 'bool'
  try:
    int(val_str)
    return 'int32'
  except ValueError:
    pass
  try:
    float(val_str)
    return 'double'
  except ValueError:
    pass
  return 'string'


def validate_target_dependencies(target, label):
  """Validate optional target.dependencies package list in aquarium.yaml."""
  deps = target.get('dependencies')
  if deps is None:
    return
  if not isinstance(deps, list) or not all(
      isinstance(d, str) and bool(d.strip()) for d in deps
  ):
    print(
        f"Error: '{label}' section 'dependencies' must be a list of non-empty"
        ' package name strings.'
    )
    sys.exit(1)


def get_target_packages(target_cfg):
  """Return deduplicated list of ROS packages required by target_cfg."""
  pkgs = []
  primary = target_cfg.get('package')
  if primary:
    pkgs.append(primary)
  for dep in target_cfg.get('dependencies') or []:
    dep_name = dep.strip()
    if dep_name and dep_name not in pkgs:
      pkgs.append(dep_name)
  return pkgs


def validate_target_packages(target_cfg, workspace_dir):
  """Validate target package and dependencies; return space-joined APT names."""
  apt_pkgs = []
  for pkg in get_target_packages(target_cfg):
    is_apt, apt_pkg = validate_target_package(pkg, workspace_dir)
    if is_apt and apt_pkg and apt_pkg not in apt_pkgs:
      apt_pkgs.append(apt_pkg)
  return ' '.join(apt_pkgs)


def parse_targets_config(config):
  """Validate and normalize target/targets configuration into (real, sim)."""
  has_target = 'target' in config
  has_targets = 'targets' in config
  if has_target and has_targets:
    print("Error: Config cannot specify both 'target' and 'targets'.")
    sys.exit(1)

  if has_target:
    target = config.get('target')
    if not isinstance(target, dict) or not all(
        [target.get('type'), target.get('package'), target.get('name')]
    ):
      print(
          "Error: 'target' section must contain 'type', 'package', and 'name'."
      )
      sys.exit(1)
    validate_target_dependencies(target, 'target')
    return target, target

  targets = config.get('targets')
  if not targets or not isinstance(targets, dict):
    print("Error: Config must define either 'target' or 'targets' section.")
    sys.exit(1)

  real_target = targets.get('real')
  sim_target = targets.get('sim')
  if not real_target or not sim_target:
    print("Error: 'targets' section must define both 'sim' and 'real' keys.")
    sys.exit(1)

  for mode_name, tgt in [('real', real_target), ('sim', sim_target)]:
    if not isinstance(tgt, dict) or not all(
        [tgt.get('type'), tgt.get('package'), tgt.get('name')]
    ):
      print(
          f"Error: Target '{mode_name}' section must contain 'type',"
          " 'package', and 'name'."
      )
      sys.exit(1)
    validate_target_dependencies(tgt, mode_name)

  return real_target, sim_target


def get_target_image_tags(real_target, sim_target):
  """Compute base, service, real, and sim image tags for targets."""
  real_package = real_target.get('package')
  real_name = real_target.get('name')
  sim_package = sim_target.get('package')
  sim_name = sim_target.get('name')

  base_image_tag = (
      f'{real_package}-{real_name}'.replace('_', '-').replace('/', '-')
  )
  service_name_clean = base_image_tag.replace('-', '_').replace('.', '_')

  if real_target == sim_target:
    return base_image_tag, service_name_clean, base_image_tag, base_image_tag

  real_base_tag = (
      f'{real_package}-{real_name}'.replace('_', '-').replace('/', '-')
  )
  sim_base_tag = f'{sim_package}-{sim_name}'.replace('_', '-').replace('/', '-')
  if real_base_tag != sim_base_tag:
    real_image_tag = real_base_tag
    sim_image_tag = sim_base_tag
  else:
    real_image_tag = f'{real_base_tag}-real'
    sim_image_tag = f'{sim_base_tag}-sim'

  return base_image_tag, service_name_clean, real_image_tag, sim_image_tag


def find_launch_file_info(package_name, launch_name, workspace_dir=None):
  """Locate a ROS 2 launch file and its relative path under package share."""
  default_rel = launch_name if '/' in launch_name else f'launch/{launch_name}'
  if not package_name or not launch_name:
    return None, default_rel

  if not workspace_dir:
    workspace_dir = find_workspace_dir() or os.getcwd()

  ros_distro = os.environ.get('ROS_DISTRO', 'lyrical')
  share_candidates = [
      os.path.join(
          workspace_dir, 'install', package_name, 'share', package_name
      ),
      os.path.join(workspace_dir, 'install', 'share', package_name),
      os.path.join('/opt/ros', ros_distro, 'share', package_name),
  ]

  for share_dir in share_candidates:
    if os.path.isdir(share_dir):
      p1 = os.path.join(share_dir, 'launch', launch_name)
      if os.path.isfile(p1):
        return p1, f'launch/{launch_name}'
      p2 = os.path.join(share_dir, launch_name)
      if os.path.isfile(p2):
        return p2, launch_name
      for root, _, files in os.walk(share_dir):
        if launch_name in files:
          full_p = os.path.join(root, launch_name)
          rel_p = os.path.relpath(full_p, share_dir).replace(os.sep, '/')
          return full_p, rel_p

  src_dir = os.path.join(workspace_dir, 'src')
  if os.path.isdir(src_dir):
    for root, _, files in os.walk(src_dir):
      if 'package.xml' in files:
        pkg_xml_path = os.path.join(root, 'package.xml')
        try:
          with open(pkg_xml_path, 'r', encoding='utf-8') as f:
            if f'<name>{package_name}</name>' not in f.read():
              continue
        except OSError:
          continue
        p1 = os.path.join(root, 'launch', launch_name)
        if os.path.isfile(p1):
          return p1, f'launch/{launch_name}'
        p2 = os.path.join(root, launch_name)
        if os.path.isfile(p2):
          return p2, launch_name
        for l_root, _, l_files in os.walk(root):
          if launch_name in l_files:
            full_p = os.path.join(l_root, launch_name)
            rel_p = os.path.relpath(full_p, root).replace(os.sep, '/')
            return full_p, rel_p

  return None, default_rel


def _format_ast_default_val(val):
  """Format a literal AST default value as a launch argument string."""
  if isinstance(val, bool):
    return str(val).lower()
  return str(val)


def _extract_python_launch_args(launch_path):
  """Extract DeclareLaunchArgument declarations from a .launch.py file."""
  discovered = []
  with open(launch_path, 'r', encoding='utf-8') as f:
    tree = ast.parse(f.read(), filename=launch_path)
  for node in ast.walk(tree):
    if not isinstance(node, ast.Call):
      continue
    func_name = ''
    if isinstance(node.func, ast.Name):
      func_name = node.func.id
    elif isinstance(node.func, ast.Attribute):
      func_name = node.func.attr
    if func_name != 'DeclareLaunchArgument':
      continue

    arg_name = None
    if (
        node.args
        and isinstance(node.args[0], ast.Constant)
        and isinstance(node.args[0].value, str)
    ):
      arg_name = node.args[0].value
    for kw in node.keywords:
      if (
          kw.arg == 'name'
          and isinstance(kw.value, ast.Constant)
          and isinstance(kw.value.value, str)
      ):
        arg_name = kw.value.value
    if not arg_name:
      continue

    has_default = False
    default_val = None
    if (
        len(node.args) >= 2
        and isinstance(node.args[1], ast.Constant)
        and node.args[1].value is not None
    ):
      has_default = True
      default_val = _format_ast_default_val(node.args[1].value)
    for kw in node.keywords:
      if kw.arg == 'default_value':
        if isinstance(kw.value, ast.Constant) and kw.value.value is not None:
          has_default = True
          default_val = _format_ast_default_val(kw.value.value)
        elif isinstance(kw.value, ast.Call):
          for sub_kw in kw.value.keywords:
            if sub_kw.arg == 'text' and isinstance(sub_kw.value, ast.Constant):
              has_default = True
              default_val = str(sub_kw.value.value)
    if has_default:
      discovered.append(f'{arg_name}:={default_val}')
    else:
      discovered.append(arg_name)
  return discovered


def parse_launch_file_arguments_from_path(launch_path):
  """Extract top-level launch arguments from an XML, Python, or YAML file."""
  discovered = []
  if not launch_path or not os.path.isfile(launch_path):
    return discovered

  if launch_path.endswith('.xml'):
    try:
      tree = ET.parse(launch_path)
      root = tree.getroot()
      if root.tag == 'launch':
        for elem in root.findall('arg'):
          arg_name = elem.get('name')
          if arg_name:
            if 'default' in elem.attrib:
              discovered.append(f"{arg_name}:={elem.get('default')}")
            else:
              discovered.append(arg_name)
    except (ET.ParseError, OSError):
      pass
  elif launch_path.endswith('.py'):
    try:
      discovered.extend(_extract_python_launch_args(launch_path))
    except (SyntaxError, OSError, ValueError):
      pass
  elif launch_path.endswith(('.yaml', '.yml')):
    try:
      with open(launch_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
      if isinstance(data, dict) and isinstance(data.get('launch'), list):
        for item in data['launch']:
          if isinstance(item, dict) and isinstance(item.get('arg'), dict):
            arg_info = item['arg']
            arg_name = arg_info.get('name')
            if arg_name:
              if 'default' in arg_info and arg_info['default'] is not None:
                discovered.append(f"{arg_name}:={arg_info['default']}")
              else:
                discovered.append(str(arg_name))
    except (yaml.YAMLError, OSError):
      pass

  return discovered


def resolve_target_arguments(target, workspace_dir=None):
  """Merge target arguments with discovered top-level launch arguments."""
  explicit = list(target.get('arguments', []) or [])
  if target.get('type') != 'launch':
    return explicit

  package = target.get('package')
  name = target.get('name')
  launch_path, _ = find_launch_file_info(package, name, workspace_dir)
  discovered = parse_launch_file_arguments_from_path(launch_path)

  order = []
  values = {}
  for item in discovered:
    if ':=' in item:
      k, v = item.split(':=', 1)
    else:
      k, v = item.strip(), None
    if not k:
      continue
    if k not in values:
      order.append(k)
    values[k] = v

  for item in explicit:
    if ':=' in item:
      k, v = item.split(':=', 1)
    else:
      k, v = item.strip(), None
    if not k:
      continue
    if k not in values:
      order.append(k)
    if v is not None or k not in values:
      values[k] = v

  return [
      f'{k}:={values[k]}' if values[k] is not None else k for k in order
  ]


def run_config(
    input_textproto, output_binpb=None, proto_path=None, workspace_dir=None
):
  """Encode a service config textproto into a binary Any (.binpb) file."""
  if not os.path.isfile(input_textproto):
    print(f"Error: Input textproto file '{input_textproto}' not found.")
    sys.exit(1)

  if not workspace_dir:
    workspace_dir = find_workspace_dir() or os.getcwd()

  if not output_binpb:
    if input_textproto.endswith('.textproto'):
      output_binpb = input_textproto[: -len('.textproto')] + '.binpb'
    elif input_textproto.endswith('.txtpb'):
      output_binpb = input_textproto[: -len('.txtpb')] + '.binpb'
    else:
      output_binpb = f'{input_textproto}.binpb'

  with open(input_textproto, 'r', encoding='utf-8') as f:
    textproto_content = f.read()

  candidate_protos = []
  if proto_path:
    if not os.path.isfile(proto_path):
      print(f"Error: Proto file '{proto_path}' not found.")
      sys.exit(1)
    candidate_protos.append(os.path.abspath(proto_path))
  else:
    search_dirs = []
    input_dir = os.path.dirname(os.path.abspath(input_textproto))
    if input_dir and os.path.isdir(input_dir):
      search_dirs.append(input_dir)
    images_dir = os.path.join(workspace_dir, 'build', 'aquarium_images')
    if os.path.isdir(images_dir) and images_dir not in search_dirs:
      search_dirs.append(images_dir)
    for directory in search_dirs:
      for fname in sorted(os.listdir(directory)):
        if fname.endswith('_config.proto'):
          full_p = os.path.join(directory, fname)
          if full_p not in candidate_protos:
            candidate_protos.append(full_p)

  if not candidate_protos:
    print(
        'Error: No *_config.proto files found. Run "aquarium generate'
        ' intrinsic" or "aquarium build" first, or pass --proto <PROTO_FILE>.'
    )
    sys.exit(1)

  proto_defs = []
  for p_file in candidate_protos:
    try:
      with open(p_file, 'r', encoding='utf-8') as f:
        p_txt = f.read()
      pkg_m = re.search(r'^\s*package\s+([a-zA-Z0-9_.]+)\s*;', p_txt, re.M)
      msg_m = re.search(r'^\s*message\s+([a-zA-Z0-9_]+)\s*\{', p_txt, re.M)
      if msg_m:
        pkg_name = pkg_m.group(1) if pkg_m else 'ai.intrinsic'
        msg_name = msg_m.group(1)
        proto_defs.append((f'{pkg_name}.{msg_name}', p_file))
    except OSError:
      pass

  any_m = re.search(
      r'\[\s*type\.googleapis\.com/([a-zA-Z0-9_.]+)\s*\]', textproto_content
  )
  target_full_msg = None
  selected_protos = list(candidate_protos)

  if any_m:
    target_full_msg = any_m.group(1)
    matched = [p for full_m, p in proto_defs if full_m == target_full_msg]
    if matched:
      selected_protos = matched
  else:
    if len(proto_defs) == 1:
      target_full_msg, only_proto = proto_defs[0]
      selected_protos = [only_proto]
      textproto_content = (
          f'[type.googleapis.com/{target_full_msg}] {{\n'
          f'{textproto_content.strip()}\n}}\n'
      )
    else:
      print(
          'Error: Textproto does not specify'
          ' [type.googleapis.com/<MsgFullName>] and multiple *_config.proto'
          ' files exist. Specify --proto <PROTO_FILE>.'
      )
      sys.exit(1)

  if not shutil.which('protoc'):
    print(
        "Error: 'protoc' not found on PATH. Install it with:"
        ' sudo apt install protobuf-compiler'
    )
    sys.exit(1)

  protoc_cmd = ['protoc']
  if os.path.exists('/usr/include/google/protobuf/any.proto'):
    protoc_cmd.extend(['-I', '/usr/include'])
  proto_dirs = []
  for proto_file in selected_protos:
    proto_dir = os.path.dirname(os.path.abspath(proto_file))
    if proto_dir not in proto_dirs:
      proto_dirs.append(proto_dir)
      protoc_cmd.extend(['-I', proto_dir])
  protoc_cmd.extend(
      ['--encode=google.protobuf.Any', 'google/protobuf/any.proto']
      + selected_protos
  )

  res = subprocess.run(
      protoc_cmd,
      input=textproto_content.encode('utf-8'),
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      check=False,
  )
  if res.returncode != 0:
    err_msg = res.stderr.decode('utf-8', errors='replace').strip()
    print(f'Error: protoc failed to encode textproto:\n{err_msg}')
    sys.exit(1)

  out_dir = os.path.dirname(os.path.abspath(output_binpb))
  if out_dir:
    os.makedirs(out_dir, exist_ok=True)
  with open(output_binpb, 'wb') as f:
    f.write(res.stdout)

  print(
      f'CONFIG: Wrote binary Any config ({len(res.stdout)} bytes) to'
      f' {output_binpb}'
  )

  service_id = None
  service_name = None
  images_dir = os.path.join(workspace_dir, 'build', 'aquarium_images')
  if target_full_msg and os.path.isdir(images_dir):
    for fname in sorted(os.listdir(images_dir)):
      if fname.endswith('.manifest.textproto'):
        m_path = os.path.join(images_dir, fname)
        try:
          with open(m_path, 'r', encoding='utf-8') as f:
            m_txt = f.read()
          if f'config_message_full_name: "{target_full_msg}"' in m_txt:
            pkg_m = re.search(r'package:\s*"([^"]+)"', m_txt)
            name_m = re.search(r'name:\s*"([^"]+)"', m_txt)
            if pkg_m and name_m:
              service_name = name_m.group(1)
              service_id = f'{pkg_m.group(1)}.{service_name}'
              break
        except OSError:
          pass

  if service_id and service_name:
    print('\nTo instantiate the service with this config, run:')
    print(f'  inctl service add {service_id} \\')
    print(f'    --name={service_name} \\')
    print(f'    --config={output_binpb} \\')
    print('    --address=localhost:17080')
  return output_binpb
