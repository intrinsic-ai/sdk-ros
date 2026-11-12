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

"""Generate ROS 2 message/service packages and gateway YAML from Protobuf."""

import argparse
import os
import re
import sys
import yaml

PROTO_TO_ROS_PRIMITIVES = {
    'double': 'float64',
    'float': 'float32',
    'int32': 'int32',
    'int64': 'int64',
    'uint32': 'uint32',
    'uint64': 'uint64',
    'sint32': 'int32',
    'sint64': 'int64',
    'fixed32': 'uint32',
    'fixed64': 'uint64',
    'sfixed32': 'int32',
    'sfixed64': 'int64',
    'bool': 'bool',
    'string': 'string',
    'bytes': 'uint8[]',
}

PROTO_TO_YAML_PRIMITIVES = {
    'double': 'double',
    'float': 'float',
    'int32': 'int32',
    'int64': 'int64',
    'uint32': 'uint32',
    'uint64': 'uint64',
    'sint32': 'int32',
    'sint64': 'int64',
    'fixed32': 'uint32',
    'fixed64': 'uint64',
    'sfixed32': 'int32',
    'sfixed64': 'int64',
    'bool': 'bool',
    'string': 'string',
    'bytes': 'uint8[]',
}

ROS_TO_YAML_PRIMITIVES = {
    'int8': 'int8',
    'uint8': 'uint8',
    'int16': 'int16',
    'uint16': 'uint16',
    'int32': 'int32',
    'uint32': 'uint32',
    'int64': 'int64',
    'uint64': 'uint64',
    'float32': 'float',
    'float64': 'double',
    'bool': 'bool',
    'string': 'string',
}

SPECIAL_TYPE_MAPPINGS = {
    'google.protobuf.Duration': 'builtin_interfaces/Duration',
    'google.protobuf.Timestamp': 'builtin_interfaces/Time',
    'google.protobuf.Empty': 'std_msgs/Empty',
}

RESOLVED_TYPE_MAPPINGS = {
    'intrinsic_interfaces/Pose': 'geometry_msgs/Pose',
    'intrinsic_interfaces/Point': 'geometry_msgs/Point',
    'intrinsic_interfaces/Quaternion': 'geometry_msgs/Quaternion',
    'google_protobuf/Duration': 'builtin_interfaces/Duration',
    'google_protobuf/Timestamp': 'builtin_interfaces/Time',
    'google_protobuf/Empty': 'std_msgs/Empty',
}


class _RosGenContext:
  """Per-invocation state for Proto-to-ROS interface generation."""

  def __init__(self, single_package=None):
    self.package_dependencies = {}
    self.generated_files = {}
    self.known_proto_packages = set()
    self.single_package_name = single_package
    self.generated_types = set()

  def register_dependency(self, pkg, dep):
    if pkg == dep:
      return
    self.package_dependencies.setdefault(pkg, set()).add(dep)

  def register_generated_file(self, pkg, file_type, name):
    entry = self.generated_files.setdefault(pkg, {'srv': set(), 'msg': set()})
    entry[file_type].add(name)


def get_short_name(type_name):
  """Return the short name of a proto type (after the last dot)."""
  return type_name.split('.')[-1]


def get_map_message_name(val_type):
  """Return the ROS message name for a map value type."""
  short_name = get_short_name(val_type)
  return short_name[0].upper() + short_name[1:] + 'Map'


def proto_package_to_ros_package(proto_package):
  """Convert a proto package name to a ROS 2 package name."""
  parts = proto_package.split('.')
  filtered_parts = [p for p in parts if not re.match(r'^v\d+$', p)]
  raw_name = '_'.join(filtered_parts)

  if raw_name.startswith('intrinsic_proto_'):
    return raw_name.replace('intrinsic_proto_', 'intrinsic_', 1)
  if raw_name == 'intrinsic_proto':
    return 'intrinsic_interfaces'
  return raw_name


def is_primitive_or_special(proto_type):
  """Return True if proto_type is a scalar primitive or well-known type."""
  return (
      proto_type in PROTO_TO_ROS_PRIMITIVES
      or proto_type in SPECIAL_TYPE_MAPPINGS
  )


def resolve_relative_type(
    ctx,
    proto_type,
    current_proto_package,
    all_messages,
    parent_message_name=None,
):
  """Resolve a relative or absolute proto type name to (ros_pkg, msg_name)."""
  if proto_type in PROTO_TO_ROS_PRIMITIVES:
    return '', proto_type

  candidates = []
  if parent_message_name:
    candidates.append(f'{parent_message_name}.{proto_type}')
  candidates.append(proto_type)

  abs_types = []
  for cand in candidates:
    if cand.startswith('.'):
      abs_types.append(cand[1:])
    else:
      parts = cand.split('.')
      curr_pkg_parts = current_proto_package.split('.')
      for idx in range(len(curr_pkg_parts) + 1):
        prefix = curr_pkg_parts if idx == 0 else curr_pkg_parts[:-idx]
        abs_types.append('.'.join(prefix + parts))
      abs_types.append(cand)

  for abs_type in abs_types:
    parts = abs_type.split('.')
    for idx in range(len(parts), 0, -1):
      candidate_proto_pkg = '.'.join(parts[:idx])
      if candidate_proto_pkg in ctx.known_proto_packages:
        msg_name = ''.join(parts[idx:])
        ros_pkg = proto_package_to_ros_package(candidate_proto_pkg)
        if ros_pkg in all_messages and msg_name in all_messages[ros_pkg]:
          return ros_pkg, msg_name

  parts = proto_type.split('.')
  msg_name = parts[-1]
  type_pkg_parts = parts[:-1]
  if type_pkg_parts:
    proto_pkg = '.'.join(type_pkg_parts)
    return proto_package_to_ros_package(proto_pkg), msg_name
  return proto_package_to_ros_package(current_proto_package), msg_name


def resolve_ros_type(
    ctx,
    proto_type,
    current_ros_package,
    current_proto_package,
    all_messages,
    all_enums,
    parent_message_name=None,
):
  """Resolve a proto type to a ROS field type and track package dependencies."""
  if proto_type in PROTO_TO_ROS_PRIMITIVES:
    return PROTO_TO_ROS_PRIMITIVES[proto_type]

  if proto_type in SPECIAL_TYPE_MAPPINGS:
    resolved = SPECIAL_TYPE_MAPPINGS[proto_type]
    dep_pkg = resolved.split('/')[0]
    ctx.register_dependency(current_ros_package, dep_pkg)
    return resolved

  pkg_name, msg_name = resolve_relative_type(
      ctx,
      proto_type,
      current_proto_package,
      all_messages,
      parent_message_name,
  )

  if ctx.single_package_name and (pkg_name, msg_name) in ctx.generated_types:
    pkg_name = ctx.single_package_name

  resolved_type = f'{pkg_name}/{msg_name}'
  if resolved_type in RESOLVED_TYPE_MAPPINGS:
    mapped_type = RESOLVED_TYPE_MAPPINGS[resolved_type]
    dep_pkg = mapped_type.split('/')[0]
    ctx.register_dependency(current_ros_package, dep_pkg)
    return mapped_type

  if msg_name in all_enums.get(pkg_name, set()):
    return 'int32'

  if pkg_name == current_ros_package:
    return msg_name

  ctx.register_dependency(current_ros_package, pkg_name)
  return f'{pkg_name}/{msg_name}'


def to_ros_field_string(
    ctx,
    field,
    current_ros_package,
    current_proto_package,
    all_messages,
    all_enums,
    parent_message_name,
):
  """Convert a parsed proto field dictionary to a ROS .msg/.srv field line."""
  name = field['name']
  if field['is_map']:
    map_msg_name = get_map_message_name(field['val_type'])
    return f'{map_msg_name}[] {name}'

  ros_type = resolve_ros_type(
      ctx,
      field['type'],
      current_ros_package,
      current_proto_package,
      all_messages,
      all_enums,
      parent_message_name,
  )
  if field['is_repeated']:
    if not ros_type.endswith('[]'):
      ros_type = f'{ros_type}[]'
  elif field['is_optional']:
    ros_type = f'{ros_type}[<=1]'

  return f'{ros_type} {name}'


def strip_nested_messages_and_enums(body):
  """Remove nested message and enum blocks from a proto message body."""
  while True:
    match = re.search(r'\b(message|enum)\s+(\w+)\s*\{', body)
    if not match:
      break
    start_idx = match.start()
    brace_count = 1
    end_idx = match.end()
    while brace_count > 0 and end_idx < len(body):
      char = body[end_idx]
      if char == '{':
        brace_count += 1
      elif char == '}':
        brace_count -= 1
      end_idx += 1
    body = body[:start_idx] + body[end_idx:]
  return body


def parse_message_fields(message_body):
  """Parse scalar, message, repeated, optional, and map fields of a message."""
  fields = []
  body_normalized = re.sub(r'oneof\s+\w+\s*\{([^}]+)\}', r'\1', message_body)
  body_normalized = strip_nested_messages_and_enums(body_normalized)
  body_normalized = re.sub(r'//.*', '', body_normalized)
  body_normalized = re.sub(r'/\*.*?\*/', '', body_normalized, flags=re.DOTALL)

  map_re = re.compile(
      r'map\s*<\s*([a-zA-Z0-9_.]+)\s*,\s*([a-zA-Z0-9_.]+)\s*>\s+'
      r'(\w+)\s*=\s*\d+\s*;'
  )
  field_re = re.compile(
      r'(repeated\s+)?(optional\s+)?([a-zA-Z0-9_.]+)\s+(\w+)\s*=\s*\d+\s*;'
  )

  for line in body_normalized.split('\n'):
    line = line.strip()
    if not line or line.startswith('reserved') or line.startswith('extensions'):
      continue

    map_match = map_re.match(line)
    if map_match:
      key_type, val_type, name = map_match.groups()
      fields.append({
          'name': name,
          'type': f'map<{key_type},{val_type}>',
          'is_map': True,
          'key_type': key_type,
          'val_type': val_type,
          'is_repeated': False,
          'is_optional': False,
      })
      continue

    field_match = field_re.match(line)
    if field_match:
      repeated, optional, f_type, name = field_match.groups()
      fields.append({
          'name': name,
          'type': f_type,
          'is_repeated': bool(repeated),
          'is_optional': bool(optional),
          'is_map': False,
      })

  return fields


def extract_service_body(content, target_service):
  """Extract the body inside `service <target_service> { ... }`."""
  match = re.search(rf'service\s+{target_service}\s*\{{', content)
  if not match:
    raise ValueError(f'Service {target_service} not found')

  start_idx = match.end()
  brace_count = 1
  end_idx = start_idx

  while brace_count > 0 and end_idx < len(content):
    char = content[end_idx]
    if char == '{':
      brace_count += 1
    elif char == '}':
      brace_count -= 1
    end_idx += 1

  if brace_count > 0:
    raise ValueError(f'Unmatched braces in service {target_service}')

  return content[start_idx : end_idx - 1]


def parse_messages_with_nesting(content):
  """Parse messages from proto content, producing concatenated nested names."""
  content = re.sub(r'//.*', '', content)
  content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)

  messages = {}
  stack = []
  for token in re.finditer(r'\bmessage\s+(\w+)\s*\{|\{|\}', content):
    match_str = token.group(0)
    if match_str.startswith('message'):
      stack.append((token.group(1), token.end()))
    elif match_str == '{':
      if not stack or stack[-1][1] != token.start():
        stack.append((None, token.end()))
    elif match_str == '}' and stack:
      name, start_idx = stack.pop()
      if name:
        body = content[start_idx : token.start()]
        full_name = ''.join(n for n, _ in stack if n) + name
        messages[full_name] = body
  return messages


def parse_proto_imports(proto_path):
  """Extract direct import paths from a .proto file."""
  with open(proto_path, 'r', encoding='utf-8') as f:
    content = f.read()
  content = re.sub(r'//.*', '', content)
  content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
  return re.findall(r'import\s+(?:public\s+)?"([^"]+)";', content)


def parse_proto_file(proto_path, target_service=None):
  """Parse a .proto file into (package, rpcs, messages, imports, enums)."""
  with open(proto_path, 'r', encoding='utf-8') as f:
    content = f.read()

  package_match = re.search(r'package\s+([a-zA-Z0-9_.]+);', content)
  if not package_match:
    raise ValueError(f'No package definition found in {proto_path}')
  proto_package = package_match.group(1)

  rpcs = []
  if target_service:
    try:
      service_body = extract_service_body(content, target_service)
      rpc_pattern = (
          r'rpc\s+(\w+)\s*\(\s*([a-zA-Z0-9_.]+)\s*\)\s*'
          r'returns\s*\(\s*([a-zA-Z0-9_.]+)\s*\)'
      )
      rpcs = re.findall(rpc_pattern, service_body)
    except ValueError:
      pass

  raw_messages = parse_messages_with_nesting(content)
  messages = {
      msg_name: parse_message_fields(msg_body)
      for msg_name, msg_body in raw_messages.items()
  }
  imports = parse_proto_imports(proto_path)
  enums = [m.group(1) for m in re.finditer(r'\benum\s+(\w+)\s*\{', content)]

  return proto_package, rpcs, messages, imports, enums


def find_bazel_workspace_root(start_path):
  """Walk upward from start_path to locate MODULE.bazel or WORKSPACE."""
  curr = os.path.abspath(start_path)
  if os.path.isfile(curr):
    curr = os.path.dirname(curr)

  temp = curr
  while temp != '/':
    if os.path.exists(os.path.join(temp, 'MODULE.bazel')) or os.path.exists(
        os.path.join(temp, 'WORKSPACE')
    ):
      return temp
    temp = os.path.dirname(temp)

  parts = curr.split(os.sep)
  if 'intrinsic' in parts:
    idx = parts.index('intrinsic')
    fallback_path = os.sep.join(parts[:idx])
    return fallback_path if fallback_path else '/'

  return curr


def parse_import_graph(ctx, start_proto_path, workspace_root, target_service):
  """Recursively parse imported .proto files starting at start_proto_path."""
  ctx.known_proto_packages.clear()

  all_messages = {}
  all_enums = {}
  visited_files = set()
  queue = [start_proto_path]

  target_package = None
  target_rpcs = []

  while queue:
    proto_path = queue.pop(0)
    if proto_path in visited_files:
      continue
    visited_files.add(proto_path)

    try:
      is_start_file = proto_path == start_proto_path
      pkg, rpcs, messages, imports, enums = parse_proto_file(
          proto_path, target_service if is_start_file else None
      )
      if is_start_file:
        target_package = pkg
        target_rpcs = rpcs

      ctx.known_proto_packages.add(pkg)
      ros_pkg = proto_package_to_ros_package(pkg)

      pkg_msgs = all_messages.setdefault(ros_pkg, {})
      for msg_name, fields in messages.items():
        pkg_msgs[msg_name] = {'fields': fields, 'proto_package': pkg}

      all_enums.setdefault(ros_pkg, set()).update(enums)

      for imp in imports:
        resolved_path = os.path.join(workspace_root, imp)
        if os.path.exists(resolved_path):
          queue.append(resolved_path)
    except Exception as e:  # pylint: disable=broad-exception-caught
      print(f'Warning: Failed to parse {proto_path}: {e}')

  return target_package, target_rpcs, all_messages, all_enums


def collect_reachable_messages(
    ctx, rpc_msgs, all_messages, all_enums, target_ros_package
):
  """Collect transitive (ros_package, message_name) types used by RPCs."""
  reachable = set()
  queue = []

  def _enqueue_fields(msg_data, parent_name):
    if not msg_data:
      return
    curr_proto_pkg = msg_data['proto_package']
    for field in msg_data['fields']:
      t = field['val_type'] if field['is_map'] else field['type']
      if is_primitive_or_special(t):
        continue
      dep_pkg, dep_name = resolve_relative_type(
          ctx, t, curr_proto_pkg, all_messages, parent_name
      )
      if dep_name in all_enums.get(dep_pkg, set()):
        continue
      if f'{dep_pkg}/{dep_name}' in RESOLVED_TYPE_MAPPINGS:
        continue
      if (dep_pkg, dep_name) not in reachable:
        reachable.add((dep_pkg, dep_name))
        queue.append((dep_pkg, dep_name))

  for msg_name in rpc_msgs:
    _enqueue_fields(
        all_messages.get(target_ros_package, {}).get(msg_name), msg_name
    )

  while queue:
    pkg, name = queue.pop(0)
    _enqueue_fields(all_messages.get(pkg, {}).get(name), name)

  return reachable


def write_status_msg(ctx, output_path, pkg_name='intrinsic_interfaces'):
  """Write the common gRPC-compatible Status.msg definition."""
  msg_dir = os.path.join(output_path, pkg_name, 'msg')
  os.makedirs(msg_dir, exist_ok=True)

  status_file = os.path.join(msg_dir, 'Status.msg')
  status_content = (
      '# gRPC Status Codes\n'
      'int32 OK=0\n'
      'int32 CANCELLED=1\n'
      'int32 UNKNOWN=2\n'
      'int32 INVALID_ARGUMENT=3\n'
      'int32 DEADLINE_EXCEEDED=4\n'
      'int32 NOT_FOUND=5\n'
      'int32 ALREADY_EXISTS=6\n'
      'int32 PERMISSION_DENIED=7\n'
      'int32 RESOURCE_EXHAUSTED=8\n'
      'int32 FAILED_PRECONDITION=9\n'
      'int32 ABORTED=10\n'
      'int32 OUT_OF_RANGE=11\n'
      'int32 UNIMPLEMENTED=12\n'
      'int32 INTERNAL=13\n'
      'int32 UNAVAILABLE=14\n'
      'int32 DATA_LOSS=15\n'
      'int32 UNAUTHENTICATED=16\n\n'
      'int32 code\n'
      'string message\n'
  )
  with open(status_file, 'w', encoding='utf-8') as f:
    f.write(status_content)
  print(f'GENERATOR: Generated {status_file}')
  ctx.register_generated_file(pkg_name, 'msg', 'Status')


def generate_build_files(ctx, output_path):
  """Generate package.xml and CMakeLists.txt for all generated ROS packages."""
  for pkg, files in ctx.generated_files.items():
    pkg_dir = os.path.join(output_path, pkg)
    os.makedirs(pkg_dir, exist_ok=True)
    deps = ctx.package_dependencies.get(pkg, set())

    pkg_xml_path = os.path.join(pkg_dir, 'package.xml')
    dep_tags = ''.join(f'  <depend>{d}</depend>\n' for d in sorted(deps))
    pkg_xml_content = (
        '<?xml version="1.0"?>\n'
        '<?xml-model'
        ' href="http://download.ros.org/schema/package_format3.xsd"'
        ' schematypens="http://www.w3.org/2001/XMLSchema"?>\n'
        '<package format="3">\n'
        f'  <name>{pkg}</name>\n'
        '  <version>0.0.1</version>\n'
        f'  <description>Generated ROS 2 package for {pkg}</description>\n'
        '  <maintainer email="ros@intrinsic.ai">Intrinsic</maintainer>\n'
        '  <license>Apache-2.0</license>\n\n'
        '  <buildtool_depend>ament_cmake</buildtool_depend>\n'
        '  <buildtool_depend>rosidl_default_generators</buildtool_depend>\n'
        f'{dep_tags}'
        '  <exec_depend>rosidl_default_runtime</exec_depend>\n'
        '  <member_of_group>rosidl_interface_packages</member_of_group>\n\n'
        '  <export>\n'
        '    <build_type>ament_cmake</build_type>\n'
        '  </export>\n'
        '</package>\n'
    )
    with open(pkg_xml_path, 'w', encoding='utf-8') as f:
      f.write(pkg_xml_content)
    print(f'GENERATOR: Generated {pkg_xml_path}')

    cmake_path = os.path.join(pkg_dir, 'CMakeLists.txt')
    find_pkg_cmds = ''.join(
        f'find_package({d} REQUIRED)\n' for d in sorted(deps)
    )
    interfaces = [f'  "srv/{srv}.srv"' for srv in sorted(files['srv'])] + [
        f'  "msg/{msg}.msg"' for msg in sorted(files['msg'])
    ]
    interfaces_str = '\n'.join(interfaces)
    dep_arg = ''
    if deps:
      dep_arg = '  DEPENDENCIES\n' + ''.join(
          f'    {d}\n' for d in sorted(deps)
      )

    cmake_content = (
        'cmake_minimum_required(VERSION 3.16)\n'
        f'project({pkg})\n\n'
        'if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")'
        '\n'
        '  add_compile_options(-Wall -Wextra -Wpedantic)\n'
        'endif()\n\n'
        'find_package(ament_cmake REQUIRED)\n'
        'find_package(rosidl_default_generators REQUIRED)\n'
        f'{find_pkg_cmds}'
        'rosidl_generate_interfaces(${PROJECT_NAME}\n'
        f'{interfaces_str}\n'
        f'{dep_arg})\n\n'
        'ament_package()\n'
    )
    with open(cmake_path, 'w', encoding='utf-8') as f:
      f.write(cmake_content)
    print(f'GENERATOR: Generated {cmake_path}')


def resolve_aquarium_yaml_type(
    ctx,
    proto_type,
    current_proto_package,
    all_messages,
    all_enums,
    parent_message_name=None,
):
  """Resolve a proto type into aquarium.yaml field type notation."""
  if proto_type in PROTO_TO_YAML_PRIMITIVES:
    return PROTO_TO_YAML_PRIMITIVES[proto_type]

  if proto_type in SPECIAL_TYPE_MAPPINGS:
    resolved = SPECIAL_TYPE_MAPPINGS[proto_type]
    parts = resolved.split('/')
    if len(parts) == 2:
      return f'{parts[0]}/msg/{parts[1]}'
    return resolved

  pkg_name, msg_name = resolve_relative_type(
      ctx,
      proto_type,
      current_proto_package,
      all_messages,
      parent_message_name,
  )
  if ctx.single_package_name and (pkg_name, msg_name) in ctx.generated_types:
    pkg_name = ctx.single_package_name

  resolved_key = f'{pkg_name}/{msg_name}'
  if resolved_key in RESOLVED_TYPE_MAPPINGS:
    mapped_type = RESOLVED_TYPE_MAPPINGS[resolved_key]
    parts = mapped_type.split('/')
    if len(parts) == 2:
      return f'{parts[0]}/msg/{parts[1]}'
    return mapped_type

  if msg_name in all_enums.get(pkg_name, set()):
    return 'int32'

  return f'{pkg_name}/msg/{msg_name}'


def to_aquarium_yaml_field_string(
    ctx,
    field,
    current_proto_package,
    all_messages,
    all_enums,
    parent_message_name,
):
  """Format a parsed proto field dictionary as an aquarium.yaml field entry."""
  name = field['name']
  if field['is_map']:
    map_name = get_map_message_name(field['val_type'])
    target_pkg = (
        ctx.single_package_name
        if ctx.single_package_name
        else proto_package_to_ros_package(current_proto_package)
    )
    return f'{target_pkg}/msg/{map_name}[] {name}'

  yaml_type = resolve_aquarium_yaml_type(
      ctx,
      field['type'],
      current_proto_package,
      all_messages,
      all_enums,
      parent_message_name,
  )
  if field['is_repeated']:
    if not yaml_type.endswith('[]'):
      yaml_type = f'{yaml_type}[]'
  elif field['is_optional']:
    yaml_type = f'{yaml_type}[bounded]'

  return f'{yaml_type} {name}'


def collect_yaml_types(
    ctx, rpc_msgs, all_messages, all_enums, target_ros_package
):
  """Collect non-primitive message types referenced in aquarium.yaml."""
  reachable = set()
  queue = []

  def _enqueue_yaml_fields(msg_data, parent_name):
    if not msg_data:
      return
    curr_proto_pkg = msg_data['proto_package']
    for field in msg_data['fields']:
      t = field['val_type'] if field['is_map'] else field['type']
      if t in PROTO_TO_ROS_PRIMITIVES:
        continue
      dep_pkg, dep_name = resolve_relative_type(
          ctx, t, curr_proto_pkg, all_messages, parent_name
      )
      if dep_name in all_enums.get(dep_pkg, set()):
        continue
      if (dep_pkg, dep_name) not in reachable:
        reachable.add((dep_pkg, dep_name))
        queue.append((dep_pkg, dep_name))

  for msg_name in rpc_msgs:
    _enqueue_yaml_fields(
        all_messages.get(target_ros_package, {}).get(msg_name), msg_name
    )

  while queue:
    pkg, name = queue.pop(0)
    _enqueue_yaml_fields(all_messages.get(pkg, {}).get(name), name)

  return reachable


def resolve_ros_type_to_yaml_type(pkg, name, single_package=None):
  """Convert a (ros_pkg, msg_name) pair to `pkg/msg/Name` YAML notation."""
  if name in ROS_TO_YAML_PRIMITIVES:
    return ROS_TO_YAML_PRIMITIVES[name]

  key = f'{pkg}/{name}'
  if key in RESOLVED_TYPE_MAPPINGS:
    mapped_type = RESOLVED_TYPE_MAPPINGS[key]
    parts = mapped_type.split('/')
    if len(parts) == 2:
      return f'{parts[0]}/msg/{parts[1]}'
    return mapped_type

  target_pkg = single_package if single_package else pkg
  return f'{target_pkg}/msg/{name}'


def generate_gateway_config(
    ctx,
    output_path,
    service_name,
    proto_package,
    rpcs,
    all_messages,
    all_enums,
    maps_to_generate,
    target_package=None,
    target_node=None,
    single_package=None,
):
  """Write aquarium.yaml mapping gRPC service RPCs to ROS 2 services."""
  target_pkg = target_package if target_package else '<TARGET_PACKAGE>'
  target_nd = target_node if target_node else '<TARGET_NODE>'
  target_ros_package = proto_package_to_ros_package(proto_package)
  target_srv_package = single_package if single_package else target_ros_package

  yaml_data = {
      'target': {
          'type': 'node',
          'package': target_pkg,
          'name': target_nd,
      },
      'types': [],
      'interfaces': [],
  }

  rpc_msgs = set()
  for _, req_type, resp_type in rpcs:
    rpc_msgs.add(get_short_name(req_type))
    rpc_msgs.add(get_short_name(resp_type))

  yaml_types = collect_yaml_types(
      ctx, rpc_msgs, all_messages, all_enums, target_ros_package
  )
  yaml_types.add(('intrinsic_interfaces', 'Status'))

  for pkg, name in sorted(yaml_types):
    yaml_name = resolve_ros_type_to_yaml_type(pkg, name, single_package)
    msg_data = all_messages.get(pkg, {}).get(name)
    if not msg_data:
      if name == 'Status':
        fields = [
            {
                'name': 'code',
                'type': 'int32',
                'is_map': False,
                'is_repeated': False,
                'is_optional': False,
            },
            {
                'name': 'message',
                'type': 'string',
                'is_map': False,
                'is_repeated': False,
                'is_optional': False,
            },
        ]
        curr_proto_pkg = 'intrinsic_proto.data_logger'
      elif yaml_name == 'std_msgs/msg/Empty':
        fields = []
        curr_proto_pkg = 'google.protobuf'
      elif yaml_name in (
          'builtin_interfaces/msg/Duration',
          'builtin_interfaces/msg/Time',
      ):
        fields = [
            {
                'name': 'sec',
                'type': 'int32',
                'is_map': False,
                'is_repeated': False,
                'is_optional': False,
            },
            {
                'name': 'nanosec',
                'type': 'uint32',
                'is_map': False,
                'is_repeated': False,
                'is_optional': False,
            },
        ]
        curr_proto_pkg = 'google.protobuf'
      else:
        continue
    else:
      fields = msg_data['fields']
      curr_proto_pkg = msg_data['proto_package']

    yaml_type = {
        'name': yaml_name,
        'fields': [
            to_aquarium_yaml_field_string(
                ctx, field, curr_proto_pkg, all_messages, all_enums, name
            )
            for field in fields
        ],
    }
    yaml_data['types'].append(yaml_type)

  for (pkg, map_name), (key_ros, val_ros) in maps_to_generate.items():
    map_pkg = target_srv_package if single_package else pkg
    if '/' in key_ros:
      p, n = key_ros.split('/')
      resolved_key = resolve_ros_type_to_yaml_type(p, n, single_package)
    else:
      resolved_key = resolve_ros_type_to_yaml_type(
          map_pkg, key_ros, single_package
      )

    if '/' in val_ros:
      p, n = val_ros.split('/')
      resolved_val = resolve_ros_type_to_yaml_type(p, n, single_package)
    else:
      resolved_val = resolve_ros_type_to_yaml_type(
          map_pkg, val_ros, single_package
      )

    yaml_type = {
        'name': f'{map_pkg}/msg/{map_name}',
        'fields': [
            f'{resolved_key} key',
            f'{resolved_val} value',
        ],
    }
    if not any(t['name'] == yaml_type['name'] for t in yaml_data['types']):
      yaml_data['types'].append(yaml_type)

  for method_name, req_type, resp_type in rpcs:
    req_short = get_short_name(req_type)
    interface_yaml = {
        'internal_interface': 'ros_service',
        'internal_name': f'/{target_nd}/{method_name}',
        'internal_type': f'{target_srv_package}/srv/{method_name}',
        'external_interface': 'grpc',
        'external_name': f'/{proto_package}.{service_name}/{method_name}',
        'request_fields': [],
        'response_fields': [],
    }

    req_msg_data = all_messages.get(target_ros_package, {}).get(req_short)
    if req_msg_data:
      req_proto_pkg = req_msg_data['proto_package']
      for field in req_msg_data['fields']:
        interface_yaml['request_fields'].append(
            to_aquarium_yaml_field_string(
                ctx, field, req_proto_pkg, all_messages, all_enums, req_short
            )
        )
    elif req_type != 'google.protobuf.Empty':
      req_yaml_type = resolve_aquarium_yaml_type(
          ctx, req_type, proto_package, all_messages, all_enums
      )
      interface_yaml['request_fields'].append(f'{req_yaml_type} request')

    resp_short = get_short_name(resp_type)
    resp_msg_data = all_messages.get(target_ros_package, {}).get(resp_short)
    if resp_msg_data:
      resp_proto_pkg = resp_msg_data['proto_package']
      for field in resp_msg_data['fields']:
        interface_yaml['response_fields'].append(
            to_aquarium_yaml_field_string(
                ctx, field, resp_proto_pkg, all_messages, all_enums, resp_short
            )
        )
    elif resp_type != 'google.protobuf.Empty':
      resp_yaml_type = resolve_aquarium_yaml_type(
          ctx, resp_type, proto_package, all_messages, all_enums
      )
      interface_yaml['response_fields'].append(f'{resp_yaml_type} response')

    interface_yaml['response_fields'].append(
        f'{target_srv_package}/msg/Status status'
    )
    yaml_data['interfaces'].append(interface_yaml)

  output_yaml = os.path.join(output_path, 'aquarium.yaml')
  with open(output_yaml, 'w', encoding='utf-8') as f:
    yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
  print(f'GENERATOR: Generated gateway config at {output_yaml}')


def run_generate_ros(
    service_name,
    proto_path,
    output_path,
    single_package=None,
    target_package=None,
    target_node=None,
):
  """Generate ROS 2 interface packages and aquarium.yaml for a Proto service."""
  ctx = _RosGenContext(single_package=single_package)

  print(
      f"GENERATOR: Starting 'generate ros' for service '{service_name}' in"
      f' {proto_path}'
  )
  if single_package:
    print(
        'GENERATOR: Single package mode enabled. Target package:'
        f" '{single_package}'"
    )

  if not os.path.exists(proto_path):
    print(f'Error: Proto file not found at {proto_path}')
    sys.exit(1)

  try:
    workspace_root = find_bazel_workspace_root(proto_path)
    print(f'GENERATOR: Found Bazel workspace root: {workspace_root}')
  except ValueError as e:
    print(f'Error: {e}')
    sys.exit(1)

  print('GENERATOR: Parsing import graph...')
  proto_package, rpcs, all_messages, all_enums = parse_import_graph(
      ctx, proto_path, workspace_root, service_name
  )

  target_ros_package = proto_package_to_ros_package(proto_package)
  target_srv_package = (
      single_package if single_package else target_ros_package
  )
  print(f"GENERATOR: Target ROS package: '{target_srv_package}'")

  rpc_msgs = set()
  for _, req_type, resp_type in rpcs:
    rpc_msgs.add(get_short_name(req_type))
    rpc_msgs.add(get_short_name(resp_type))

  print('GENERATOR: Analyzing reachable types...')
  reachable_types = collect_reachable_messages(
      ctx, rpc_msgs, all_messages, all_enums, target_ros_package
  )

  undefined_types = []
  defined_types = []
  for pkg, name in sorted(reachable_types):
    if name in all_messages.get(pkg, {}):
      defined_types.append((pkg, name))
    else:
      undefined_types.append((pkg, name))

  print(f'GENERATOR: Found {len(defined_types)} defined types to generate.')
  if undefined_types:
    print(
        f'GENERATOR: WARNING: Found {len(undefined_types)} undefined types:'
    )
    for pkg, name in undefined_types:
      print(f'  - {pkg}/{name}')

  ctx.generated_types = set(defined_types)
  ctx.generated_types.add(('intrinsic_interfaces', 'Status'))

  status_package = single_package if single_package else 'intrinsic_interfaces'
  write_status_msg(ctx, output_path, status_package)

  srv_dir = os.path.join(output_path, target_srv_package, 'srv')
  os.makedirs(srv_dir, exist_ok=True)
  maps_to_generate = {}

  for method_name, req_type, resp_type in rpcs:
    req_short = get_short_name(req_type)
    indent_pkg = target_srv_package

    req_fields_str = ''
    req_msg_data = all_messages.get(target_ros_package, {}).get(req_short)
    if req_msg_data:
      req_proto_pkg = req_msg_data['proto_package']
      for field in req_msg_data['fields']:
        req_fields_str += (
            to_ros_field_string(
                ctx,
                field,
                indent_pkg,
                req_proto_pkg,
                all_messages,
                all_enums,
                req_short,
            )
            + '\n'
        )
        if field['is_map']:
          map_name = get_map_message_name(field['val_type'])
          key_ros = resolve_ros_type(
              ctx,
              field['key_type'],
              indent_pkg,
              req_proto_pkg,
              all_messages,
              all_enums,
              req_short,
          )
          val_ros = resolve_ros_type(
              ctx,
              field['val_type'],
              indent_pkg,
              req_proto_pkg,
              all_messages,
              all_enums,
              req_short,
          )
          maps_to_generate[(indent_pkg, map_name)] = (key_ros, val_ros)
    elif req_type != 'google.protobuf.Empty':
      req_ros_type = resolve_ros_type(
          ctx, req_type, indent_pkg, proto_package, all_messages, all_enums
      )
      req_fields_str = f'{req_ros_type} request\n'

    resp_fields_str = ''
    resp_short = get_short_name(resp_type)
    resp_msg_data = all_messages.get(target_ros_package, {}).get(resp_short)
    if resp_msg_data:
      resp_proto_pkg = resp_msg_data['proto_package']
      for field in resp_msg_data['fields']:
        resp_fields_str += (
            to_ros_field_string(
                ctx,
                field,
                indent_pkg,
                resp_proto_pkg,
                all_messages,
                all_enums,
                resp_short,
            )
            + '\n'
        )
        if field['is_map']:
          map_name = get_map_message_name(field['val_type'])
          key_ros = resolve_ros_type(
              ctx,
              field['key_type'],
              indent_pkg,
              resp_proto_pkg,
              all_messages,
              all_enums,
              resp_short,
          )
          val_ros = resolve_ros_type(
              ctx,
              field['val_type'],
              indent_pkg,
              resp_proto_pkg,
              all_messages,
              all_enums,
              resp_short,
          )
          maps_to_generate[(indent_pkg, map_name)] = (key_ros, val_ros)
    elif resp_type != 'google.protobuf.Empty':
      resp_ros_type = resolve_ros_type(
          ctx, resp_type, indent_pkg, proto_package, all_messages, all_enums
      )
      resp_fields_str = f'{resp_ros_type} response\n'

    resp_fields_str += f'{status_package}/Status status\n'
    if status_package != indent_pkg:
      ctx.register_dependency(indent_pkg, status_package)

    srv_file_path = os.path.join(srv_dir, f'{method_name}.srv')
    with open(srv_file_path, 'w', encoding='utf-8') as f:
      f.write(f'{req_fields_str}---\n{resp_fields_str}')
    print(f'GENERATOR: Generated {srv_file_path}')
    ctx.register_generated_file(indent_pkg, 'srv', method_name)

  for pkg, name in defined_types:
    target_pkg = single_package if single_package else pkg
    msg_dir = os.path.join(output_path, target_pkg, 'msg')
    os.makedirs(msg_dir, exist_ok=True)

    msg_file_path = os.path.join(msg_dir, f'{name}.msg')
    msg_lines = []
    msg_data = all_messages[pkg][name]
    curr_proto_pkg = msg_data['proto_package']
    for field in msg_data['fields']:
      msg_lines.append(
          to_ros_field_string(
              ctx,
              field,
              target_pkg,
              curr_proto_pkg,
              all_messages,
              all_enums,
              name,
          )
          + '\n'
      )
      if field['is_map']:
        map_name = get_map_message_name(field['val_type'])
        key_ros = resolve_ros_type(
            ctx,
            field['key_type'],
            target_pkg,
            curr_proto_pkg,
            all_messages,
            all_enums,
            name,
        )
        val_ros = resolve_ros_type(
            ctx,
            field['val_type'],
            target_pkg,
            curr_proto_pkg,
            all_messages,
            all_enums,
            name,
        )
        maps_to_generate[(target_pkg, map_name)] = (key_ros, val_ros)

    with open(msg_file_path, 'w', encoding='utf-8') as f:
      f.writelines(msg_lines)
    print(f'GENERATOR: Generated {msg_file_path}')
    ctx.register_generated_file(target_pkg, 'msg', name)

  for (pkg, map_name), (key_ros, val_ros) in maps_to_generate.items():
    msg_dir = os.path.join(output_path, pkg, 'msg')
    os.makedirs(msg_dir, exist_ok=True)
    msg_file_path = os.path.join(msg_dir, f'{map_name}.msg')
    with open(msg_file_path, 'w', encoding='utf-8') as f:
      f.write(f'{key_ros} first\n{val_ros} second\n')
    print(f'GENERATOR: Generated {msg_file_path}')
    ctx.register_generated_file(pkg, 'msg', map_name)

  print('GENERATOR: Generating gateway configuration...')
  generate_gateway_config(
      ctx=ctx,
      output_path=output_path,
      service_name=service_name,
      proto_package=proto_package,
      rpcs=rpcs,
      all_messages=all_messages,
      all_enums=all_enums,
      maps_to_generate=maps_to_generate,
      target_package=target_package,
      target_node=target_node,
      single_package=single_package,
  )

  print('GENERATOR: Generating build files for all packages...')
  generate_build_files(ctx, output_path)
  print('GENERATOR: Completed generating all ROS interfaces.')


def build_arg_parser(parser=None):
  """Configure argparse options for `aquarium generate ros`."""
  if parser is None:
    parser = argparse.ArgumentParser(
        prog='aquarium generate ros',
        description=(
            'Generate ROS 2 interfaces from a Proto service definition.'
        ),
    )
  parser.add_argument(
      '--service',
      required=True,
      help='Proto service name (e.g. CameraService).',
  )
  parser.add_argument(
      '--proto_path',
      required=True,
      help='Path to the service .proto file.',
  )
  parser.add_argument(
      '--output_path',
      required=True,
      help='Output directory for generated ROS 2 packages.',
  )
  parser.add_argument(
      '--single_package',
      help='Generate all messages and services into a single ROS 2 package.',
  )
  parser.add_argument(
      '--target_package',
      help='Target ROS 2 package name for the gateway config target.',
  )
  parser.add_argument(
      '--target_node',
      help='Target ROS 2 node name for the gateway config target.',
  )
  return parser


def main(argv):
  """CLI entrypoint for `aquarium generate ros`."""
  parser = build_arg_parser()
  parsed = parser.parse_args(argv)
  run_generate_ros(
      service_name=parsed.service,
      proto_path=parsed.proto_path,
      output_path=parsed.output_path,
      single_package=parsed.single_package,
      target_package=parsed.target_package,
      target_node=parsed.target_node,
  )
