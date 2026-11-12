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

"""Intrinsic Flowstate skill, service manifest, and container codegen."""

import os
import shutil
import sys
from xml.sax.saxutils import quoteattr
import yaml

from .config import find_launch_file_info
from .config import get_target_image_tags
from .config import get_target_packages
from .config import infer_proto_type
from .config import parse_targets_config
from .config import resolve_target_arguments
from .config import sanitize_skill_stem
from .workspace import find_sdk_ros_dir
from .workspace import find_workspace_dir
from .workspace import get_resource_path
from .workspace import get_runtime_context_proto_path
from .workspace import is_package_in_workspace_install
from .workspace import is_package_in_workspace_src
from .workspace import render_template

STAGEABLE_RESOURCES = (
    'dockerignore',
    'Dockerfile.underlay_deps',
    'Dockerfile.underlay',
    'Dockerfile.service',
    'Dockerfile.skill',
    'entrypoint.sh',
    'skill_entrypoint.py',
    'prune_runtime_artifacts.py',
)

EXECUTABLE_RESOURCES = frozenset({
    'entrypoint.sh',
    'skill_entrypoint.py',
    'prune_runtime_artifacts.py',
})


def stage_container_resources(output_dir, workspace_dir=None):
  """Copy packaged Dockerfiles and runtime scripts into output_dir."""
  os.makedirs(output_dir, exist_ok=True)
  for resource_name in STAGEABLE_RESOURCES:
    src_path = get_resource_path(resource_name)
    dst_path = os.path.join(output_dir, resource_name)
    shutil.copyfile(src_path, dst_path)
    if resource_name in EXECUTABLE_RESOURCES:
      os.chmod(dst_path, 0o755)

  if workspace_dir:
    rel_out = os.path.relpath(output_dir, workspace_dir).replace(os.sep, '/')
    if rel_out.startswith('build/') and rel_out != 'build/aquarium_images':
      dockerignore_path = os.path.join(output_dir, 'dockerignore')
      parts = rel_out.split('/')
      extra_lines = []
      for idx in range(2, len(parts) + 1):
        prefix = '/'.join(parts[:idx])
        extra_lines.append(f'!{prefix}/\n')
      extra_lines.append(f'{rel_out}/*.tar\n')
      extra_lines.append(f'{rel_out}/**/*.tar\n')
      with open(dockerignore_path, 'a', encoding='utf-8') as f:
        f.writelines(extra_lines)


def map_ros_type_to_proto(ros_type):
  """Map a ROS field type string from aquarium.yaml to Protobuf syntax."""
  mapping = {
      'int64': 'int64',
      'int32': 'int32',
      'int16': 'int32',
      'int8': 'int32',
      'uint64': 'uint64',
      'uint32': 'uint32',
      'uint16': 'uint32',
      'uint8': 'uint32',
      'float64': 'double',
      'float32': 'float',
      'double': 'double',
      'float': 'float',
      'string': 'string',
      'bool': 'bool',
      'boolean': 'bool',
  }
  is_array = False
  if ros_type.endswith('[]') or '[' in ros_type:
    is_array = True
    base_type = ros_type.split('[')[0].strip()
  else:
    base_type = ros_type.strip()

  if base_type in mapping:
    proto_type = mapping[base_type]
  elif '/' in base_type:
    proto_type = base_type.split('/')[-1]
  else:
    proto_type = base_type

  if is_array:
    return f'repeated {proto_type}'
  return proto_type


def _base_ros_type(field_type):
  """Strip array suffixes and whitespace from a ROS field type string."""
  if '[' in field_type:
    return field_type.split('[', 1)[0].strip()
  return field_type.strip()


def _select_skill_types(req_fields, res_fields, types_list):
  """Return deduplicated, dependency-ordered nested types used by a skill."""
  if not types_list:
    return []

  by_full = {}
  by_short = {}
  for t_info in types_list:
    raw_name = (t_info.get('name') or '').strip()
    fields = t_info.get('fields') or []
    if not raw_name or not fields:
      continue
    short_name = raw_name.split('/')[-1]
    if raw_name not in by_full:
      by_full[raw_name] = t_info
    by_short.setdefault(short_name, []).append(t_info)

  ordered = []
  visited_full = set()
  chosen_by_short = {}

  def _resolve_type_info(type_token):
    base = _base_ros_type(type_token)
    if base in by_full:
      return by_full[base]
    short = base.split('/')[-1]
    candidates = by_short.get(short, [])
    if len(candidates) == 1:
      return candidates[0]
    if len(candidates) > 1:
      for cand in candidates:
        if cand.get('name') == base:
          return cand
      return candidates[0]
    return None

  def _visit(type_token):
    t_info = _resolve_type_info(type_token)
    if t_info is None:
      return
    full_name = t_info.get('name', '').strip()
    short_name = full_name.split('/')[-1]
    prev_info = chosen_by_short.get(short_name)
    if prev_info is not None and prev_info.get('fields') != t_info.get(
        'fields'
    ):
      print(
          'BUILDER ERROR: Conflicting nested ROS types'
          f" '{prev_info.get('name')}' and '{full_name}' share Protobuf"
          f" message name '{short_name}'."
      )
      sys.exit(1)
    if full_name in visited_full:
      return
    visited_full.add(full_name)
    chosen_by_short[short_name] = t_info
    for field_spec in t_info.get('fields') or []:
      parts = field_spec.split(maxsplit=1)
      if len(parts) == 2:
        _visit(parts[0])
    ordered.append(t_info)

  for ft, _ in list(req_fields) + list(res_fields):
    _visit(ft)

  return ordered


def generate_skill(interface, skills_dir, workspace_dir, types_list=None):
  """Generate an Intrinsic C++ skill ROS package for an exposed interface."""
  del workspace_dir
  internal_interface = interface.get('internal_interface')
  internal_name = interface.get('internal_name')
  external_name = interface.get('external_name')

  skill_stem = sanitize_skill_stem(external_name)
  skill_name = f'{skill_stem}_skill'
  skill_package_dir = os.path.join(skills_dir, skill_name)
  os.makedirs(skill_package_dir, exist_ok=True)

  if 'type_hash' not in interface:
    print(
        f"BUILDER ERROR: Interface config for '{internal_name}' is missing"
        ' dynamic type details (type_hash). Run the assistant first.'
    )
    sys.exit(1)

  req_fields = []
  if internal_interface in ('ros_service', 'ros_action_client'):
    for field_spec in interface.get('request_fields') or []:
      parts = field_spec.split(maxsplit=1)
      if len(parts) == 2:
        req_fields.append((parts[0], parts[1]))
      else:
        print(
            'BUILDER ERROR: Invalid request field format in config:'
            f" '{field_spec}'"
        )
        sys.exit(1)
    if internal_interface == 'ros_action_client':
      if not any(fn == 'action_name' for _, fn in req_fields):
        req_fields.append(('string', 'action_name'))
  elif internal_interface == 'ros_topic_subscription':
    req_fields = [
        ('bool', 'wait_for_next_message'),
        ('double', 'timeout_seconds'),
    ]

  res_fields = []
  for field_spec in interface.get('response_fields') or []:
    parts = field_spec.split(maxsplit=1)
    if len(parts) == 2:
      res_fields.append((parts[0], parts[1]))
    else:
      print(
          'BUILDER ERROR: Invalid response field format in config:'
          f" '{field_spec}'"
      )
      sys.exit(1)

  proto_msg_class = skill_stem.title().replace('_', '')
  skill_guard = f'{skill_name.upper()}_H_'

  # 1. Generate .proto definition
  proto_path = os.path.join(skill_package_dir, f'{skill_name}.proto')
  proto_lines = [
      'syntax = "proto3";\n',
      'package ai.intrinsic;\n\n',
  ]
  selected_types = _select_skill_types(req_fields, res_fields, types_list)
  for t_info in selected_types:
    t_raw_name = t_info.get('name', '')
    t_name = t_raw_name.split('/')[-1]
    t_fields = t_info.get('fields', [])
    if t_name and t_fields:
      proto_lines.append(f'message {t_name} {{\n')
      for idx, field_spec in enumerate(t_fields, 1):
        parts = field_spec.split(maxsplit=1)
        if len(parts) == 2:
          ft, fn = parts[0], parts[1]
          proto_lines.append(f'  {map_ros_type_to_proto(ft)} {fn} = {idx};\n')
      proto_lines.append('}\n\n')

  proto_lines.append(f'message {proto_msg_class}Params {{\n')
  for idx, (ft, fn) in enumerate(req_fields, 1):
    proto_lines.append(f'  {map_ros_type_to_proto(ft)} {fn} = {idx};\n')
  proto_lines.append('}\n\n')

  proto_lines.append(f'message {proto_msg_class}Result {{\n')
  proto_lines.append('  string aquarium_skill_status = 1;\n')
  for idx, (ft, fn) in enumerate(res_fields, 2):
    proto_lines.append(f'  {map_ros_type_to_proto(ft)} {fn} = {idx};\n')
  proto_lines.append('}\n')

  with open(proto_path, 'w', encoding='utf-8') as f:
    f.writelines(proto_lines)

  # 2. Generate skill manifest textproto
  manifest_path = os.path.join(
      skill_package_dir, f'{skill_name}.manifest.textproto'
  )
  manifest_content = (
      '# proto-file: https://github.com/intrinsic-ai/sdk/blob/main/'
      'intrinsic/skills/proto/skill_manifest.proto\n'
      '# proto-message: intrinsic_proto.skill.SkillManifest\n'
      'id {\n'
      '  package: "ai.intrinsic"\n'
      f'  name: "{skill_name}"\n'
      '}\n'
      f'display_name: "{skill_name.replace("_", " ").title()}"\n'
      'vendor {\n'
      '  display_name: "Intrinsic"\n'
      '}\n'
      'options {\n'
      '  supports_cancellation: false\n'
      '  cc_config {\n'
      f'    create_skill: "::{proto_msg_class}Skill::CreateSkill"\n'
      '  }\n'
      '}\n'
      f'parameter {{ message_full_name: "ai.intrinsic.{proto_msg_class}Params"'
      ' }\n'
      'return_type {'
      f' message_full_name: "ai.intrinsic.{proto_msg_class}Result" }}\n'
  )
  with open(manifest_path, 'w', encoding='utf-8') as f:
    f.write(manifest_content)

  # 3. Render package.xml, CMakeLists.txt, header, and skill implementation
  escaped_external_name = external_name.replace('\\', '\\\\').replace(
      '"', '\\"'
  )
  common_tokens = {
      'SKILL_NAME': skill_name,
      'SKILL_GUARD': skill_guard,
      'PROTO_MSG_CLASS': proto_msg_class,
      'EXTERNAL_NAME': escaped_external_name,
  }
  template_outputs = (
      ('skill_package.xml.in', 'package.xml'),
      ('skill_CMakeLists.txt.in', 'CMakeLists.txt'),
      ('skill.h.in', f'{skill_name}.h'),
      ('skill.cc.in', f'{skill_name}.cc'),
  )
  for template_file, output_file in template_outputs:
    rendered = render_template(template_file, common_tokens)
    out_path = os.path.join(skill_package_dir, output_file)
    with open(out_path, 'w', encoding='utf-8') as f:
      f.write(rendered)

  # 4. Render test client
  setup_lines = []
  for ft, fn in req_fields:
    proto_type = map_ros_type_to_proto(ft)
    if proto_type in ('int32', 'int64', 'uint32', 'uint64'):
      setup_lines.append(f'  params.set_{fn}(42);')
    elif proto_type in ('double', 'float'):
      setup_lines.append(f'  params.set_{fn}(3.14);')
    elif proto_type == 'string':
      setup_lines.append(f'  params.set_{fn}("test");')
    elif proto_type == 'bool':
      setup_lines.append(f'  params.set_{fn}(true);')
    elif not proto_type.startswith('repeated '):
      setup_lines.append(f'  params.mutable_{fn}();')

  client_tokens = dict(common_tokens)
  client_tokens['PARAMS_SETUP'] = '\n'.join(setup_lines)
  client_path = os.path.join(skill_package_dir, f'{skill_name}_client.cc')
  with open(client_path, 'w', encoding='utf-8') as f:
    f.write(render_template('skill_client.cc.in', client_tokens))

  print(f'GENERATOR: Generated Skill package at {skill_package_dir}')
  return skill_name


def generate_launch_xml_content(target, workspace_dir=None):
  """Generate aquarium_launch.xml content for a node or launch target."""
  target_type = target.get('type')
  package = target.get('package')
  name = target.get('name')
  target_arguments = resolve_target_arguments(target, workspace_dir)

  launch_content = '<launch>\n'

  if target_type == 'node':
    args_str = ' '.join(target_arguments) if target_arguments else ''
    launch_content += (
        f'  <node pkg={quoteattr(package)} exec={quoteattr(name)}'
        f' name={quoteattr(name)} args={quoteattr(args_str)}/>\n'
    )
  elif target_type == 'launch':
    _, launch_rel_path = find_launch_file_info(package, name, workspace_dir)
    if target_arguments:
      for arg in target_arguments:
        if ':=' in arg:
          k, v = arg.split(':=', 1)
          launch_content += (
              f'  <arg name={quoteattr(k)} default={quoteattr(v)}/>\n'
          )
        elif not arg.startswith('-') and arg.strip():
          k = arg.strip()
          launch_content += f'  <arg name={quoteattr(k)} default=""/>\n'

    include_file = f'$(find-pkg-share {package})/{launch_rel_path}'
    launch_content += f'  <include file={quoteattr(include_file)}>\n'
    if target_arguments:
      for arg in target_arguments:
        if ':=' in arg:
          k, _ = arg.split(':=', 1)
          var_ref = f'$(var {k})'
          launch_content += (
              f'    <arg name={quoteattr(k)} value={quoteattr(var_ref)}/>\n'
          )
        elif not arg.startswith('-') and arg.strip():
          k = arg.strip()
          var_ref = f'$(var {k})'
          launch_content += (
              f'    <arg name={quoteattr(k)} value={quoteattr(var_ref)}/>\n'
          )
    launch_content += '  </include>\n'

  launch_content += (
      '  <node pkg="aquarium" exec="aquarium_gateway"'
      ' name="aquarium_gateway">\n'
      '    <param name="config_file" value="/workspace/aquarium.yaml"/>\n'
      '  </node>\n'
      '</launch>\n'
  )
  return launch_content


def generate_entrypoint_content(
    service_name_clean, config_msg_name, target_arguments=None
):
  """Render service container entrypoint.py from service_entrypoint.py.in."""
  allowed_arg_names = []
  for arg in target_arguments or []:
    if ':=' in arg:
      allowed_arg_names.append(arg.split(':=', 1)[0])
    elif not arg.startswith('-') and arg.strip():
      allowed_arg_names.append(arg.strip())

  return render_template(
      'service_entrypoint.py.in',
      {
          'SERVICE_NAME_CLEAN': service_name_clean,
          'CONFIG_MSG_NAME': config_msg_name,
          'ALLOWED_ARG_NAMES_REPR': repr(allowed_arg_names),
      },
  )


def _infer_apt_packages_str(target_cfg, workspace_dir, ros_distro):
  """Infer space-joined ros-<distro>-<pkg> names for non-workspace packages."""
  apt_pkgs = []
  for pkg in get_target_packages(target_cfg):
    if not is_package_in_workspace_src(
        workspace_dir, pkg
    ) and not is_package_in_workspace_install(workspace_dir, pkg):
      apt_name = f"ros-{ros_distro}-{pkg.replace('_', '-')}"
      if apt_name not in apt_pkgs:
        apt_pkgs.append(apt_name)
  return ' '.join(apt_pkgs)


def _print_generation_instructions(
    workspace_dir,
    output_dir,
    real_target,
    sim_target,
    real_image_tag,
    sim_image_tag,
    base_image_tag,
    service_name_clean,
    real_apt_package,
    sim_apt_package,
    service_manifest_path,
    generated_skills,
):
  """Print copy-pasteable buildah and inbuild commands for manual builds."""
  output_dir_rel = os.path.relpath(output_dir, workspace_dir)
  config_rel = os.path.relpath(
      os.path.join(output_dir, 'aquarium.yaml'), workspace_dir
  )
  service_manifest_rel = os.path.relpath(service_manifest_path, workspace_dir)
  ignorefile_rel = os.path.join(output_dir_rel, 'dockerignore')
  dockerfile_deps_rel = os.path.join(output_dir_rel, 'Dockerfile.underlay_deps')
  dockerfile_underlay_rel = os.path.join(output_dir_rel, 'Dockerfile.underlay')
  dockerfile_service_rel = os.path.join(output_dir_rel, 'Dockerfile.service')
  dockerfile_skill_rel = os.path.join(output_dir_rel, 'Dockerfile.skill')
  sdk_ros_rel = find_sdk_ros_dir(workspace_dir)

  print('\nGENERATOR: Generation complete.')
  print(
      'First, if you have not built the SDK underlay images yet, build them'
      ' with:'
  )
  print(
      f'  buildah bud --ignorefile {ignorefile_rel} --format docker'
      f' -t aquarium-sdk-underlay-deps:latest -f {dockerfile_deps_rel} .'
  )
  print(
      f'  buildah bud --ignorefile {ignorefile_rel} --format docker'
      ' -t aquarium-sdk-underlay:latest'
      f' --build-arg SDK_ROS_DIR={sdk_ros_rel}'
      f' -f {dockerfile_underlay_rel} .'
  )

  print(
      '\nTo build and bundle the Service, run the following commands from the'
      ' workspace root:'
  )

  inbuild_cmd = 'inbuild'
  if not shutil.which('inbuild'):
    local_inbuild = os.path.join(
        workspace_dir, 'install', 'intrinsic_sdk_cmake', 'bin', 'inbuild'
    )
    if os.path.exists(local_inbuild):
      inbuild_cmd = os.path.relpath(local_inbuild, workspace_dir)

  def _print_service_image_cmd(target_cfg, img_tag, apt_pkg, l_file, e_file):
    pkgs_str = ' '.join(get_target_packages(target_cfg))
    pkg_arg = f'"{pkgs_str}"' if ' ' in pkgs_str else pkgs_str
    apt_arg = f'"{apt_pkg}"' if ' ' in apt_pkg else apt_pkg
    t_rosdep = target_cfg.get('rosdep') or {}
    r_skip = t_rosdep.get('skip', []) if isinstance(t_rosdep, dict) else []
    r_skip_str = ' '.join(r_skip) if isinstance(r_skip, list) else ''
    t_prune = target_cfg.get('prune') or {}
    p_pats = t_prune.get('patterns', []) if isinstance(t_prune, dict) else []
    p_pats_str = ' '.join(p_pats) if isinstance(p_pats, list) else ''
    l_rel = os.path.relpath(os.path.join(output_dir, l_file), workspace_dir)
    e_rel = os.path.relpath(os.path.join(output_dir, e_file), workspace_dir)

    print(f'  # Build image {img_tag}')
    print(
        f'  buildah bud --ignorefile {ignorefile_rel}'
        f' --format docker -t {img_tag} \\'
    )
    print(f'    --build-arg OUTPUT_DIR={output_dir_rel} \\')
    print(f'    --build-arg TARGET_PACKAGE={pkg_arg} \\')
    print(f'    --build-arg TARGET_APT_PACKAGE={apt_arg} \\')
    if r_skip_str:
      print(f'    --build-arg ROSDEP_SKIP_KEYS="{r_skip_str}" \\')
    if p_pats_str:
      print(f'    --build-arg PRUNE_PATTERNS="{p_pats_str}" \\')
    print(f'    --build-arg CONFIG_PATH={config_rel} \\')
    print(f'    --build-arg LAUNCH_PATH={l_rel} \\')
    print(f'    --build-arg ENTRYPOINT_PATH={e_rel} \\')
    print(f'    -f {dockerfile_service_rel} .')
    print(f'  # Save image {img_tag} to tar')
    print(
        f'  buildah push {img_tag}:latest'
        f' docker-archive:{output_dir_rel}/{img_tag}.tar'
    )

  service_desc_rel = os.path.join(
      output_dir_rel, f'{service_name_clean}_config.desc'
  )
  default_cfg_rel = os.path.join(
      output_dir_rel, f'{base_image_tag}_default_config.textproto'
  )

  if real_target == sim_target:
    _print_service_image_cmd(
        real_target,
        real_image_tag,
        real_apt_package,
        'aquarium_launch.xml',
        'entrypoint.py',
    )
    oci_flags = [f'    --oci_image {output_dir_rel}/{real_image_tag}.tar \\']
  else:
    _print_service_image_cmd(
        real_target,
        real_image_tag,
        real_apt_package,
        'aquarium_launch_real.xml',
        'entrypoint_real.py',
    )
    _print_service_image_cmd(
        sim_target,
        sim_image_tag,
        sim_apt_package,
        'aquarium_launch_sim.xml',
        'entrypoint_sim.py',
    )
    oci_flags = [
        f'    --oci_image {output_dir_rel}/{real_image_tag}.tar \\',
        f'    --oci_image {output_dir_rel}/{sim_image_tag}.tar \\',
    ]

  print('  # Extract service config descriptor and create Flowstate bundle')
  print(
      f'  buildah unshare bash -c "ctr=\\$(buildah from'
      f' {real_image_tag}:latest) && mnt=\\$(buildah mount \\$ctr) &&'
      f' cp \\$mnt/workspace/config.desc {service_desc_rel} &&'
      ' buildah unmount \\$ctr && buildah rm \\$ctr"'
  )
  print(f'  {inbuild_cmd} service bundle \\')
  print(f'    --manifest {service_manifest_rel} \\')
  for line in oci_flags:
    print(line)
  print(f'    --default_config {default_cfg_rel} \\')
  print(f'    --file_descriptor_set {service_desc_rel} \\')
  print(f'    --output {output_dir_rel}/{base_image_tag}.bundle.tar')

  all_prune_patterns = []
  for tgt in (real_target, sim_target):
    t_prune = tgt.get('prune') or {}
    if isinstance(t_prune, dict):
      for pat in t_prune.get('patterns', []):
        if pat not in all_prune_patterns:
          all_prune_patterns.append(pat)
  prune_patterns_str = ' '.join(all_prune_patterns)

  if generated_skills:
    skills_dir = os.path.join(output_dir, 'skills')
    print(
        '\nTo build and bundle the Skills, run the following commands from the'
        ' workspace root:'
    )
    for skill in generated_skills:
      skill_dir_rel = os.path.relpath(
          os.path.join(skills_dir, skill), workspace_dir
      )
      skill_manifest_rel = os.path.join(
          skill_dir_rel, f'{skill}.manifest.textproto'
      )
      skill_desc_rel = os.path.join(skill_dir_rel, f'{skill}_protos.desc')
      intrinsic_desc_rel = os.path.join(output_dir_rel, 'intrinsic_proto.desc')
      augmented_manifest_rel = os.path.join(
          skill_dir_rel, f'{skill}_augmented_manifest.pbbin'
      )
      augmented_desc_rel = os.path.join(
          skill_dir_rel, f'{skill}_augmented_protos.desc'
      )

      print(f'  # --- Skill: {skill} ---')
      print(
          f'  buildah bud --ignorefile {ignorefile_rel} --format docker'
          f' -t {skill}:latest \\'
      )
      print(f'    --build-arg OUTPUT_DIR={output_dir_rel} \\')
      print(f'    --build-arg SKILL_NAME={skill} \\')
      if prune_patterns_str:
        print(f'    --build-arg PRUNE_PATTERNS="{prune_patterns_str}" \\')
      print(f'    -f {dockerfile_skill_rel} .')
      print(
          f'  buildah unshare bash -c "ctr=\\$(buildah from {skill}:latest) &&'
          ' mnt=\\$(buildah mount \\$ctr) && cp'
          f' \\$mnt/opt/{skill}_workspace/install/{skill}/share/{skill}/'
          f'{skill}_protos.desc {skill_desc_rel} &&'
          f' found_desc=\\$(find \\$mnt/opt/{skill}_workspace/install -name'
          " 'intrinsic_proto.desc' -print -quit) &&"
          ' if [ -n \\"\\$found_desc\\" ]; then cp \\"\\$found_desc\\"'
          f' {intrinsic_desc_rel}; fi && buildah unmount \\$ctr &&'
          ' buildah rm \\$ctr"'
      )
      print(
          f'  buildah push {skill}:latest'
          f' docker-archive:{skill_dir_rel}/{skill}.tar'
      )
      print(f'  {inbuild_cmd} skill manifest \\')
      print(f'    --manifest {skill_manifest_rel} \\')
      print(
          f'    --file_descriptor_sets {skill_desc_rel},{intrinsic_desc_rel} \\'
      )
      print(f'    --output {augmented_manifest_rel} \\')
      print(f'    --file_descriptor_set_out {augmented_desc_rel}')
      print(f'  {inbuild_cmd} skill bundle \\')
      print(f'    --augmented_manifest {augmented_manifest_rel} \\')
      print(f'    --augmented_file_descriptor_set {augmented_desc_rel} \\')
      print(f'    --oci_image {skill_dir_rel}/{skill}.tar \\')
      print(f'    --output {output_dir_rel}/{skill}.bundle.tar')


def run_generate(config_path, output_dir=None, privileged=False):
  """Generate Intrinsic service manifests, entrypoints, and C++ skills."""
  if not os.path.exists(config_path):
    print(f"Error: Configuration file '{config_path}' not found.")
    sys.exit(1)

  try:
    with open(config_path, 'r', encoding='utf-8') as f:
      config = yaml.safe_load(f)
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f'Error: Failed to parse YAML configuration: {e}')
    sys.exit(1)

  real_target, sim_target = parse_targets_config(config)
  real_name = real_target.get('name')

  ros_distro = os.environ.get('ROS_DISTRO', 'lyrical')
  workspace_dir = find_workspace_dir() or os.getcwd()

  real_apt_package = _infer_apt_packages_str(
      real_target, workspace_dir, ros_distro
  )
  sim_apt_package = _infer_apt_packages_str(
      sim_target, workspace_dir, ros_distro
  )

  real_target = dict(real_target)
  sim_target = dict(sim_target)
  real_target['arguments'] = resolve_target_arguments(
      real_target, workspace_dir
  )
  sim_target['arguments'] = resolve_target_arguments(sim_target, workspace_dir)

  if not output_dir:
    output_dir = os.path.join(workspace_dir, 'build', 'aquarium_images')
  else:
    output_dir = os.path.abspath(output_dir)

  rel_output_dir = os.path.relpath(output_dir, workspace_dir)
  if rel_output_dir.startswith('..'):
    print(
        f"Error: Output directory '{output_dir}' must be inside workspace"
        f" '{workspace_dir}'"
    )
    sys.exit(1)

  os.makedirs(output_dir, exist_ok=True)
  with open(os.path.join(output_dir, 'COLCON_IGNORE'), 'w', encoding='utf-8'):
    pass

  stage_container_resources(output_dir, workspace_dir)

  for fname in os.listdir(output_dir):
    if fname.endswith('_config.proto'):
      try:
        os.remove(os.path.join(output_dir, fname))
      except OSError:
        pass

  # Generate launch file(s)
  real_launch_content = generate_launch_xml_content(real_target, workspace_dir)
  launch_path = os.path.join(output_dir, 'aquarium_launch.xml')
  try:
    with open(launch_path, 'w', encoding='utf-8') as f:
      f.write(real_launch_content)
    print(f'GENERATOR: Generated launch file at {launch_path}')
  except OSError as e:
    print(f'Error: Failed to write launch file: {e}')
    sys.exit(1)

  if real_target != sim_target:
    real_launch_path = os.path.join(output_dir, 'aquarium_launch_real.xml')
    sim_launch_path = os.path.join(output_dir, 'aquarium_launch_sim.xml')
    sim_launch_content = generate_launch_xml_content(sim_target, workspace_dir)
    try:
      with open(real_launch_path, 'w', encoding='utf-8') as f:
        f.write(real_launch_content)
      with open(sim_launch_path, 'w', encoding='utf-8') as f:
        f.write(sim_launch_content)
      print(
          f'GENERATOR: Generated real and sim launch files at'
          f' {real_launch_path} and {sim_launch_path}'
      )
    except OSError as e:
      print(f'Error: Failed to write sim/real launch files: {e}')
      sys.exit(1)

  # Copy aquarium.yaml into output_dir (skip if already identical file)
  dst_config_path = os.path.join(output_dir, 'aquarium.yaml')
  try:
    if os.path.abspath(config_path) != os.path.abspath(dst_config_path):
      shutil.copy(config_path, dst_config_path)
    print(f'GENERATOR: Copied config to {dst_config_path}')
  except OSError as e:
    print(f'Error: Failed to copy config file: {e}')
    sys.exit(1)

  # Generate skills for exposed interfaces
  skills_dir = os.path.join(output_dir, 'skills')
  os.makedirs(skills_dir, exist_ok=True)

  generated_skills = []
  types_list = config.get('types', [])
  for interface in config.get('interfaces', []):
    if interface.get('internal_interface') in (
        'ros_service',
        'ros_topic_subscription',
        'ros_action_client',
    ):
      skill_pkg_name = generate_skill(
          interface, skills_dir, workspace_dir, types_list=types_list
      )
      if skill_pkg_name:
        generated_skills.append(skill_pkg_name)

  # Generate service config proto and default config textproto
  (
      base_image_tag,
      service_name_clean,
      real_image_tag,
      sim_image_tag,
  ) = get_target_image_tags(real_target, sim_target)
  config_msg_name = (
      ''.join(x.title() for x in service_name_clean.split('_')) + 'Config'
  )
  config_msg_full_name = f'ai.intrinsic.{config_msg_name}'

  config_fields = []
  seen_keys = set()
  field_idx = 1
  for tgt in (real_target, sim_target):
    tgt_type = tgt.get('type')
    tgt_args = tgt.get('arguments', []) or []
    for arg in tgt_args:
      if ':=' in arg:
        k, v = arg.split(':=', 1)
        if k not in seen_keys:
          seen_keys.add(k)
          t = infer_proto_type(v)
          config_fields.append((t, k, field_idx, v))
          field_idx += 1
      elif tgt_type == 'launch' and not arg.startswith('-'):
        k = arg.strip()
        if k and k not in seen_keys:
          seen_keys.add(k)
          config_fields.append(('string', k, field_idx, None))
          field_idx += 1

  proto_path = os.path.join(output_dir, f'{service_name_clean}_config.proto')
  proto_lines = [
      'syntax = "proto3";\n',
      'package ai.intrinsic;\n\n',
      f'message {config_msg_name} {{\n',
  ]
  for t, k, idx, _ in config_fields:
    proto_lines.append(f'  optional {t} {k} = {idx};\n')
  proto_lines.append('}\n')

  with open(proto_path, 'w', encoding='utf-8') as f:
    f.writelines(proto_lines)
  print(f'GENERATOR: Generated service config proto at {proto_path}')

  runtime_context_proto_path = get_runtime_context_proto_path(workspace_dir)
  shutil.copyfile(
      runtime_context_proto_path,
      os.path.join(output_dir, 'runtime_context.proto'),
  )

  default_config_path = os.path.join(
      output_dir, f'{base_image_tag}_default_config.textproto'
  )
  default_cfg_lines = [
      '# proto-file: google/protobuf/any.proto\n',
      '# proto-message: Any\n',
      f'[type.googleapis.com/{config_msg_full_name}] {{\n',
  ]
  for t, k, _, v in config_fields:
    if v is None:
      continue
    if t == 'string':
      escaped_v = v.replace('"', '\\"')
      default_cfg_lines.append(f'  {k}: "{escaped_v}"\n')
    elif t == 'bool':
      default_cfg_lines.append(f'  {k}: {v.lower()}\n')
    else:
      default_cfg_lines.append(f'  {k}: {v}\n')
  default_cfg_lines.append('}\n')

  with open(default_config_path, 'w', encoding='utf-8') as f:
    f.writelines(default_cfg_lines)
  print(
      f'GENERATOR: Generated default config textproto at {default_config_path}'
  )

  # Generate entrypoint.py (and real/sim entrypoints if targets differ)
  real_entrypoint_content = generate_entrypoint_content(
      service_name_clean, config_msg_name, real_target.get('arguments', [])
  )
  entrypoint_path = os.path.join(output_dir, 'entrypoint.py')
  with open(entrypoint_path, 'w', encoding='utf-8') as f:
    f.write(real_entrypoint_content)
  os.chmod(entrypoint_path, 0o755)
  print(f'GENERATOR: Generated entrypoint.py at {entrypoint_path}')

  if real_target != sim_target:
    real_ep_path = os.path.join(output_dir, 'entrypoint_real.py')
    sim_ep_path = os.path.join(output_dir, 'entrypoint_sim.py')
    sim_entrypoint_content = generate_entrypoint_content(
        service_name_clean, config_msg_name, sim_target.get('arguments', [])
    )
    with open(real_ep_path, 'w', encoding='utf-8') as f:
      f.write(real_entrypoint_content)
    os.chmod(real_ep_path, 0o755)
    with open(sim_ep_path, 'w', encoding='utf-8') as f:
      f.write(sim_entrypoint_content)
    os.chmod(sim_ep_path, 0o755)
    print(
        f'GENERATOR: Generated real and sim entrypoints at {real_ep_path} and'
        f' {sim_ep_path}'
    )

  # Generate service manifest textproto
  service_manifest_path = os.path.join(
      output_dir, f'{base_image_tag}.manifest.textproto'
  )
  security_context_block = ''
  if privileged:
    security_context_block = (
        '      settings {\n'
        '        security_context {\n'
        '          privileged: true\n'
        '        }\n'
        '      }\n'
    )

  real_archive = f'{real_image_tag}.tar'
  sim_archive = f'{sim_image_tag}.tar'
  if real_archive == sim_archive:
    image_filenames_str = f'["{real_archive}"]'
  else:
    image_filenames_str = f'["{real_archive}", "{sim_archive}"]'

  display_title = real_name.replace('_', ' ').replace('/', ' ').title()
  service_manifest_content = (
      'metadata {\n'
      '  id {\n'
      '    package: "ai.intrinsic"\n'
      f'    name: "{service_name_clean}"\n'
      '  }\n'
      '  vendor {\n'
      '    display_name: "Intrinsic"\n'
      '  }\n'
      f'  display_name: "{display_title} Service"\n'
      '}\n'
      'service_def {\n'
      f'  config_message_full_name: "{config_msg_full_name}"\n'
      '  real_spec {\n'
      '    image {\n'
      f'      archive_filename: "{real_archive}"\n'
      f'{security_context_block}'
      '    }\n'
      '  }\n'
      '  sim_spec {\n'
      '    image {\n'
      f'      archive_filename: "{sim_archive}"\n'
      f'{security_context_block}'
      '    }\n'
      '  }\n'
      '}\n'
      'assets {\n'
      f'  image_filenames: {image_filenames_str}\n'
      '}\n'
  )
  try:
    with open(service_manifest_path, 'w', encoding='utf-8') as f:
      f.write(service_manifest_content)
    print(f'GENERATOR: Generated service manifest at {service_manifest_path}')
  except OSError as e:
    print(f'Error: Failed to write service manifest: {e}')
    sys.exit(1)

  _print_generation_instructions(
      workspace_dir=workspace_dir,
      output_dir=output_dir,
      real_target=real_target,
      sim_target=sim_target,
      real_image_tag=real_image_tag,
      sim_image_tag=sim_image_tag,
      base_image_tag=base_image_tag,
      service_name_clean=service_name_clean,
      real_apt_package=real_apt_package,
      sim_apt_package=sim_apt_package,
      service_manifest_path=service_manifest_path,
      generated_skills=generated_skills,
  )
