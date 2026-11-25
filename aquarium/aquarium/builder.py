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

"""Container image building and Flowstate asset bundling via buildah/inbuild."""

import os
import shutil
import subprocess
import sys
import yaml

from .codegen_intrinsic import run_generate
from .codegen_intrinsic import stage_container_resources
from .config import get_target_image_tags
from .config import get_target_packages
from .config import parse_targets_config
from .config import resolve_target_arguments
from .config import sanitize_skill_stem
from .config import validate_target_packages
from .workspace import find_sdk_ros_dir
from .workspace import find_workspace_dir
from .workspace import get_inbuild_path
from .workspace import get_intrinsic_proto_desc_path


def _get_clean_env():
  """Return environment copy without stale XDG_RUNTIME_DIR paths."""
  env = os.environ.copy()
  if 'XDG_RUNTIME_DIR' in env and not os.path.exists(env['XDG_RUNTIME_DIR']):
    del env['XDG_RUNTIME_DIR']
  return env


def run_build(config_path, privileged=False, no_cache=False):
  """Generate artifacts, build OCI container images, and create bundles."""
  workspace_dir = find_workspace_dir()
  if not workspace_dir:
    print(
        "Error: 'aquarium build' must be run from a ROS workspace containing"
        " 'src/'."
    )
    sys.exit(1)

  if not os.path.exists(config_path):
    print(f"Error: Configuration file '{config_path}' not found.")
    sys.exit(1)

  if not shutil.which('buildah'):
    print(
        "Error: 'buildah' not found on PATH. Install it with:"
        ' sudo apt install buildah'
    )
    sys.exit(1)

  try:
    with open(config_path, 'r', encoding='utf-8') as f:
      config = yaml.safe_load(f)
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f'Error: Failed to parse YAML configuration: {e}')
    sys.exit(1)

  real_target, sim_target = parse_targets_config(config)
  real_target = dict(real_target)
  sim_target = dict(sim_target)
  real_target['arguments'] = resolve_target_arguments(
      real_target, workspace_dir
  )
  sim_target['arguments'] = resolve_target_arguments(sim_target, workspace_dir)

  real_apt_package = validate_target_packages(real_target, workspace_dir)
  if real_target != sim_target:
    sim_apt_package = validate_target_packages(sim_target, workspace_dir)
  else:
    sim_apt_package = real_apt_package

  os.makedirs(os.path.join(workspace_dir, 'install'), exist_ok=True)

  output_dir = os.path.join(workspace_dir, 'build', 'aquarium_images')
  stage_container_resources(output_dir, workspace_dir)
  with open(os.path.join(output_dir, 'COLCON_IGNORE'), 'w', encoding='utf-8'):
    pass

  output_dir_rel = os.path.relpath(output_dir, workspace_dir)
  config_rel = os.path.relpath(
      os.path.join(output_dir, 'aquarium.yaml'), workspace_dir
  )
  sdk_ros_rel = find_sdk_ros_dir(workspace_dir)

  (
      base_image_tag,
      service_name_clean,
      real_image_tag,
      sim_image_tag,
  ) = get_target_image_tags(real_target, sim_target)
  service_manifest_path = os.path.join(
      output_dir, f'{base_image_tag}.manifest.textproto'
  )
  service_manifest_rel = os.path.relpath(service_manifest_path, workspace_dir)

  ignorefile_rel = os.path.join(output_dir_rel, 'dockerignore')
  dockerfile_service_rel = os.path.join(output_dir_rel, 'Dockerfile.service')
  dockerfile_skill_rel = os.path.join(output_dir_rel, 'Dockerfile.skill')
  dockerfile_underlay_rel = os.path.join(output_dir_rel, 'Dockerfile.underlay')
  dockerfile_underlay_deps_rel = os.path.join(
      output_dir_rel, 'Dockerfile.underlay_deps'
  )

  def run_cmd(cmd):
    print(f"BUILDER: Running: {' '.join(cmd)}")
    res = subprocess.run(
        cmd, cwd=workspace_dir, env=_get_clean_env(), check=True
    )
    return res.returncode == 0

  if no_cache:
    print('BUILDER: Pulling latest upstream base images...')
    run_cmd([
        'buildah',
        'pull',
        '--policy',
        'always',
        'docker.io/library/ros:lyrical',
    ])
    run_cmd([
        'buildah',
        'pull',
        '--policy',
        'always',
        'docker.io/library/ros:lyrical-ros-core',
    ])

  # 1. Build underlay dependencies image if missing or --no-cache
  underlay_deps_found = False
  if not no_cache:
    underlay_deps_check = subprocess.run(
        [
            'buildah',
            'inspect',
            '--type',
            'image',
            'aquarium-sdk-underlay-deps:latest',
        ],
        env=_get_clean_env(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    underlay_deps_found = underlay_deps_check.returncode == 0

  if not underlay_deps_found:
    print(
        'BUILDER: Underlay dependencies image'
        " 'aquarium-sdk-underlay-deps:latest' not found or --no-cache set."
        ' Building it...'
    )
    deps_cmd = ['buildah', 'bud']
    if no_cache:
      deps_cmd.append('--no-cache')
    deps_cmd.extend([
        '--ignorefile',
        ignorefile_rel,
        '--format',
        'docker',
        '-t',
        'aquarium-sdk-underlay-deps:latest',
        '-f',
        dockerfile_underlay_deps_rel,
        '.',
    ])
    run_cmd(deps_cmd)
  else:
    print(
        'BUILDER: Underlay dependencies image'
        " 'aquarium-sdk-underlay-deps:latest' found."
    )

  # 2. Build SDK underlay image if missing or --no-cache
  underlay_found = False
  if not no_cache:
    underlay_check = subprocess.run(
        [
            'buildah',
            'inspect',
            '--type',
            'image',
            'aquarium-sdk-underlay:latest',
        ],
        env=_get_clean_env(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    underlay_found = underlay_check.returncode == 0

  if not underlay_found:
    print(
        "BUILDER: Underlay image 'aquarium-sdk-underlay:latest' not found or"
        ' --no-cache set. Building it...'
    )
    underlay_cmd = ['buildah', 'bud']
    if no_cache:
      underlay_cmd.append('--no-cache')
    underlay_cmd.extend([
        '--ignorefile',
        ignorefile_rel,
        '--format',
        'docker',
        '-t',
        'aquarium-sdk-underlay:latest',
        '--build-arg',
        f'SDK_ROS_DIR={sdk_ros_rel}',
        '-f',
        dockerfile_underlay_rel,
        '.',
    ])
    run_cmd(underlay_cmd)
  else:
    print("BUILDER: Underlay image 'aquarium-sdk-underlay:latest' found.")

  # 3. Generate manifests, entrypoints, and skill sources
  run_generate(config_path, output_dir=output_dir, privileged=privileged)

  # 4. Build and save service image(s)
  def build_and_save_service_image(
      target_cfg, img_tag, apt_pkg, launch_filename, entrypoint_filename
  ):
    pkgs_str = ' '.join(get_target_packages(target_cfg))
    t_rosdep = target_cfg.get('rosdep') or {}
    r_skip = t_rosdep.get('skip', []) if isinstance(t_rosdep, dict) else []
    r_skip_str = ' '.join(r_skip) if isinstance(r_skip, list) else ''

    t_prune = target_cfg.get('prune') or {}
    p_patterns = (
        t_prune.get('patterns', []) if isinstance(t_prune, dict) else []
    )
    p_patterns_str = (
        ' '.join(p_patterns) if isinstance(p_patterns, list) else ''
    )

    l_rel = os.path.relpath(
        os.path.join(output_dir, launch_filename), workspace_dir
    )
    e_rel = os.path.relpath(
        os.path.join(output_dir, entrypoint_filename), workspace_dir
    )

    print(f"BUILDER: Building Service Image '{img_tag}'...")
    service_build_cmd = ['buildah', 'bud']
    if no_cache:
      service_build_cmd.append('--no-cache')
    service_build_cmd.extend([
        '--ignorefile',
        ignorefile_rel,
        '--format',
        'docker',
        '-t',
        img_tag,
        '--build-arg',
        f'OUTPUT_DIR={output_dir_rel}',
        '--build-arg',
        f'TARGET_PACKAGE={pkgs_str}',
        '--build-arg',
        f'TARGET_APT_PACKAGE={apt_pkg}',
    ])
    if r_skip_str:
      service_build_cmd.extend(
          ['--build-arg', f'ROSDEP_SKIP_KEYS={r_skip_str}']
      )
    if p_patterns_str:
      service_build_cmd.extend(
          ['--build-arg', f'PRUNE_PATTERNS={p_patterns_str}']
      )
    service_build_cmd.extend([
        '--build-arg',
        f'CONFIG_PATH={config_rel}',
        '--build-arg',
        f'LAUNCH_PATH={l_rel}',
        '--build-arg',
        f'ENTRYPOINT_PATH={e_rel}',
        '-f',
        dockerfile_service_rel,
        '.',
    ])
    run_cmd(service_build_cmd)

    print(f"BUILDER: Saving Service Image '{img_tag}' to tar...")
    os.makedirs(output_dir, exist_ok=True)
    t_path = os.path.join(output_dir, f'{img_tag}.tar')
    if os.path.exists(t_path):
      print(f'BUILDER: Overwriting existing tar {t_path}')
      os.remove(t_path)
    run_cmd(
        ['buildah', 'push', f'{img_tag}:latest', f'docker-archive:{t_path}']
    )
    return t_path

  if real_target == sim_target:
    real_tar_path = build_and_save_service_image(
        real_target,
        real_image_tag,
        real_apt_package,
        'aquarium_launch.xml',
        'entrypoint.py',
    )
    oci_image_args = ['--oci_image', real_tar_path]
  else:
    real_tar_path = build_and_save_service_image(
        real_target,
        real_image_tag,
        real_apt_package,
        'aquarium_launch_real.xml',
        'entrypoint_real.py',
    )
    sim_tar_path = build_and_save_service_image(
        sim_target,
        sim_image_tag,
        sim_apt_package,
        'aquarium_launch_sim.xml',
        'entrypoint_sim.py',
    )
    oci_image_args = [
        '--oci_image',
        real_tar_path,
        '--oci_image',
        sim_tar_path,
    ]

  # 5. Extract service config descriptor set and create service bundle
  service_desc_rel = os.path.join(
      output_dir_rel, f'{service_name_clean}_config.desc'
  )
  service_desc_abs = os.path.abspath(
      os.path.join(output_dir, f'{service_name_clean}_config.desc')
  )
  print(
      f'BUILDER: Extracting service config descriptor to {service_desc_rel}...'
  )
  service_bash_cmd = (
      f'ctr=$(buildah from {real_image_tag}:latest) && '
      'mnt=$(buildah mount $ctr) && '
      f'cp $mnt/workspace/config.desc {service_desc_abs} && '
      'buildah unmount $ctr && '
      'buildah rm $ctr'
  )
  run_cmd(['buildah', 'unshare', 'bash', '-c', service_bash_cmd])

  print('BUILDER: Creating Service Flowstate Bundle...')
  inbuild_bin = get_inbuild_path(workspace_dir)
  bundle_cmd = (
      [
          inbuild_bin,
          'service',
          'bundle',
          '--manifest',
          service_manifest_rel,
      ]
      + oci_image_args
      + [
          '--default_config',
          os.path.join(
              output_dir_rel, f'{base_image_tag}_default_config.textproto'
          ),
          '--file_descriptor_set',
          service_desc_rel,
          '--output',
          os.path.join(output_dir_rel, f'{base_image_tag}.bundle.tar'),
      ]
  )
  run_cmd(bundle_cmd)

  # 6. Build and bundle skill image(s)
  all_prune_patterns = []
  for tgt in (real_target, sim_target):
    t_prune = tgt.get('prune') or {}
    if isinstance(t_prune, dict):
      for pattern in t_prune.get('patterns', []):
        if pattern not in all_prune_patterns:
          all_prune_patterns.append(pattern)
  prune_patterns_str = ' '.join(all_prune_patterns)

  skills_dir = os.path.join(output_dir, 'skills')
  generated_skills = []
  for interface in config.get('interfaces', []):
    if interface.get('internal_interface') in (
        'ros_service',
        'ros_topic_subscription',
        'ros_action_client',
    ):
      stem = sanitize_skill_stem(interface.get('external_name'))
      generated_skills.append(f'{stem}_skill')

  if generated_skills:
    print(f'BUILDER: Found {len(generated_skills)} skills to build.')
    for skill in generated_skills:
      print(f'BUILDER: --- Building Skill: {skill} ---')
      skill_dir_rel = os.path.relpath(
          os.path.join(skills_dir, skill), workspace_dir
      )
      skill_manifest_rel = os.path.join(
          skill_dir_rel, f'{skill}.manifest.textproto'
      )
      skill_desc_rel = os.path.join(skill_dir_rel, f'{skill}_protos.desc')

      skill_build_cmd = ['buildah', 'bud']
      if no_cache:
        skill_build_cmd.append('--no-cache')
      skill_build_cmd.extend([
          '--ignorefile',
          ignorefile_rel,
          '--format',
          'docker',
          '-t',
          f'{skill}:latest',
          '--build-arg',
          f'OUTPUT_DIR={output_dir_rel}',
          '--build-arg',
          f'SKILL_NAME={skill}',
      ])
      if prune_patterns_str:
        skill_build_cmd.extend(
            ['--build-arg', f'PRUNE_PATTERNS={prune_patterns_str}']
        )
      skill_build_cmd.extend(['-f', dockerfile_skill_rel, '.'])
      run_cmd(skill_build_cmd)

      print(f'BUILDER: Extracting descriptor set from {skill} image...')
      abs_skill_desc = os.path.abspath(
          os.path.join(workspace_dir, skill_desc_rel)
      )
      os.makedirs(os.path.dirname(abs_skill_desc), exist_ok=True)
      abs_intrinsic_desc = os.path.abspath(
          os.path.join(output_dir, 'intrinsic_proto.desc')
      )

      bash_cmd = (
          f'ctr=$(buildah from {skill}:latest) && mnt=$(buildah mount $ctr) &&'
          f' cp $mnt/opt/{skill}_workspace/install/{skill}/share/{skill}/'
          f'{skill}_protos.desc {abs_skill_desc} && found_desc=$(find'
          f" $mnt/opt/{skill}_workspace/install -name 'intrinsic_proto.desc'"
          ' -print -quit) && if [ -n "$found_desc" ]; then cp "$found_desc"'
          f' {abs_intrinsic_desc}; fi && buildah unmount $ctr && buildah rm'
          ' $ctr'
      )
      run_cmd(['buildah', 'unshare', 'bash', '-c', bash_cmd])

      skill_tar_path = os.path.join(skills_dir, skill, f'{skill}.tar')
      if os.path.exists(skill_tar_path):
        print(f'BUILDER: Overwriting existing tar {skill_tar_path}')
        os.remove(skill_tar_path)
      run_cmd([
          'buildah',
          'push',
          f'{skill}:latest',
          f'docker-archive:{skill_tar_path}',
      ])

      augmented_manifest_path = os.path.join(
          skills_dir, skill, f'{skill}_augmented_manifest.pbbin'
      )
      augmented_desc_path = os.path.join(
          skills_dir, skill, f'{skill}_augmented_protos.desc'
      )

      intrinsic_desc = get_intrinsic_proto_desc_path(workspace_dir)
      if not intrinsic_desc:
        print(
            "Error: Could not locate 'intrinsic_proto.desc' required for"
            ' platform interfaces.'
        )
        sys.exit(1)

      intrinsic_desc_rel = os.path.relpath(intrinsic_desc, workspace_dir)
      desc_sets_str = f'{skill_desc_rel},{intrinsic_desc_rel}'

      help_res = subprocess.run(
          [inbuild_bin, 'skill', 'bundle', '--help'],
          stdout=subprocess.PIPE,
          stderr=subprocess.STDOUT,
          text=True,
          check=False,
      )
      if '--augmented_manifest' in help_res.stdout:
        run_cmd([
            inbuild_bin,
            'skill',
            'manifest',
            '--manifest',
            skill_manifest_rel,
            '--file_descriptor_sets',
            desc_sets_str,
            '--output',
            augmented_manifest_path,
            '--file_descriptor_set_out',
            augmented_desc_path,
        ])
        run_cmd([
            inbuild_bin,
            'skill',
            'bundle',
            '--augmented_manifest',
            augmented_manifest_path,
            '--augmented_file_descriptor_set',
            augmented_desc_path,
            '--oci_image',
            skill_tar_path,
            '--output',
            os.path.join(output_dir_rel, f'{skill}.bundle.tar'),
        ])
      else:
        run_cmd([
            inbuild_bin,
            'skill',
            'bundle',
            '--manifest',
            skill_manifest_rel,
            '--file_descriptor_set',
            skill_desc_rel,
            '--file_descriptor_set',
            intrinsic_desc_rel,
            '--oci_image',
            skill_tar_path,
            '--output',
            os.path.join(output_dir_rel, f'{skill}.bundle.tar'),
        ])

  print('\nBUILDER: Build and bundling complete.')
