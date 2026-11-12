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

"""Workspace discovery, packaged resource paths, and SDK tool resolution."""

import json
import os
import shutil
import stat
import subprocess
import sys
import urllib.request

DEFAULT_SDK_VERSION = 'v1.36.20260831'

_PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
_EXCLUDED_WORKSPACE_DIRS = frozenset({
    '/',
    '/opt',
    '/tmp',
    '/usr',
    '/usr/local',
    '/var',
})


def get_resource_path(name):
  """Return the absolute path to a packaged container resource file."""
  path = os.path.join(_PACKAGE_DIR, 'resources', name)
  if not os.path.isfile(path):
    raise FileNotFoundError(f'Packaged resource not found: {path}')
  return path


def get_template_path(name):
  """Return the absolute path to a packaged code generation template."""
  path = os.path.join(_PACKAGE_DIR, 'templates', name)
  if not os.path.isfile(path):
    raise FileNotFoundError(f'Packaged template not found: {path}')
  return path


def render_template(template_name, replacements):
  """Render a @TOKEN@-parameterized template from aquarium/templates/."""
  template_path = get_template_path(template_name)
  with open(template_path, 'r', encoding='utf-8') as f:
    content = f.read()
  for key, value in replacements.items():
    content = content.replace(f'@{key}@', str(value))
  return content


def _has_ros_package_under_src(src_dir, max_depth=4):
  """Return True if src_dir contains at least one package.xml within depth."""
  if not os.path.isdir(src_dir):
    return False
  src_dir = os.path.abspath(src_dir)
  base_depth = src_dir.rstrip(os.sep).count(os.sep)
  for root, dirs, files in os.walk(src_dir):
    if 'package.xml' in files:
      return True
    cur_depth = root.rstrip(os.sep).count(os.sep) - base_depth
    if cur_depth >= max_depth:
      dirs[:] = []
  return False


def find_workspace_dir(start_dir=None):
  """Locate the enclosing colcon workspace root containing a ROS src/ tree."""
  cur = os.path.abspath(start_dir or os.getcwd())
  while True:
    if cur not in _EXCLUDED_WORKSPACE_DIRS:
      src_dir = os.path.join(cur, 'src')
      if _has_ros_package_under_src(src_dir):
        return cur
    parent = os.path.dirname(cur)
    if parent == cur:
      break
    cur = parent
  return None


def find_sdk_ros_dir(workspace_dir):
  """Return workspace-relative path to the sdk-ros directory under src/."""
  src_dir = os.path.join(workspace_dir, 'src')
  if os.path.isdir(src_dir):
    default_candidate = os.path.join(
        src_dir, 'sdk-ros', 'intrinsic_sdk_cmake', 'package.xml'
    )
    if os.path.isfile(default_candidate):
      return 'src/sdk-ros'
    for root, _, files in os.walk(src_dir):
      if 'package.xml' in files:
        pkg_xml_path = os.path.join(root, 'package.xml')
        try:
          with open(pkg_xml_path, 'r', encoding='utf-8') as f:
            if '<name>intrinsic_sdk_cmake</name>' in f.read():
              sdk_root = os.path.dirname(root)
              return os.path.relpath(sdk_root, workspace_dir).replace(
                  os.sep, '/'
              )
        except OSError:
          continue
  return 'src/sdk-ros'


def _read_sdk_version_json(json_path):
  """Read sdk_version string from a sdk_version.json file if valid."""
  if not os.path.isfile(json_path):
    return None
  try:
    with open(json_path, 'r', encoding='utf-8') as f:
      data = json.load(f)
    version = data.get('sdk_version')
    if isinstance(version, str) and version.strip():
      return version.strip()
  except (OSError, ValueError, TypeError):
    pass
  return None


def get_sdk_version(workspace_dir=None):
  """Resolve the pinned Intrinsic SDK version from sdk_version.json."""
  if not workspace_dir:
    workspace_dir = find_workspace_dir()

  candidates = []
  if workspace_dir:
    sdk_ros_rel = find_sdk_ros_dir(workspace_dir)
    candidates.append(
        os.path.join(
            workspace_dir,
            sdk_ros_rel,
            'intrinsic_sdk_bundle_library_py',
            'resource',
            'sdk_version.json',
        )
    )
    candidates.append(
        os.path.join(
            workspace_dir,
            'install',
            'intrinsic_sdk_bundle_library_py',
            'share',
            'intrinsic_sdk_bundle_library_py',
            'resource',
            'sdk_version.json',
        )
    )
    candidates.append(
        os.path.join(
            workspace_dir,
            'install',
            'share',
            'intrinsic_sdk_bundle_library_py',
            'resource',
            'sdk_version.json',
        )
    )

  # Sibling package path when aquarium resides inside sdk-ros/aquarium.
  candidates.append(
      os.path.abspath(
          os.path.join(
              _PACKAGE_DIR,
              '..',
              '..',
              'intrinsic_sdk_bundle_library_py',
              'resource',
              'sdk_version.json',
          )
      )
  )

  for json_path in candidates:
    version = _read_sdk_version_json(json_path)
    if version:
      return version

  if workspace_dir:
    src_dir = os.path.join(workspace_dir, 'src')
    if os.path.isdir(src_dir):
      for root, _, files in os.walk(src_dir):
        if 'sdk_version.json' in files:
          version = _read_sdk_version_json(
              os.path.join(root, 'sdk_version.json')
          )
          if version:
            return version

  return DEFAULT_SDK_VERSION


def get_inbuild_path(workspace_dir=None):
  """Locate or download the Intrinsic SDK inbuild CLI binary."""
  inbuild_in_path = shutil.which('inbuild')
  if inbuild_in_path:
    return inbuild_in_path

  if not workspace_dir:
    workspace_dir = find_workspace_dir() or os.getcwd()

  candidates = [
      os.path.join(
          workspace_dir, 'install', 'intrinsic_sdk_cmake', 'bin', 'inbuild'
      ),
  ]
  if sys.prefix != sys.base_prefix:
    candidates.append(os.path.join(sys.prefix, 'bin', 'inbuild'))
  candidates.append(
      os.path.join(
          workspace_dir, 'install', 'aquarium', 'lib', 'aquarium', 'inbuild'
      )
  )

  for local_path in candidates:
    if os.path.exists(local_path):
      return local_path

  local_path = candidates[0]
  version = get_sdk_version(workspace_dir)
  print(f"BUILDER: 'inbuild' not found. Downloading {version} to {local_path}.")
  os.makedirs(os.path.dirname(local_path), exist_ok=True)

  url = (
      'https://github.com/intrinsic-ai/sdk/releases/download/'
      f'{version}/inbuild-linux-amd64'
  )
  try:
    urllib.request.urlretrieve(url, local_path)
    st = os.stat(local_path)
    os.chmod(local_path, st.st_mode | stat.S_IEXEC)
    print("BUILDER: 'inbuild' downloaded successfully.")
    return local_path
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f"Error: Failed to download 'inbuild' from {url}: {e}")
    sys.exit(1)


def get_intrinsic_proto_desc_path(workspace_dir=None):
  """Locate or extract intrinsic_proto.desc from workspace or underlay image."""
  if not workspace_dir:
    workspace_dir = find_workspace_dir() or os.getcwd()

  candidates = [
      os.path.join(
          workspace_dir, 'build', 'aquarium_images', 'intrinsic_proto.desc'
      ),
      os.path.join(
          workspace_dir,
          'install',
          'intrinsic_sdk_cmake',
          'share',
          'intrinsic_sdk_cmake',
          'intrinsic_proto.desc',
      ),
      os.path.join(
          workspace_dir,
          'install',
          'share',
          'intrinsic_sdk_cmake',
          'intrinsic_proto.desc',
      ),
      os.path.join(
          workspace_dir, 'build', 'intrinsic_sdk_cmake', 'intrinsic_proto.desc'
      ),
  ]
  for candidate in candidates:
    if os.path.exists(candidate):
      return candidate

  extracted_dest = candidates[0]
  os.makedirs(os.path.dirname(extracted_dest), exist_ok=True)
  extract_cmd = (
      'ctr=$(buildah from aquarium-sdk-underlay:latest 2>/dev/null) && if [ -n'
      ' "$ctr" ]; then mnt=$(buildah mount $ctr 2>/dev/null) &&'
      ' found_desc=$(find $mnt/workspace/install -name'
      " 'intrinsic_proto.desc' -print -quit 2>/dev/null) && if [ -n"
      f' "$found_desc" ]; then cp "$found_desc" \'{extracted_dest}\'; fi &&'
      ' buildah unmount $ctr 2>/dev/null; buildah rm $ctr 2>/dev/null; fi'
  )
  try:
    subprocess.run(
        ['buildah', 'unshare', 'bash', '-c', extract_cmd],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if os.path.exists(extracted_dest):
      return extracted_dest
  except OSError:
    pass

  return None


def get_runtime_context_proto_path(workspace_dir=None):
  """Locate or download runtime_context.proto matching sdk_version.json."""
  if not workspace_dir:
    workspace_dir = find_workspace_dir() or os.getcwd()

  sdk_clone_path = os.path.join(
      workspace_dir,
      'src',
      'sdk',
      'intrinsic',
      'resources',
      'proto',
      'runtime_context.proto',
  )
  if os.path.exists(sdk_clone_path):
    return sdk_clone_path

  fetched_sdk_path = os.path.join(
      workspace_dir,
      'build',
      'intrinsic_sdk_cmake',
      '_deps',
      'intrinsic_sdk-src',
      'intrinsic',
      'resources',
      'proto',
      'runtime_context.proto',
  )
  if os.path.exists(fetched_sdk_path):
    return fetched_sdk_path

  if sys.prefix != sys.base_prefix:
    local_path = os.path.join(
        sys.prefix, 'share', 'aquarium', 'runtime_context.proto'
    )
  else:
    local_path = os.path.join(
        workspace_dir,
        'install',
        'aquarium',
        'share',
        'aquarium',
        'runtime_context.proto',
    )

  if os.path.exists(local_path):
    return local_path

  print("GENERATOR: 'runtime_context.proto' not found. Downloading...")
  os.makedirs(os.path.dirname(local_path), exist_ok=True)

  version = get_sdk_version(workspace_dir)
  url = (
      f'https://raw.githubusercontent.com/intrinsic-ai/sdk/{version}/'
      'intrinsic/resources/proto/runtime_context.proto'
  )
  try:
    print(f'GENERATOR: Downloading from {url} to {local_path}...')
    urllib.request.urlretrieve(url, local_path)
    print("GENERATOR: 'runtime_context.proto' downloaded successfully.")
    return local_path
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f"Error: Failed to download 'runtime_context.proto' from {url}: {e}")
    sys.exit(1)


def is_package_in_workspace_src(workspace_dir, package_name):
  """Scan workspace src/ for package.xml with <name>package_name</name>."""
  src_dir = os.path.join(workspace_dir, 'src')
  if not os.path.exists(src_dir):
    return False
  for root, _, files in os.walk(src_dir):
    if 'package.xml' in files:
      pkg_xml_path = os.path.join(root, 'package.xml')
      try:
        with open(pkg_xml_path, 'r', encoding='utf-8') as f:
          if f'<name>{package_name}</name>' in f.read():
            return True
      except OSError:
        pass
  return False


def is_package_in_workspace_install(workspace_dir, package_name):
  """Check if package install or share directory exists in workspace/install."""
  install_dir = os.path.join(workspace_dir, 'install')
  if not os.path.exists(install_dir):
    return False
  if os.path.exists(os.path.join(install_dir, package_name)):
    return True
  ament_index_pkg = os.path.join(
      install_dir,
      'share',
      'ament_index',
      'resource_index',
      'packages',
      package_name,
  )
  if os.path.exists(ament_index_pkg):
    return True
  if os.path.exists(os.path.join(install_dir, 'share', package_name)):
    return True
  return False


def is_host_ros_apt_repo_configured():
  """Check if host system APT cache has ROS distribution packages indexed."""
  ros_distro = os.environ.get('ROS_DISTRO', 'lyrical')
  try:
    res = subprocess.run(
        ['apt-cache', 'search', f'^ros-{ros_distro}-'],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=5,
        text=True,
        check=False,
    )
    return bool(res.stdout.strip())
  except (OSError, subprocess.SubprocessError):
    return False


def is_released_apt_package(package_name, workspace_dir=None):
  """Check if an APT package exists for the given ROS package name.

  Returns (found_status, apt_package_name) where found_status is True, False,
  or None (unknown when host package index tools are unavailable).
  """
  del workspace_dir
  ros_distro = os.environ.get('ROS_DISTRO', 'lyrical')
  apt_package_name = f"ros-{ros_distro}-{package_name.replace('_', '-')}"

  if is_host_ros_apt_repo_configured():
    try:
      res = subprocess.run(
          ['apt-cache', 'show', apt_package_name],
          stdin=subprocess.DEVNULL,
          stdout=subprocess.PIPE,
          stderr=subprocess.PIPE,
          timeout=5,
          text=True,
          check=False,
      )
      out = res.stdout + res.stderr
      if res.returncode == 0 and 'Package:' in out:
        return True, apt_package_name
      if 'Unable to locate package' in out or 'No packages found' in out:
        return False, apt_package_name
    except (OSError, subprocess.SubprocessError):
      pass

  try:
    res = subprocess.run(
        ['rosdep', 'resolve', package_name],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=5,
        text=True,
        check=False,
    )
    out = res.stdout + res.stderr
    if res.returncode == 0:
      return True, apt_package_name
    if 'Cannot find' in out or 'No definition' in out:
      return False, apt_package_name
  except (OSError, subprocess.SubprocessError):
    pass

  return None, apt_package_name


def validate_target_package(package_name, workspace_dir):
  """Validate target ROS package and return (is_apt, apt_package_name)."""
  if is_package_in_workspace_src(workspace_dir, package_name):
    print(
        f"BUILDER: Target package '{package_name}' found in workspace 'src/'."
    )
    return False, ''

  if is_package_in_workspace_install(workspace_dir, package_name):
    print(
        f"BUILDER: Target package '{package_name}' found in local workspace"
        ' install directory.'
    )
    return False, ''

  is_released, apt_pkg_name = is_released_apt_package(
      package_name, workspace_dir
  )
  if is_released is True:
    print(
        f"BUILDER: Target package '{package_name}' found as released APT"
        f" package '{apt_pkg_name}'."
    )
    return True, apt_pkg_name
  if is_released is None:
    print(
        f"BUILDER: Package index tool unavailable to verify '{package_name}';"
        f" assuming released APT package '{apt_pkg_name}'."
    )
    return True, apt_pkg_name

  print(
      f"BUILDER ERROR: Target ROS package '{package_name}' is neither present"
      f" in local workspace ('{workspace_dir}') nor available as a released ROS"
      f" package ('{apt_pkg_name}')."
  )
  print('Aborting build before image creation.')
  sys.exit(1)
