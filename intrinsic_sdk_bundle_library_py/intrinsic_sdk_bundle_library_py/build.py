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

import argparse
import json
import os
import platform
import subprocess
import sys
import urllib.request


def run_command(cmd, check=True):
    print(f"Running: {' '.join(cmd)}")
    return subprocess.run(cmd, check=check)


def get_sdk_version():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    version_file = os.path.join(os.path.dirname(current_dir), 'resource', 'sdk_version.json')

    try:
        with open(version_file, 'r') as f:
            data = json.load(f)
            return data.get('sdk_version')
    except Exception as e:
        print(f'Error reading SDK version from {version_file}: {e}')
        return None


def download_inbuild(sdk_version, dest_dir='.'):
    system = platform.system().lower()
    machine = platform.machine().lower()

    # Map machine to expected URL format
    if machine in ['x86_64', 'amd64']:
        arch = 'amd64'
    elif machine in ['aarch64', 'arm64']:
        arch = 'arm64'
    else:
        arch = machine  # fallback

    # Handle Windows extension
    ext = '.exe' if system == 'windows' else ''

    # Construct filename
    binary_name = f'inbuild-{system}-{arch}{ext}'

    inbuild_path = os.path.join(dest_dir, 'inbuild' + ext)

    if not os.path.exists(inbuild_path):
        url = f'https://github.com/intrinsic-ai/sdk/releases/download/{sdk_version}/{binary_name}'
        print(f'Downloading inbuild from {url} to {inbuild_path}...')
        try:
            urllib.request.urlretrieve(url, inbuild_path)
            os.chmod(inbuild_path, 0o755)
        except Exception as e:
            print(f'Failed to download inbuild from {url}: {e}')
            print(
                f'This could be because a binary for your system ({system}) '
                f'and architecture ({arch}) is not available in the releases.'
            )
            sys.exit(1)
    else:
        print(f'inbuild already exists at {inbuild_path}')
    return inbuild_path


def build_container(args):
    print('Building container...')
    tag = None
    dockerfile = None
    name = None
    package = None

    if args.service_name and args.service_package:
        name = args.service_name
        package = args.service_package
        dockerfile = args.dockerfile or os.path.join(
            os.path.dirname(__file__), '..', 'resource', 'service.Dockerfile'
        )
    elif args.skill_name and args.skill_package:
        name = args.skill_name
        package = args.skill_package
        dockerfile = args.dockerfile or os.path.join(
            os.path.dirname(__file__), '..', 'resource', 'skill.Dockerfile'
        )
    else:
        print('Error: Must specify either service or skill name and package.')
        return

    tag = f'{package}:{name}'
    images_dir = args.images_dir or './images'

    # Ensure .dockerignore exists to avoid sending large directories to build context
    dockerignore_path = '.dockerignore'
    if not os.path.exists(dockerignore_path):
        print(f'Creating {dockerignore_path}...')
        try:
            with open(dockerignore_path, 'w') as f:
                f.write('images\nbuild\nlog\ninstall\n')
        except Exception as e:
            print(f'Warning: Could not create {dockerignore_path}: {e}')

    # Ensure builder exists
    builder_name = args.builder_name or 'container-builder'
    try:
        run_command(['docker', 'buildx', 'inspect', '--builder', builder_name])
    except Exception:
        print(f"Builder {builder_name} not found. Creating it...")
        run_command(['docker', 'buildx', 'create', '--name', builder_name, '--driver', 'docker-container'])
        run_command(['docker', 'buildx', 'use', builder_name])

    tar_dir = os.path.join(images_dir, name)
    os.makedirs(tar_dir, exist_ok=True)
    tar_path = os.path.join(tar_dir, f'{name}.tar')

    # Docker buildx build
    cmd = ['docker', 'buildx', 'build', '-t', tag, '-f', dockerfile, '--builder', builder_name]
    if args.no_cache:
        cmd.append('--no-cache')

    # Output flag
    output_arg = (
        f"type=docker,"
        f"dest={tar_path},"
        f"compression=zstd,"
        f"push=false,"
        f"name={tag}"
    )
    cmd.extend(['--output', output_arg])

    # Build args
    cmd.extend(['--build-arg', f'ROS_DISTRO={args.ros_distro}'])
    cmd.extend(['--build-arg', f'SKILL_TYPE={args.skill_type}'])
    if args.service_name:
        cmd.extend([
            '--build-arg', f'SERVICE_PACKAGE={package}',
            '--build-arg', f'SERVICE_NAME={name}',
            '--build-arg', f'SERVICE_EXECUTABLE_NAME={name}_main'
        ])
    else:
        cmd.extend([
            '--build-arg', f'SKILL_PACKAGE={package}',
            '--build-arg', f'SKILL_NAME={name}',
        ])
        skill_executable = args.skill_executable or f'lib/{package}/{name}_main'
        cmd.extend(['--build-arg', f'SKILL_EXECUTABLE={skill_executable}'])

        skill_config = args.skill_config or f'share/{package}/{name}_config.pbbin'
        cmd.extend(['--build-arg', f'SKILL_CONFIG={skill_config}'])

        if args.skill_asset_id_org:
            cmd.extend(['--build-arg', f'SKILL_ASSET_ID_ORG={args.skill_asset_id_org}'])

    if args.dependencies:
        cmd.extend(['--build-arg', f'DEPENDENCIES={args.dependencies}'])

    cmd.append('.')

    run_command(cmd)
    print(f'Saved compressed image to {tar_path}')
    
    print(f'Stopping builder {builder_name}...')
    run_command(['docker', 'buildx', 'stop', builder_name])


def build_bundle(args):
    print('Building bundle...')
    name = args.service_name or args.skill_name
    package = args.service_package or args.skill_package

    if not name or not package:
        print('Error: Must specify name and package.')
        return

    if not args.manifest_path:
        print('Error: Must specify --manifest_path.')
        return

    images_dir = args.images_dir or './images'
    tar_path = os.path.join(images_dir, name, f'{name}.tar')

    if not os.path.exists(tar_path):
        print(f'Error: Image tar not found at {tar_path}. Run build-container first.')
        return

    # Load image (Podman)
    run_command(['podman', 'load', '-i', tar_path])

    # Extract descriptor
    container_name = f'temp_container_{name}'
    run_command(['podman', 'create', '--replace', '--name', container_name, f'{package}:{name}'])

    desc_path = os.path.join(images_dir, name, f'{name}_protos.desc')
    if args.service_name:
        src_path = f'/opt/ros/overlay/install/share/{package}/{name}_protos.desc'
    else:
        src_path = f'/opt/{name}_workspace/install/share/{package}/{name}_protos.desc'

    run_command(['podman', 'cp', f'{container_name}:{src_path}', desc_path])
    run_command(['podman', 'rm', '-f', container_name])

    # Get SDK version and download inbuild
    sdk_version = get_sdk_version()
    if not sdk_version:
        print('Error: Could not determine SDK version.')
        return

    inbuild_path = download_inbuild(sdk_version)

    # Build bundle
    inbuild_cmd = [inbuild_path]
    if args.service_name:
        inbuild_cmd.extend(['service', 'bundle'])
    else:
        inbuild_cmd.extend(['skill', 'bundle'])

    inbuild_cmd.extend([
        '--file_descriptor_set', desc_path,
        '--manifest', args.manifest_path,
        '--oci_image', tar_path,
        '--output', os.path.join(images_dir, name, f'{name}.bundle.tar')
    ])

    if args.default_config:
        inbuild_cmd.extend(['--default_config', args.default_config])

    run_command(inbuild_cmd)

    bundle_path = os.path.join(images_dir, name, f'{name}.bundle.tar')



def main():
    parser = argparse.ArgumentParser(description='Build container and bundle for skills/services.')
    subparsers = parser.add_subparsers(dest='command', help='Command to run')

    # Build container parser
    parser_container = subparsers.add_parser('container', help='Build container')
    parser_container.add_argument('--images_dir', default='./images')
    parser_container.add_argument('--builder_name', default='container-builder')
    parser_container.add_argument('--service_name')
    parser_container.add_argument('--service_package')
    parser_container.add_argument('--skill_name')
    parser_container.add_argument('--skill_package')
    parser_container.add_argument('--dockerfile')
    parser_container.add_argument('--dependencies')
    parser_container.add_argument('--ros_distro', default='jazzy')
    parser_container.add_argument('--skill_executable')
    parser_container.add_argument('--skill_config')
    parser_container.add_argument('--skill_asset_id_org')
    parser_container.add_argument('--skill_type', choices=['cpp', 'python'], default='cpp')
    parser_container.add_argument('--no-cache', action='store_true', help='Do not use cache when building the image')

    # Build bundle parser
    parser_bundle = subparsers.add_parser('bundle', help='Build bundle')
    parser_bundle.add_argument('--images_dir', default='./images')
    parser_bundle.add_argument('--service_name')
    parser_bundle.add_argument('--service_package')
    parser_bundle.add_argument('--skill_name')
    parser_bundle.add_argument('--skill_package')
    parser_bundle.add_argument('--manifest_path', required=True)
    parser_bundle.add_argument('--default_config')

    args = parser.parse_args()

    if args.command == 'container':
        build_container(args)
    elif args.command == 'bundle':
        build_bundle(args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
