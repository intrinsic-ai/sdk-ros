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
from pathlib import Path
import re

from colcon_core.package_selection import add_arguments as add_package_selection_arguments
from colcon_core.package_selection import get_packages
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint

from intrinsic_sdk_bundle_library_py.build import (
    add_common_argument,
    build_bundle,
    build_container,
)


class BundleVerb(VerbExtensionPoint):
    """Bundle skills or services from a ROS workspace."""

    def __init__(self):
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):
        # Register package selection arguments (e.g. --packages-select)
        add_package_selection_arguments(parser)

        # Verb-specific arguments
        add_common_argument(parser, 'ros_distro')
        add_common_argument(parser, 'bundle_dir')
        add_common_argument(parser, 'builder_name')
        add_common_argument(parser, 'no_cache')
        add_common_argument(parser, 'keep_builder')
        add_common_argument(parser, 'default_config')

    def main(self, *, context):
        args = context.args

        # Get all packages and filter selected
        decorators = get_packages(args)
        selected_decorators = [d for d in decorators if d.selected]

        if not selected_decorators:
            print('No packages selected or found in the workspace.')
            return 0

        # Scan for packages with manifests
        bundlable_packages = []
        for decorator in selected_decorators:
            pkg_path = Path(decorator.descriptor.path)
            # Find manifest file
            manifests = list(pkg_path.glob('*.manifest.textproto'))
            if not manifests:
                continue

            if len(manifests) > 1:
                print(
                    f'Warning: Multiple manifest files found in {decorator.descriptor.name}, '
                    f'using the first one: {manifests[0]}'
                )

            manifest_path = manifests[0]

            # Parse manifest
            try:
                with open(manifest_path, 'r') as f:
                    content = f.read()
            except Exception as e:
                print(f'Error reading manifest {manifest_path}: {e}')
                continue

            info = self._parse_manifest(content)
            info['manifest_path'] = str(manifest_path.resolve())
            info['package_name'] = decorator.descriptor.name

            bundlable_packages.append(info)

        if not bundlable_packages:
            print('No packages with a *.manifest.textproto file were selected.')
            return 0

        print(f'Found {len(bundlable_packages)} package(s) to bundle:')
        for pkg in bundlable_packages:
            skill_type = pkg['skill_type']
            type_str = 'service' if pkg['is_service'] else f'skill ({skill_type})'
            pkg_name = pkg['package_name']
            print(f'  - {pkg_name} ({type_str})')

        # Now build each package
        for pkg in bundlable_packages:
            pkg_name = pkg['package_name']
            print(f'\n--- Processing {pkg_name} ---')

            # Construct arguments for build_container
            container_args = argparse.Namespace(
                command='container',
                images_dir=args.bundle_dir,
                builder_name=args.builder_name,
                ros_distro=args.ros_distro,
                no_cache=args.no_cache,
                keep_builder=args.keep_builder,
                dockerfile=None,
                dependencies=None,
                source_dir=None,
                overlay_source=None,

                # Skill specific
                skill_name=None if pkg['is_service'] else pkg['name'],
                skill_package=None if pkg['is_service'] else pkg['package_name'],
                skill_executable=None,  # Will default in build.py
                skill_config=None,      # Will default in build.py
                skill_asset_id_org=pkg['package_org'] if not pkg['is_service'] else None,
                skill_type=pkg['skill_type'] if not pkg['is_service'] else 'cpp',

                # Service specific
                service_name=pkg['name'] if pkg['is_service'] else None,
                service_package=pkg['package_name'] if pkg['is_service'] else None
            )

            # Build container
            try:
                build_container(container_args)
            except Exception as e:
                print(f'Error building container for {pkg_name}: {e}')
                return 1

            # Construct arguments for build_bundle
            bundle_args = argparse.Namespace(
                command='bundle',
                images_dir=args.bundle_dir,
                builder_name=args.builder_name,  # Ignored

                skill_name=None if pkg['is_service'] else pkg['name'],
                skill_package=None if pkg['is_service'] else pkg['package_name'],
                service_name=pkg['name'] if pkg['is_service'] else None,
                service_package=pkg['package_name'] if pkg['is_service'] else None,

                manifest_path=pkg['manifest_path'],
                default_config=args.default_config
            )

            # Build bundle
            try:
                build_bundle(bundle_args)
            except Exception as e:
                print(f'Error building bundle for {pkg_name}: {e}')
                return 1

        print('\nAll bundles built successfully!')
        return 0

    def _parse_manifest(self, content):
        is_service = 'service_def' in content

        match_name = re.search(r'id\s*\{\s*[^^{}]*name:\s*"([^"]+)"', content, re.DOTALL)
        name = match_name.group(1) if match_name else None

        match_pkg = re.search(r'id\s*\{\s*[^^{}]*package:\s*"([^"]+)"', content, re.DOTALL)
        package_org = match_pkg.group(1) if match_pkg else None

        skill_type = None
        if not is_service:
            if 'python_config' in content:
                skill_type = 'python'
            else:
                # Default to cpp if cc_config or not specified
                skill_type = 'cpp'

        return {
            'is_service': is_service,
            'name': name,
            'package_org': package_org,
            'skill_type': skill_type
        }
