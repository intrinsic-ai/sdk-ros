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
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, mock_open, patch

try:
    from intrinsic_sdk_bundle_library_py.colcon_verb import BundleVerb
except ImportError:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from intrinsic_sdk_bundle_library_py.colcon_verb import BundleVerb


class TestColconVerb(unittest.TestCase):

    def test_parse_manifest_skill_cpp(self):
        verb = BundleVerb()
        content = """id {
          package: "com.example"
          name: "test_cpp_skill"
        }
        options {
          cc_config {
            create_skill: "::TestCppSkill::CreateSkill"
          }
        }"""
        info = verb._parse_manifest(content)
        self.assertFalse(info['is_service'])
        self.assertEqual(info['name'], 'test_cpp_skill')
        self.assertEqual(info['package_org'], 'com.example')
        self.assertEqual(info['skill_type'], 'cpp')

    def test_parse_manifest_skill_python(self):
        verb = BundleVerb()
        content = """id {
          package: "com.example"
          name: "test_python_skill"
        }
        options {
          python_config {
            skill_module: "test_python_skill"
          }
        }"""
        info = verb._parse_manifest(content)
        self.assertFalse(info['is_service'])
        self.assertEqual(info['name'], 'test_python_skill')
        self.assertEqual(info['package_org'], 'com.example')
        self.assertEqual(info['skill_type'], 'python')

    def test_parse_manifest_service(self):
        verb = BundleVerb()
        content = """metadata {
          id {
            package: "ai.intrinsic"
            name: "test_cpp_service"
          }
        }
        service_def {
          real_spec {}
        }"""
        info = verb._parse_manifest(content)
        self.assertTrue(info['is_service'])
        self.assertEqual(info['name'], 'test_cpp_service')
        self.assertEqual(info['package_org'], 'ai.intrinsic')
        self.assertIsNone(info['skill_type'])

    @patch('intrinsic_sdk_bundle_library_py.colcon_verb.build_bundle')
    @patch('intrinsic_sdk_bundle_library_py.colcon_verb.build_container')
    @patch('intrinsic_sdk_bundle_library_py.colcon_verb.get_packages')
    @patch('builtins.open', new_callable=mock_open, read_data=(
        'id { package: "com.example" name: "my_skill" }\n'
        'options { cc_config {} }'
    ))
    @patch('pathlib.Path.glob')
    def test_main_with_skill(
        self, mock_glob, mock_open_file, mock_get_packages,
        mock_build_container, mock_build_bundle
    ):
        verb = BundleVerb()

        # Mock colcon packages
        mock_descriptor = MagicMock()
        mock_descriptor.name = 'my_skill_pkg'
        mock_descriptor.path = '/ws/src/my_skill_pkg'

        mock_decorator = MagicMock()
        mock_decorator.descriptor = mock_descriptor
        mock_decorator.selected = True

        mock_get_packages.return_value = [mock_decorator]

        # Mock glob to find manifest
        mock_glob.return_value = [
            Path('/ws/src/my_skill_pkg/my_skill_pkg.manifest.textproto')
        ]

        # Mock context and args
        mock_context = MagicMock()
        mock_context.args = argparse.Namespace(
            ros_distro='jazzy',
            bundle_dir='./intrinsic_asset_bundles',
            builder_name='container-builder',
            no_cache=False,
            keep_builder=False,
            default_config=None
        )

        result = verb.main(context=mock_context)
        self.assertEqual(result, 0)

        # Verify build_container call
        mock_build_container.assert_called_once()
        container_args = mock_build_container.call_args[0][0]
        self.assertEqual(container_args.skill_name, 'my_skill')
        self.assertEqual(container_args.skill_package, 'my_skill_pkg')
        self.assertEqual(container_args.skill_type, 'cpp')
        self.assertEqual(container_args.skill_asset_id_org, 'com.example')
        self.assertIsNone(container_args.service_name)

        # Verify build_bundle call
        mock_build_bundle.assert_called_once()
        bundle_args = mock_build_bundle.call_args[0][0]
        self.assertEqual(bundle_args.skill_name, 'my_skill')
        self.assertEqual(bundle_args.skill_package, 'my_skill_pkg')
        self.assertEqual(
            bundle_args.manifest_path,
            str(Path('/ws/src/my_skill_pkg/my_skill_pkg.manifest.textproto').resolve())
        )


if __name__ == '__main__':
    unittest.main()
