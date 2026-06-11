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
import sys
import unittest

try:
    from intrinsic_sdk_bundle_library_py.build import add_common_argument, COMMON_ARGUMENTS
except ImportError:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from intrinsic_sdk_bundle_library_py.build import add_common_argument, COMMON_ARGUMENTS


class TestArguments(unittest.TestCase):

    def test_common_arguments_keys(self):
        expected_keys = {
            'ros_distro',
            'bundle_dir',
            'builder_name',
            'no_cache',
            'keep_builder',
            'default_config',
        }
        self.assertEqual(set(COMMON_ARGUMENTS.keys()), expected_keys)

    def test_add_common_argument_ros_distro(self):
        parser = argparse.ArgumentParser()
        add_common_argument(parser, 'ros_distro')

        # Test default
        args = parser.parse_args([])
        self.assertEqual(args.ros_distro, 'jazzy')

        # Test dash flag
        args = parser.parse_args(['--ros-distro', 'rolling'])
        self.assertEqual(args.ros_distro, 'rolling')

        # Test underscore flag
        args = parser.parse_args(['--ros_distro', 'iron'])
        self.assertEqual(args.ros_distro, 'iron')

    def test_add_common_argument_bundle_dir(self):
        parser = argparse.ArgumentParser()
        add_common_argument(parser, 'bundle_dir')

        # Test default
        args = parser.parse_args([])
        self.assertEqual(args.bundle_dir, './intrinsic_asset_bundles')

        # Test bundle dash flag
        args = parser.parse_args(['--bundle-dir', '/tmp/foo'])
        self.assertEqual(args.bundle_dir, '/tmp/foo')

        # Test bundle underscore flag
        args = parser.parse_args(['--bundle_dir', '/tmp/bar'])
        self.assertEqual(args.bundle_dir, '/tmp/bar')

        # Test legacy images dash flag
        args = parser.parse_args(['--images-dir', '/tmp/baz'])
        self.assertEqual(args.bundle_dir, '/tmp/baz')

        # Test legacy images underscore flag
        args = parser.parse_args(['--images_dir', '/tmp/qux'])
        self.assertEqual(args.bundle_dir, '/tmp/qux')

    def test_add_common_argument_builder_name(self):
        parser = argparse.ArgumentParser()
        add_common_argument(parser, 'builder_name')

        # Test default
        args = parser.parse_args([])
        self.assertEqual(args.builder_name, 'container-builder')

        # Test dash flag
        args = parser.parse_args(['--builder-name', 'custom-builder'])
        self.assertEqual(args.builder_name, 'custom-builder')

        # Test underscore flag
        args = parser.parse_args(['--builder_name', 'another-builder'])
        self.assertEqual(args.builder_name, 'another-builder')

    def test_add_common_argument_no_cache(self):
        parser = argparse.ArgumentParser()
        add_common_argument(parser, 'no_cache')

        # Test default
        args = parser.parse_args([])
        self.assertFalse(args.no_cache)

        # Test flag
        args = parser.parse_args(['--no-cache'])
        self.assertTrue(args.no_cache)

    def test_add_common_argument_keep_builder(self):
        parser = argparse.ArgumentParser()
        add_common_argument(parser, 'keep_builder')

        # Test default
        args = parser.parse_args([])
        self.assertFalse(args.keep_builder)

        # Test flag
        args = parser.parse_args(['--keep-builder'])
        self.assertTrue(args.keep_builder)

    def test_add_common_argument_default_config(self):
        parser = argparse.ArgumentParser()
        add_common_argument(parser, 'default_config')

        # Test default
        args = parser.parse_args([])
        self.assertIsNone(args.default_config)

        # Test dash flag
        args = parser.parse_args(['--default-config', 'config.pbtxt'])
        self.assertEqual(args.default_config, 'config.pbtxt')

        # Test underscore flag
        args = parser.parse_args(['--default_config', 'another.pbtxt'])
        self.assertEqual(args.default_config, 'another.pbtxt')

    def test_kwargs_overrides(self):
        parser = argparse.ArgumentParser()
        add_common_argument(parser, 'builder_name', help='Custom help message')

        # Verify parsing still works
        args = parser.parse_args(['--builder-name', 'my-builder'])
        self.assertEqual(args.builder_name, 'my-builder')


if __name__ == '__main__':
    unittest.main()
