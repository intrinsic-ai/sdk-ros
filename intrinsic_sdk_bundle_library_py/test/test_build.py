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

import os
import sys
import unittest
from unittest.mock import MagicMock, mock_open, patch

try:
    from intrinsic_sdk_bundle_library_py import build
except ImportError:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from intrinsic_sdk_bundle_library_py import build


class TestBuild(unittest.TestCase):

    @patch(
        'intrinsic_sdk_bundle_library_py.build.get_package_share_directory',
        return_value='/dummy/share/path'
    )
    @patch('builtins.open', new_callable=mock_open, read_data='{"sdk_version": "v0.1.0"}')
    def test_get_sdk_version(self, mock_file, mock_get_share):
        version = build.get_sdk_version()
        self.assertEqual(version, 'v0.1.0')

    @patch(
        'intrinsic_sdk_bundle_library_py.build.get_package_share_directory',
        return_value='/dummy/share/path'
    )
    @patch('builtins.open', side_effect=Exception('File not found'))
    def test_get_sdk_version_error(self, mock_file, mock_get_share):
        version = build.get_sdk_version()
        self.assertIsNone(version)

    @patch('urllib.request.urlopen')
    @patch('urllib.request.urlretrieve')
    @patch('os.chmod')
    @patch('os.path.exists', return_value=False)
    @patch('platform.system', return_value='Linux')
    @patch('platform.machine', return_value='x86_64')
    def test_download_inbuild(
        self, mock_machine, mock_system, mock_exists, mock_chmod, mock_urlretrieve, mock_urlopen
    ):
        mock_urlopen.return_value.__enter__.return_value = MagicMock()
        path = build.download_inbuild('v0.1.0')
        self.assertEqual(path, './inbuild')
        mock_urlretrieve.assert_called_once()

    @patch('urllib.request.urlopen')
    def test_resolve_latest_version_base_not_exists_patch_exists(self, mock_urlopen):
        from urllib.error import HTTPError
        mock_404 = HTTPError('url', 404, 'Not Found', {}, None)
        mock_urlopen.side_effect = [mock_404, MagicMock(), mock_404]
        res = build._resolve_latest_version('v1.31.20260427', 'inbuild-linux-amd64')
        self.assertEqual(res, 'v1.31.20260427.1')

    @patch('urllib.request.urlopen')
    def test_resolve_latest_version_base_exists(self, mock_urlopen):
        from urllib.error import HTTPError
        mock_404 = HTTPError('url', 404, 'Not Found', {}, None)
        mock_urlopen.side_effect = [MagicMock(), mock_404]
        res = build._resolve_latest_version('v1.31.20260427', 'inbuild-linux-amd64')
        self.assertEqual(res, 'v1.31.20260427')

    @patch('urllib.request.urlopen')
    def test_resolve_latest_version_newer_patch_exists(self, mock_urlopen):
        from urllib.error import HTTPError
        mock_404 = HTTPError('url', 404, 'Not Found', {}, None)
        mock_urlopen.side_effect = [MagicMock(), MagicMock(), mock_404]
        res = build._resolve_latest_version('v1.31.20260427.1', 'inbuild-linux-amd64')
        self.assertEqual(res, 'v1.31.20260427.2')

    @patch('urllib.request.urlopen')
    def test_resolve_latest_version_network_error_fallback(self, mock_urlopen):
        mock_urlopen.side_effect = Exception('Connection refused')
        res = build._resolve_latest_version('v1.31.20260427', 'inbuild-linux-amd64')
        self.assertEqual(res, 'v1.31.20260427')

    @patch('subprocess.run')
    def test_run_command(self, mock_run):
        build.run_command(['ls'])
        mock_run.assert_called_once_with(['ls'], check=True)

    @patch('intrinsic_sdk_bundle_library_py.build.run_command')
    def test_build_container_skill(self, mock_run_command):
        args = MagicMock()
        args.service_name = None
        args.service_package = None
        args.skill_name = 'test_skill'
        args.skill_package = 'test_package'
        args.dockerfile = None
        args.bundle_dir = './intrinsic_asset_bundles'
        args.builder_name = 'test-builder'
        args.no_cache = False
        args.ros_distro = 'jazzy'
        args.skill_type = 'python'
        args.skill_executable = None
        args.skill_config = None
        args.skill_asset_id_org = None
        args.dependencies = None

        mock_run_command.return_value = MagicMock()

        build.build_container(args)

        calls = mock_run_command.call_args_list
        found_build = False
        for call in calls:
            args_list = call[0][0]
            if 'docker' in args_list and 'buildx' in args_list and 'build' in args_list:
                found_build = True
                output_idx = args_list.index('--output')
                output_val = args_list[output_idx + 1]
                self.assertIn(
                    'dest=./intrinsic_asset_bundles/test_skill/test_skill.tar',
                    output_val
                )
                break
        self.assertTrue(found_build)

    @patch('shutil.which', return_value=None)
    @patch('intrinsic_sdk_bundle_library_py.build.download_inbuild', return_value='./inbuild')
    @patch('intrinsic_sdk_bundle_library_py.build.get_sdk_version', return_value='v0.1.0')
    @patch('intrinsic_sdk_bundle_library_py.build.run_command')
    @patch('os.path.exists', return_value=True)
    def test_build_bundle_skill(
        self, mock_exists, mock_run_command, mock_get_version, mock_download, mock_which
    ):
        args = MagicMock()
        args.service_name = None
        args.service_package = None
        args.skill_name = 'test_skill'
        args.skill_package = 'test_package'
        args.manifest_path = 'manifest.textproto'
        args.bundle_dir = './intrinsic_asset_bundles'
        args.default_config = None

        mock_run_command.return_value = MagicMock()

        build.build_bundle(args)

        calls = mock_run_command.call_args_list
        found_bundle = False
        for call in calls:
            args_list = call[0][0]
            if './inbuild' in args_list and 'skill' in args_list and 'bundle' in args_list:
                found_bundle = True
                output_idx = args_list.index('--output')
                self.assertEqual(
                    args_list[output_idx + 1],
                    './intrinsic_asset_bundles/test_skill/test_skill.bundle.tar'
                )
                break
        self.assertTrue(found_bundle)

    @patch('intrinsic_sdk_bundle_library_py.build.run_command')
    def test_build_container_service(self, mock_run_command):
        args = MagicMock()
        args.service_name = 'test_service'
        args.service_package = 'test_package'
        args.skill_name = None
        args.skill_package = None
        args.dockerfile = None
        args.bundle_dir = './intrinsic_asset_bundles'
        args.builder_name = 'test-builder'
        args.no_cache = False
        args.ros_distro = 'jazzy'
        args.skill_type = 'cpp'
        args.dependencies = None

        mock_run_command.return_value = MagicMock()

        build.build_container(args)

        calls = mock_run_command.call_args_list
        found_build = False
        for call in calls:
            args_list = call[0][0]
            if 'docker' in args_list and 'buildx' in args_list and 'build' in args_list:
                found_build = True
                output_idx = args_list.index('--output')
                output_val = args_list[output_idx + 1]
                self.assertIn(
                    'dest=./intrinsic_asset_bundles/test_service/test_service.tar',
                    output_val
                )
                break
        self.assertTrue(found_build)

    @patch('shutil.which', return_value=None)
    @patch('intrinsic_sdk_bundle_library_py.build.download_inbuild', return_value='./inbuild')
    @patch('intrinsic_sdk_bundle_library_py.build.get_sdk_version', return_value='v0.1.0')
    @patch('intrinsic_sdk_bundle_library_py.build.run_command')
    @patch('os.path.exists', return_value=True)
    def test_build_bundle_service(
        self, mock_exists, mock_run_command, mock_get_version, mock_download, mock_which
    ):
        args = MagicMock()
        args.service_name = 'test_service'
        args.service_package = 'test_package'
        args.skill_name = None
        args.skill_package = None
        args.manifest_path = 'manifest.textproto'
        args.bundle_dir = './intrinsic_asset_bundles'
        args.default_config = None

        mock_run_command.return_value = MagicMock()

        build.build_bundle(args)

        calls = mock_run_command.call_args_list
        found_bundle = False
        for call in calls:
            args_list = call[0][0]
            if './inbuild' in args_list and 'service' in args_list and 'bundle' in args_list:
                found_bundle = True
                output_idx = args_list.index('--output')
                self.assertEqual(
                    args_list[output_idx + 1],
                    './intrinsic_asset_bundles/test_service/test_service.bundle.tar'
                )
                break
        self.assertTrue(found_bundle)


if __name__ == '__main__':
    unittest.main()
