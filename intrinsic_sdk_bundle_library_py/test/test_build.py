import os
import unittest
from unittest.mock import patch, mock_open, MagicMock
import sys

try:
    from intrinsic_sdk_bundle_library_py import build
except ImportError:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from intrinsic_sdk_bundle_library_py import build

class TestBuild(unittest.TestCase):

    @patch('builtins.open', new_callable=mock_open, read_data='{"sdk_version": "v0.1.0"}')
    def test_get_sdk_version(self, mock_file):
        version = build.get_sdk_version()
        self.assertEqual(version, "v0.1.0")

    @patch('builtins.open', side_effect=Exception("File not found"))
    def test_get_sdk_version_error(self, mock_file):
        version = build.get_sdk_version()
        self.assertIsNone(version)

    @patch('urllib.request.urlretrieve')
    @patch('os.chmod')
    @patch('os.path.exists', return_value=False)
    @patch('platform.system', return_value='Linux')
    @patch('platform.machine', return_value='x86_64')
    def test_download_inbuild(self, mock_machine, mock_system, mock_exists, mock_chmod, mock_urlretrieve):
        path = build.download_inbuild("v0.1.0")
        self.assertEqual(path, "./inbuild")
        mock_urlretrieve.assert_called_once()

    @patch('subprocess.run')
    def test_run_command(self, mock_run):
        build.run_command(['ls'])
        mock_run.assert_called_once_with(['ls'], check=True)

    @patch('intrinsic_sdk_bundle_library_py.build.run_command')
    def test_build_container_skill(self, mock_run_command):
        args = MagicMock()
        args.service_name = None
        args.service_package = None
        args.skill_name = "test_skill"
        args.skill_package = "test_package"
        args.dockerfile = None
        args.images_dir = "./images"
        args.builder_name = "test-builder"
        args.no_cache = False
        args.ros_distro = "jazzy"
        args.skill_type = "python"
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
                break
        self.assertTrue(found_build)

    @patch('intrinsic_sdk_bundle_library_py.build.download_inbuild', return_value='./inbuild')
    @patch('intrinsic_sdk_bundle_library_py.build.get_sdk_version', return_value='v0.1.0')
    @patch('intrinsic_sdk_bundle_library_py.build.run_command')
    @patch('os.path.exists', return_value=True)
    def test_build_bundle_skill(self, mock_exists, mock_run_command, mock_get_version, mock_download):
        args = MagicMock()
        args.service_name = None
        args.service_package = None
        args.skill_name = "test_skill"
        args.skill_package = "test_package"
        args.manifest_path = "manifest.textproto"
        args.images_dir = "./images"
        args.default_config = None

        mock_run_command.return_value = MagicMock()

        build.build_bundle(args)
        
        calls = mock_run_command.call_args_list
        found_bundle = False
        for call in calls:
            args_list = call[0][0]
            if './inbuild' in args_list and 'skill' in args_list and 'bundle' in args_list:
                found_bundle = True
                break
        self.assertTrue(found_bundle)

    @patch('intrinsic_sdk_bundle_library_py.build.run_command')
    def test_build_container_service(self, mock_run_command):
        args = MagicMock()
        args.service_name = "test_service"
        args.service_package = "test_package"
        args.skill_name = None
        args.skill_package = None
        args.dockerfile = None
        args.images_dir = "./images"
        args.builder_name = "test-builder"
        args.no_cache = False
        args.ros_distro = "jazzy"
        args.skill_type = "cpp"
        args.dependencies = None

        mock_run_command.return_value = MagicMock()

        build.build_container(args)
        
        calls = mock_run_command.call_args_list
        found_build = False
        for call in calls:
            args_list = call[0][0]
            if 'docker' in args_list and 'buildx' in args_list and 'build' in args_list:
                found_build = True
                break
        self.assertTrue(found_build)

    @patch('intrinsic_sdk_bundle_library_py.build.download_inbuild', return_value='./inbuild')
    @patch('intrinsic_sdk_bundle_library_py.build.get_sdk_version', return_value='v0.1.0')
    @patch('intrinsic_sdk_bundle_library_py.build.run_command')
    @patch('os.path.exists', return_value=True)
    def test_build_bundle_service(self, mock_exists, mock_run_command, mock_get_version, mock_download):
        args = MagicMock()
        args.service_name = "test_service"
        args.service_package = "test_package"
        args.skill_name = None
        args.skill_package = None
        args.manifest_path = "manifest.textproto"
        args.images_dir = "./images"
        args.default_config = None

        mock_run_command.return_value = MagicMock()

        build.build_bundle(args)
        
        calls = mock_run_command.call_args_list
        found_bundle = False
        for call in calls:
            args_list = call[0][0]
            if './inbuild' in args_list and 'service' in args_list and 'bundle' in args_list:
                found_bundle = True
                break
        self.assertTrue(found_bundle)

if __name__ == '__main__':
    unittest.main()
