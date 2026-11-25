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

"""Command-line entrypoint and public Python API for aquarium."""

import argparse
import sys

from . import codegen_intrinsic as _codegen_intrinsic
from . import config as _config
from .builder import run_build
from .codegen_intrinsic import generate_skill
from .codegen_intrinsic import map_ros_type_to_proto
from .codegen_intrinsic import run_generate
from .codegen_intrinsic import stage_container_resources
from .config import find_launch_file_info as _find_launch_file_info
from .config import get_target_image_tags as _get_target_image_tags
from .config import get_target_packages as _get_target_packages
from .config import infer_proto_type
from .config import parse_targets_config as _parse_targets_config
from .config import resolve_target_arguments as _resolve_target_arguments
from .config import run_config

_generate_entrypoint_content = _codegen_intrinsic.generate_entrypoint_content
_generate_launch_xml_content = _codegen_intrinsic.generate_launch_xml_content
_parse_launch_file_arguments_from_path = (
    _config.parse_launch_file_arguments_from_path
)
from .generate_ros import build_arg_parser as build_generate_ros_parser
from .generate_ros import run_generate_ros
from .runner import build_target_ros2_command
from .runner import run_assistant
from .runner import run_gateway
from .workspace import find_sdk_ros_dir
from .workspace import find_workspace_dir
from .workspace import get_inbuild_path
from .workspace import get_intrinsic_proto_desc_path
from .workspace import get_runtime_context_proto_path
from .workspace import get_sdk_version

try:
  import rclpy  # pylint: disable=unused-import

  HAS_RCLPY = True
except ImportError:
  HAS_RCLPY = False

__all__ = [
    'HAS_RCLPY',
    '_find_launch_file_info',
    '_generate_entrypoint_content',
    '_generate_launch_xml_content',
    '_get_target_image_tags',
    '_get_target_packages',
    '_parse_launch_file_arguments_from_path',
    '_parse_targets_config',
    '_resolve_target_arguments',
    'build_parser',
    'build_target_ros2_command',
    'find_sdk_ros_dir',
    'find_workspace_dir',
    'generate_skill',
    'get_inbuild_path',
    'get_intrinsic_proto_desc_path',
    'get_runtime_context_proto_path',
    'get_sdk_version',
    'infer_proto_type',
    'main',
    'map_ros_type_to_proto',
    'run_assistant',
    'run_build',
    'run_config',
    'run_gateway',
    'run_generate',
    'stage_container_resources',
]


def build_parser():
  """Construct the top-level argparse parser for the aquarium CLI."""
  parser = argparse.ArgumentParser(
      prog='aquarium',
      description=(
          'Dynamic ROS 2 to Intrinsic Flowstate gateway, setup assistant, and'
          ' asset bundler.'
      ),
  )
  subparsers = parser.add_subparsers(dest='command')

  # aquarium assistant {launch,run} <package_name> <target> [arguments...]
  assistant_parser = subparsers.add_parser(
      'assistant',
      help=(
          'Launch a ROS 2 node or launch file in the background and run the'
          ' setup assistant TUI.'
      ),
  )
  assistant_parser.add_argument(
      'mode',
      choices=('launch', 'run'),
      help="Whether to invoke 'ros2 launch' or 'ros2 run'.",
  )
  assistant_parser.add_argument(
      'package_name',
      help='Target ROS 2 package name.',
  )
  assistant_parser.add_argument(
      'target',
      help='Target launch file or executable name.',
  )
  assistant_parser.add_argument(
      'remaining_args',
      nargs=argparse.REMAINDER,
      help='Optional --selection <file> and ROS 2/launch arguments.',
  )

  # aquarium build [--privileged] [--no-cache] <config_path>
  build_cmd_parser = subparsers.add_parser(
      'build',
      help='Generate artifacts, build container images, and create bundles.',
  )
  build_cmd_parser.add_argument(
      '--privileged',
      action='store_true',
      help='Run service container with privileged security context.',
  )
  build_cmd_parser.add_argument(
      '--no-cache',
      action='store_true',
      dest='no_cache',
      help='Force clean container image builds without layer cache.',
  )
  build_cmd_parser.add_argument(
      'config_path',
      help='Path to aquarium.yaml configuration file.',
  )

  # aquarium config <input_textproto> [output_binpb] [-o ...] [--proto ...]
  config_parser = subparsers.add_parser(
      'config',
      help=(
          'Encode a service config .textproto file into a binary'
          ' google.protobuf.Any (.binpb) file for inctl service add --config.'
      ),
  )
  config_parser.add_argument(
      'input_textproto',
      help='Input service configuration .textproto file.',
  )
  config_parser.add_argument(
      'output_binpb_pos',
      nargs='?',
      default=None,
      help='Optional output .binpb path.',
  )
  config_parser.add_argument(
      '-o',
      '--output',
      dest='output_binpb_opt',
      default=None,
      help='Output .binpb file path.',
  )
  config_parser.add_argument(
      '--proto',
      dest='proto_path',
      default=None,
      help='Path to the generated *_config.proto file.',
  )

  # aquarium generate {intrinsic,ros} ...
  generate_parser = subparsers.add_parser(
      'generate',
      help='Generate Intrinsic Flowstate or ROS 2 interface packages.',
  )
  generate_subparsers = generate_parser.add_subparsers(dest='subcommand')

  gen_intrinsic = generate_subparsers.add_parser(
      'intrinsic',
      help='Generate Flowstate manifests, entrypoints, and C++ skill packages.',
  )
  gen_intrinsic.add_argument(
      '--privileged',
      action='store_true',
      help='Configure service manifest with privileged security context.',
  )
  gen_intrinsic.add_argument(
      'config_path',
      help='Path to aquarium.yaml configuration file.',
  )
  gen_intrinsic.add_argument(
      'output_dir',
      nargs='?',
      default=None,
      help='Output directory (defaults to build/aquarium_images).',
  )

  gen_ros = generate_subparsers.add_parser(
      'ros',
      help='Generate ROS 2 interface packages from a Proto service definition.',
  )
  build_generate_ros_parser(gen_ros)

  # aquarium run [--sim] <config_path>
  run_parser = subparsers.add_parser(
      'run',
      help=(
          'Launch rmw_zenohd and the configured target in the background and'
          ' run aquarium_gateway in the foreground.'
      ),
  )
  run_parser.add_argument(
      '--sim',
      action='store_true',
      help="Use the 'sim' target when 'targets' is defined in aquarium.yaml.",
  )
  run_parser.add_argument(
      'config_path',
      help='Path to aquarium.yaml configuration file.',
  )

  return parser


def main(argv=None):
  """Run the aquarium CLI orchestrator."""
  raw_args = list(sys.argv[1:] if argv is None else argv)
  parser = build_parser()

  if not raw_args:
    parser.print_help()
    sys.exit(1)

  parsed = parser.parse_args(raw_args)

  if parsed.command == 'assistant':
    if not HAS_RCLPY:
      print(
          "Error: 'assistant' command requires rclpy (ROS 2 Python bindings)."
      )
      sys.exit(1)
    run_assistant(
        parsed.mode,
        parsed.package_name,
        parsed.target,
        parsed.remaining_args,
    )
  elif parsed.command == 'build':
    run_build(
        parsed.config_path,
        privileged=parsed.privileged,
        no_cache=parsed.no_cache,
    )
  elif parsed.command == 'config':
    output_binpb = parsed.output_binpb_opt or parsed.output_binpb_pos
    run_config(
        parsed.input_textproto,
        output_binpb=output_binpb,
        proto_path=parsed.proto_path,
    )
  elif parsed.command == 'generate':
    if parsed.subcommand == 'intrinsic':
      run_generate(
          parsed.config_path,
          parsed.output_dir,
          privileged=parsed.privileged,
      )
    elif parsed.subcommand == 'ros':
      run_generate_ros(
          service_name=parsed.service,
          proto_path=parsed.proto_path,
          output_path=parsed.output_path,
          single_package=parsed.single_package,
          target_package=parsed.target_package,
          target_node=parsed.target_node,
      )
    else:
      parser.parse_args(['generate', '--help'])
      sys.exit(1)
  elif parsed.command == 'run':
    if not HAS_RCLPY:
      print("Error: 'run' command requires rclpy (ROS 2 Python bindings).")
      sys.exit(1)
    run_gateway(parsed.config_path, use_sim=parsed.sim)
  else:
    parser.print_help()
    sys.exit(1)


if __name__ == '__main__':
  main()
