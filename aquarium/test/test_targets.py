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

import io
import json
import os
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

from aquarium.builder import run_build
from aquarium.cli import _generate_entrypoint_content
from aquarium.cli import _generate_launch_xml_content
from aquarium.cli import _get_target_image_tags
from aquarium.cli import _get_target_packages
from aquarium.cli import _parse_launch_file_arguments_from_path
from aquarium.cli import _parse_targets_config
from aquarium.cli import _resolve_target_arguments
from aquarium.cli import build_parser
from aquarium.cli import build_target_ros2_command
from aquarium.cli import find_sdk_ros_dir
from aquarium.cli import find_workspace_dir
from aquarium.cli import generate_skill
from aquarium.cli import get_sdk_version
from aquarium.cli import run_config
from aquarium.cli import run_generate
from aquarium.cli import stage_container_resources
from aquarium.codegen_intrinsic import STAGEABLE_RESOURCES
from aquarium.generate_ros import run_generate_ros


class TestTargetsConfig(unittest.TestCase):

  def test_parse_valid_single_target(self):
    config = {
        'target': {
            'type': 'node',
            'package': 'demo_nodes_cpp',
            'name': 'add_two_ints_server',
        }
    }
    real_target, sim_target = _parse_targets_config(config)
    self.assertEqual(real_target, sim_target)
    self.assertEqual(real_target['package'], 'demo_nodes_cpp')

  def test_parse_valid_distinct_targets(self):
    config = {
        'targets': {
            'sim': {
                'type': 'launch',
                'package': 'robotiq_bringup',
                'name': 'robotiq.launch.xml',
                'arguments': ['use_fake_hardware:=true'],
            },
            'real': {
                'type': 'launch',
                'package': 'robotiq_bringup',
                'name': 'robotiq.launch.xml',
                'arguments': ['use_fake_hardware:=false'],
            },
        }
    }
    real_target, sim_target = _parse_targets_config(config)
    self.assertNotEqual(real_target, sim_target)
    self.assertEqual(sim_target['arguments'], ['use_fake_hardware:=true'])
    self.assertEqual(real_target['arguments'], ['use_fake_hardware:=false'])

  def test_parse_both_target_and_targets_exits(self):
    config = {
        'target': {
            'type': 'node',
            'package': 'demo_nodes_cpp',
            'name': 'add_two_ints_server',
        },
        'targets': {
            'sim': {
                'type': 'node',
                'package': 'demo_nodes_cpp',
                'name': 'add_two_ints_server',
            },
            'real': {
                'type': 'node',
                'package': 'demo_nodes_cpp',
                'name': 'add_two_ints_server',
            },
        },
    }
    with self.assertRaises(SystemExit):
      _parse_targets_config(config)

  def test_parse_missing_both_exits(self):
    config = {'interfaces': []}
    with self.assertRaises(SystemExit):
      _parse_targets_config(config)

  def test_parse_missing_sim_or_real_exits(self):
    config = {
        'targets': {
            'real': {
                'type': 'node',
                'package': 'demo_nodes_cpp',
                'name': 'add_two_ints_server',
            }
        }
    }
    with self.assertRaises(SystemExit):
      _parse_targets_config(config)

  def test_image_tags_identical_targets(self):
    target = {
        'type': 'launch',
        'package': 'my_robot_bringup',
        'name': 'robot.launch.py',
    }
    base_tag, svc_name, real_tag, sim_tag = _get_target_image_tags(
        target, target
    )
    self.assertEqual(base_tag, 'my-robot-bringup-robot.launch.py')
    self.assertEqual(svc_name, 'my_robot_bringup_robot_launch_py')
    self.assertEqual(real_tag, sim_tag)
    self.assertEqual(real_tag, base_tag)

  def test_image_tags_distinct_package_or_name(self):
    real_target = {
        'type': 'launch',
        'package': 'real_robot_bringup',
        'name': 'real.launch.py',
    }
    sim_target = {
        'type': 'launch',
        'package': 'sim_robot_bringup',
        'name': 'sim.launch.py',
    }
    _, _, real_tag, sim_tag = _get_target_image_tags(real_target, sim_target)
    self.assertNotEqual(real_tag, sim_tag)
    self.assertEqual(real_tag, 'real-robot-bringup-real.launch.py')
    self.assertEqual(sim_tag, 'sim-robot-bringup-sim.launch.py')

  def test_image_tags_same_name_distinct_arguments(self):
    real_target = {
        'type': 'launch',
        'package': 'robot_bringup',
        'name': 'robot.launch.py',
        'arguments': ['use_sim:=false'],
    }
    sim_target = {
        'type': 'launch',
        'package': 'robot_bringup',
        'name': 'robot.launch.py',
        'arguments': ['use_sim:=true'],
    }
    _, _, real_tag, sim_tag = _get_target_image_tags(real_target, sim_target)
    self.assertNotEqual(real_tag, sim_tag)
    self.assertEqual(real_tag, 'robot-bringup-robot.launch.py-real')
    self.assertEqual(sim_tag, 'robot-bringup-robot.launch.py-sim')

  def test_generate_launch_and_entrypoint(self):
    target = {
        'type': 'launch',
        'package': 'robotiq_bringup',
        'name': 'robotiq.launch.py',
        'arguments': ['com_port:=/dev/ttyUSB0', 'required_token'],
    }
    xml_content = _generate_launch_xml_content(target)
    self.assertIn('find-pkg-share robotiq_bringup', xml_content)
    self.assertIn('robotiq.launch.py', xml_content)
    self.assertIn('<arg name="com_port" default="/dev/ttyUSB0"/>', xml_content)
    self.assertIn('<arg name="required_token" default=""/>', xml_content)
    self.assertIn(
        '<arg name="required_token" value="$(var required_token)"/>',
        xml_content,
    )
    self.assertIn(
        '<node pkg="aquarium" exec="aquarium_gateway"',
        xml_content,
    )

    entrypoint_content = _generate_entrypoint_content(
        'robotiq_service',
        'RobotiqServiceConfig',
        ['com_port:=/dev/ttyUSB0', 'required_token'],
    )
    self.assertIn('robotiq_service_config_pb2', entrypoint_content)
    self.assertIn("'com_port'", entrypoint_content)
    self.assertIn("'required_token'", entrypoint_content)
    self.assertIn('config.HasField(field.name)', entrypoint_content)
    self.assertIn('is_running_in_cluster()', entrypoint_content)

  def test_parse_launch_file_arguments_xml_py_yaml(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      xml_path = os.path.join(tmpdir, 'test.launch.xml')
      with open(xml_path, 'w', encoding='utf-8') as f:
        f.write(
            '<launch>\n'
            '  <arg name="device_ip_address" default="192.168.1.40"/>\n'
            '  <arg name="device_network_port" default="10001"/>\n'
            '  <arg name="device_serial"/>\n'
            '</launch>\n'
        )
      self.assertEqual(
          _parse_launch_file_arguments_from_path(xml_path),
          [
              'device_ip_address:=192.168.1.40',
              'device_network_port:=10001',
              'device_serial',
          ],
      )

      py_path = os.path.join(tmpdir, 'test.launch.py')
      with open(py_path, 'w', encoding='utf-8') as f:
        f.write(
            'from launch import LaunchDescription\n'
            'from launch.actions import DeclareLaunchArgument\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([\n'
            '        DeclareLaunchArgument("ip", default_value="10.0.0.1"),\n'
            '        DeclareLaunchArgument("use_sim", default_value=True),\n'
            '        DeclareLaunchArgument("no_default_arg"),\n'
            '    ])\n'
        )
      self.assertEqual(
          _parse_launch_file_arguments_from_path(py_path),
          ['ip:=10.0.0.1', 'use_sim:=true', 'no_default_arg'],
      )

      yaml_path = os.path.join(tmpdir, 'test.launch.yaml')
      with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(
            'launch:\n'
            '  - arg:\n'
            '      name: "host"\n'
            '      default: "localhost"\n'
            '  - arg:\n'
            '      name: "token"\n'
        )
      self.assertEqual(
          _parse_launch_file_arguments_from_path(yaml_path),
          ['host:=localhost', 'token'],
      )

  def test_resolve_target_arguments_from_workspace(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      pkg_dir = os.path.join(tmpdir, 'src', 'my_pkg')
      os.makedirs(pkg_dir)
      with open(
          os.path.join(pkg_dir, 'package.xml'), 'w', encoding='utf-8'
      ) as f:
        f.write('<package><name>my_pkg</name></package>\n')
      with open(
          os.path.join(pkg_dir, 'launch.xml'), 'w', encoding='utf-8'
      ) as f:
        f.write(
            '<launch>\n'
            '  <arg name="ip" default="192.168.1.40"/>\n'
            '  <arg name="port" default="10001"/>\n'
            '  <arg name="serial"/>\n'
            '</launch>\n'
        )

      target = {
          'type': 'launch',
          'package': 'my_pkg',
          'name': 'launch.xml',
          'arguments': ['ip:=192.168.1.99'],
      }
      resolved = _resolve_target_arguments(target, workspace_dir=tmpdir)
      self.assertEqual(
          resolved, ['ip:=192.168.1.99', 'port:=10001', 'serial']
      )
      xml_out = _generate_launch_xml_content(target, workspace_dir=tmpdir)
      self.assertIn(
          '<include file="$(find-pkg-share my_pkg)/launch.xml">', xml_out
      )

  def test_run_config_encodes_binpb_and_prints_inctl(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      images_dir = os.path.join(tmpdir, 'build', 'aquarium_images')
      os.makedirs(images_dir)
      proto_path = os.path.join(images_dir, 'test_svc_config.proto')
      with open(proto_path, 'w', encoding='utf-8') as f:
        f.write(
            'syntax = "proto3";\n'
            'package ai.intrinsic;\n'
            'message TestSvcConfig {\n'
            '  optional string device_ip_address = 1;\n'
            '  optional int32 device_network_port = 2;\n'
            '}\n'
        )
      manifest_path = os.path.join(images_dir, 'test-svc.manifest.textproto')
      with open(manifest_path, 'w', encoding='utf-8') as f:
        f.write(
            'metadata { id { package: "ai.intrinsic" name: "test_svc" } }\n'
            'service_def {\n'
            '  config_message_full_name: "ai.intrinsic.TestSvcConfig"\n'
            '}\n'
        )

      wrapped_txtpb = os.path.join(tmpdir, 'wrapped_config.textproto')
      with open(wrapped_txtpb, 'w', encoding='utf-8') as f:
        f.write(
            '[type.googleapis.com/ai.intrinsic.TestSvcConfig] {\n'
            '  device_ip_address: "192.168.1.99"\n'
            '  device_network_port: 10001\n'
            '}\n'
        )
      buf = io.StringIO()
      with mock.patch('sys.stdout', buf):
        out_binpb = run_config(wrapped_txtpb, workspace_dir=tmpdir)
      stdout_txt = buf.getvalue()
      self.assertIn('inctl service add ai.intrinsic.test_svc', stdout_txt)
      self.assertNotIn('inctl_external', stdout_txt)
      self.assertEqual(out_binpb, os.path.join(tmpdir, 'wrapped_config.binpb'))
      self.assertTrue(os.path.isfile(out_binpb))
      with open(out_binpb, 'rb') as f:
        raw_wrapped = f.read()
      self.assertIn(b'192.168.1.99', raw_wrapped)

      plain_txtpb = os.path.join(tmpdir, 'plain_config.textproto')
      with open(plain_txtpb, 'w', encoding='utf-8') as f:
        f.write(
            'device_ip_address: "192.168.1.99"\n'
            'device_network_port: 10001\n'
        )
      out_plain = run_config(plain_txtpb, workspace_dir=tmpdir)
      with open(out_plain, 'rb') as f:
        raw_plain = f.read()
      self.assertEqual(raw_wrapped, raw_plain)

      partial_txtpb = os.path.join(tmpdir, 'partial_config.textproto')
      with open(partial_txtpb, 'w', encoding='utf-8') as f:
        f.write('device_ip_address: "192.168.1.99"\n')
      out_partial = run_config(partial_txtpb, workspace_dir=tmpdir)
      with open(out_partial, 'rb') as f:
        raw_partial = f.read()
      self.assertIn(b'192.168.1.99', raw_partial)
      self.assertLess(len(raw_partial), len(raw_plain))

  def test_generate_skill_templates_and_generic_client(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      interface = {
          'internal_interface': 'ros_action_client',
          'internal_name': '/sample_action',
          'internal_type': 'example_interfaces/action/SampleAction',
          'external_interface': 'zenoh_queryable_proto',
          'external_name': 'sample_action',
          'type_hash': 'RIHS01_dummy',
          'request_fields': ['uint16 port', 'uint64 count'],
          'response_fields': ['uint16 status'],
      }
      skills_dir = os.path.join(tmpdir, 'skills')
      os.makedirs(skills_dir)
      self.assertTrue(generate_skill(interface, skills_dir, tmpdir))

      pkg_xml = os.path.join(skills_dir, 'sample_action_skill', 'package.xml')
      with open(pkg_xml, 'r', encoding='utf-8') as f:
        pkg_xml_txt = f.read()
      self.assertIn('ros@intrinsic.ai', pkg_xml_txt)
      self.assertNotIn('google.com', pkg_xml_txt)

      client_cc = os.path.join(
          skills_dir, 'sample_action_skill', 'sample_action_skill_client.cc'
      )
      with open(client_cc, 'r', encoding='utf-8') as f:
        content = f.read()
      self.assertIn('params.set_port(42);', content)
      self.assertIn('params.set_count(42);', content)
      self.assertNotIn('mutable_port', content)
      self.assertNotIn('mutable_count', content)

      # Verify add_two_ints uses the exact same generic codegen path
      add_interface = {
          'internal_interface': 'ros_service',
          'internal_name': '/add_two_ints',
          'internal_type': 'example_interfaces/srv/AddTwoInts',
          'external_interface': 'zenoh_queryable_proto',
          'external_name': 'add_two_ints',
          'type_hash': 'RIHS01_dummy',
          'request_fields': ['int64 a', 'int64 b'],
          'response_fields': ['int64 sum'],
      }
      self.assertTrue(generate_skill(add_interface, skills_dir, tmpdir))
      add_client_cc = os.path.join(
          skills_dir, 'add_two_ints_skill', 'add_two_ints_skill_client.cc'
      )
      with open(add_client_cc, 'r', encoding='utf-8') as f:
        add_content = f.read()
      self.assertIn('params.set_a(42);', add_content)
      self.assertIn('params.set_b(42);', add_content)
      self.assertNotIn('set_b(58)', add_content)

  def test_parse_target_dependencies_valid_and_deduplicated(self):
    config = {
        'target': {
            'type': 'launch',
            'package': 'aquarium',
            'name': 'robotiq.launch.xml',
            'dependencies': [
                'robotiq_driver',
                'robotiq_description',
                'aquarium',
            ],
        }
    }
    real_target, sim_target = _parse_targets_config(config)
    self.assertEqual(real_target, sim_target)
    self.assertEqual(
        _get_target_packages(real_target),
        ['aquarium', 'robotiq_driver', 'robotiq_description'],
    )

  def test_parse_target_dependencies_invalid_exits(self):
    bad_str_config = {
        'target': {
            'type': 'launch',
            'package': 'aquarium',
            'name': 'robotiq.launch.xml',
            'dependencies': 'robotiq_driver',
        }
    }
    with self.assertRaises(SystemExit):
      _parse_targets_config(bad_str_config)

    bad_empty_item_config = {
        'targets': {
            'sim': {
                'type': 'launch',
                'package': 'aquarium',
                'name': 'robotiq.launch.xml',
                'dependencies': [''],
            },
            'real': {
                'type': 'launch',
                'package': 'aquarium',
                'name': 'robotiq.launch.xml',
            },
        }
    }
    with self.assertRaises(SystemExit):
      _parse_targets_config(bad_empty_item_config)

  def test_sdk_ros_aquarium_layout_and_sdk_version(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      aq_dir = os.path.join(tmpdir, 'src', 'sdk-ros', 'aquarium')
      cmake_pkg_dir = os.path.join(
          tmpdir, 'src', 'sdk-ros', 'intrinsic_sdk_cmake'
      )
      sbl_res_dir = os.path.join(
          tmpdir,
          'src',
          'sdk-ros',
          'intrinsic_sdk_bundle_library_py',
          'resource',
      )
      ex_dir = os.path.join(aq_dir, 'examples', 'robotiq')
      os.makedirs(ex_dir)
      os.makedirs(cmake_pkg_dir)
      os.makedirs(sbl_res_dir)

      with open(
          os.path.join(aq_dir, 'package.xml'), 'w', encoding='utf-8'
      ) as f:
        f.write('<package><name>aquarium</name></package>\n')
      with open(
          os.path.join(cmake_pkg_dir, 'package.xml'), 'w', encoding='utf-8'
      ) as f:
        f.write('<package><name>intrinsic_sdk_cmake</name></package>\n')
      with open(
          os.path.join(sbl_res_dir, 'sdk_version.json'), 'w', encoding='utf-8'
      ) as f:
        json.dump({'sdk_version': 'v1.99.20261231'}, f)
      with open(
          os.path.join(ex_dir, 'robotiq.launch.xml'), 'w', encoding='utf-8'
      ) as f:
        f.write(
            '<launch>\n'
            '  <arg name="use_fake_hardware" default="false"/>\n'
            '</launch>\n'
        )

      self.assertEqual(find_workspace_dir(ex_dir), tmpdir)
      self.assertEqual(find_sdk_ros_dir(tmpdir), 'src/sdk-ros')
      self.assertEqual(get_sdk_version(tmpdir), 'v1.99.20261231')

      target_by_name = {
          'type': 'launch',
          'package': 'aquarium',
          'name': 'robotiq.launch.xml',
          'dependencies': ['robotiq_driver', 'robotiq_description'],
      }
      xml_by_name = _generate_launch_xml_content(
          target_by_name, workspace_dir=tmpdir
      )
      self.assertIn(
          '<include file="$(find-pkg-share'
          ' aquarium)/examples/robotiq/robotiq.launch.xml">',
          xml_by_name,
      )

      target_by_relpath = {
          'type': 'launch',
          'package': 'aquarium',
          'name': 'examples/robotiq/robotiq.launch.xml',
      }
      base_tag, svc_name, _, _ = _get_target_image_tags(
          target_by_relpath, target_by_relpath
      )
      self.assertNotIn('/', base_tag)
      self.assertEqual(
          svc_name, 'aquarium_examples_robotiq_robotiq_launch_xml'
      )

  def test_stage_container_resources_and_generate_no_src_aquarium_paths(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      aq_dir = os.path.join(tmpdir, 'src', 'sdk-ros', 'aquarium')
      sdk_proto_dir = os.path.join(
          tmpdir, 'src', 'sdk', 'intrinsic', 'resources', 'proto'
      )
      os.makedirs(aq_dir)
      os.makedirs(sdk_proto_dir)
      with open(
          os.path.join(aq_dir, 'package.xml'), 'w', encoding='utf-8'
      ) as f:
        f.write('<package><name>aquarium</name></package>\n')
      with open(
          os.path.join(sdk_proto_dir, 'runtime_context.proto'),
          'w',
          encoding='utf-8',
      ) as f:
        f.write('syntax = "proto3";\npackage intrinsic_proto.config;\n')

      cfg_path = os.path.join(tmpdir, 'aquarium.yaml')
      with open(cfg_path, 'w', encoding='utf-8') as f:
        f.write(
            'target:\n'
            '  type: "node"\n'
            '  package: "demo_nodes_cpp"\n'
            '  name: "add_two_ints_server"\n'
            'interfaces:\n'
            '  - internal_interface: "ros_service"\n'
            '    internal_name: "/add_two_ints"\n'
            '    internal_type: "example_interfaces/srv/AddTwoInts"\n'
            '    external_interface: "zenoh_queryable_proto"\n'
            '    external_name: "add_two_ints"\n'
            '    type_hash: "RIHS01_dummy"\n'
            '    request_fields: ["int64 a", "int64 b"]\n'
            '    response_fields: ["int64 sum"]\n'
        )

      out_dir = os.path.join(tmpdir, 'build', 'custom_out')
      stage_container_resources(out_dir, workspace_dir=tmpdir)
      for res_name in STAGEABLE_RESOURCES:
        self.assertTrue(os.path.isfile(os.path.join(out_dir, res_name)))
      with open(
          os.path.join(out_dir, 'dockerignore'), 'r', encoding='utf-8'
      ) as f:
        dockerignore_txt = f.read()
      self.assertIn('!build/custom_out/', dockerignore_txt)

      buf = io.StringIO()
      old_cwd = os.getcwd()
      try:
        os.chdir(tmpdir)
        with mock.patch('sys.stdout', buf):
          run_generate(cfg_path, output_dir=out_dir)
      finally:
        os.chdir(old_cwd)

      gen_stdout = buf.getvalue()
      self.assertNotIn('src/aquarium', gen_stdout)
      self.assertIn('build/custom_out/Dockerfile.service', gen_stdout)
      self.assertIn('build/custom_out/Dockerfile.skill', gen_stdout)

  def test_cli_argparse_and_run_gateway_target_commands(self):
    parser = build_parser()

    a_parsed = parser.parse_args([
        'assistant',
        'launch',
        'aquarium',
        'robotiq.launch.xml',
        '--selection',
        'sel.yaml',
        'com_port:=/dev/ttyUSB0',
        '--ros-args',
        '-p',
        'foo:=bar',
    ])
    self.assertEqual(a_parsed.command, 'assistant')
    self.assertEqual(a_parsed.mode, 'launch')
    self.assertEqual(
        a_parsed.remaining_args,
        [
            '--selection',
            'sel.yaml',
            'com_port:=/dev/ttyUSB0',
            '--ros-args',
            '-p',
            'foo:=bar',
        ],
    )

    b_parsed = parser.parse_args(
        ['build', '--privileged', '--no-cache', 'aquarium.yaml']
    )
    self.assertTrue(b_parsed.privileged)
    self.assertTrue(b_parsed.no_cache)
    self.assertEqual(b_parsed.config_path, 'aquarium.yaml')

    r_parsed = parser.parse_args(['run', '--sim', 'aquarium.yaml'])
    self.assertTrue(r_parsed.sim)

    cmd_node = build_target_ros2_command({
        'target': {
            'type': 'node',
            'package': 'demo_nodes_cpp',
            'name': 'add_two_ints_server',
        }
    })
    self.assertEqual(
        cmd_node, ['ros2', 'run', 'demo_nodes_cpp', 'add_two_ints_server']
    )

    dual_cfg = {
        'targets': {
            'real': {
                'type': 'launch',
                'package': 'robot_bringup',
                'name': 'robot.launch.xml',
                'arguments': ['sim:=false'],
            },
            'sim': {
                'type': 'launch',
                'package': 'robot_bringup',
                'name': 'robot.launch.xml',
                'arguments': ['sim:=true'],
            },
        }
    }
    self.assertEqual(
        build_target_ros2_command(dual_cfg, use_sim=False),
        ['ros2', 'launch', 'robot_bringup', 'robot.launch.xml', 'sim:=false'],
    )
    self.assertEqual(
        build_target_ros2_command(dual_cfg, use_sim=True),
        ['ros2', 'launch', 'robot_bringup', 'robot.launch.xml', 'sim:=true'],
    )

  def test_generate_ros_state_isolation_and_maintainer(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      with open(os.path.join(tmpdir, 'WORKSPACE'), 'w', encoding='utf-8'):
        pass
      proto_a = os.path.join(tmpdir, 'svc_a.proto')
      with open(proto_a, 'w', encoding='utf-8') as f:
        f.write(
            'syntax = "proto3";\n'
            'package demo.alpha;\n'
            'message AlphaItem { int32 id = 1; }\n'
            'message PingReq { AlphaItem item = 1; }\n'
            'message PingResp { bool ok = 1; }\n'
            'service AlphaService {\n'
            '  rpc Ping (PingReq) returns (PingResp);\n'
            '}\n'
        )
      proto_b = os.path.join(tmpdir, 'svc_b.proto')
      with open(proto_b, 'w', encoding='utf-8') as f:
        f.write(
            'syntax = "proto3";\n'
            'package demo.beta;\n'
            'message PongReq { string label = 1; }\n'
            'message PongResp { int32 count = 1; }\n'
            'service BetaService {\n'
            '  rpc Pong (PongReq) returns (PongResp);\n'
            '}\n'
        )

      out_a = os.path.join(tmpdir, 'out_a')
      out_b = os.path.join(tmpdir, 'out_b')
      run_generate_ros(
          service_name='AlphaService',
          proto_path=proto_a,
          output_path=out_a,
          single_package='alpha_msgs',
      )
      run_generate_ros(
          service_name='BetaService',
          proto_path=proto_b,
          output_path=out_b,
          single_package='beta_msgs',
      )

      # Second run must not inherit alpha_msgs packages or files from first run.
      self.assertFalse(os.path.exists(os.path.join(out_b, 'alpha_msgs')))
      beta_pkg_xml = os.path.join(out_b, 'beta_msgs', 'package.xml')
      with open(beta_pkg_xml, 'r', encoding='utf-8') as f:
        xml_txt = f.read()
      self.assertIn('ros@intrinsic.ai', xml_txt)
      self.assertNotIn('todo@todo.com', xml_txt)

  def test_launch_xml_escapes_special_characters(self):
    target = {
        'type': 'launch',
        'package': 'my_pkg',
        'name': 'demo.launch.py',
        'arguments': [
            'xacro_cmd:=xacro "a&b" <tag>',
        ],
    }
    xml_out = _generate_launch_xml_content(target)
    root = ET.fromstring(xml_out)
    arg_elem = root.find('arg')
    self.assertIsNotNone(arg_elem)
    self.assertEqual(arg_elem.attrib['name'], 'xacro_cmd')
    self.assertEqual(arg_elem.attrib['default'], 'xacro "a&b" <tag>')

  def test_skill_sanitization_and_per_skill_types_filtering(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      skills_dir = os.path.join(tmpdir, 'skills')
      os.makedirs(skills_dir)

      types_list = [
          {
              'name': 'sensor_msgs/msg/JointState',
              'fields': [
                  'std_msgs/msg/Header header',
                  'string[] name',
                  'double[] position',
              ],
          },
          {
              'name': 'std_msgs/msg/Header',
              'fields': [
                  'builtin_interfaces/msg/Time stamp',
                  'string frame_id',
              ],
          },
          {
              'name': 'builtin_interfaces/msg/Time',
              'fields': ['int32 sec', 'uint32 nanosec'],
          },
          # Duplicate Header entry should be deduplicated cleanly.
          {
              'name': 'std_msgs/msg/Header',
              'fields': [
                  'builtin_interfaces/msg/Time stamp',
                  'string frame_id',
              ],
          },
          # Unreferenced type from another interface should be omitted.
          {
              'name': 'pkg_unused/msg/Unused',
              'fields': ['int32 extra'],
          },
      ]
      interface = {
          'internal_interface': 'ros_action_client',
          'internal_name': '/gripper/cmd',
          'internal_type': 'control_msgs/action/ParallelGripperCommand',
          'external_interface': 'zenoh_queryable_proto',
          'external_name': 'gripper/cmd-v2',
          'type_hash': 'RIHS01_dummy',
          'request_fields': ['sensor_msgs/msg/JointState command'],
          'response_fields': ['bool reached_goal'],
      }

      skill_pkg = generate_skill(
          interface, skills_dir, tmpdir, types_list=types_list
      )
      self.assertEqual(skill_pkg, 'gripper_cmd_v2_skill')

      proto_file = os.path.join(
          skills_dir, 'gripper_cmd_v2_skill', 'gripper_cmd_v2_skill.proto'
      )
      with open(proto_file, 'r', encoding='utf-8') as f:
        proto_txt = f.read()
      self.assertEqual(proto_txt.count('message Header {'), 1)
      self.assertNotIn('message Unused {', proto_txt)
      self.assertLess(
          proto_txt.index('message Time {'),
          proto_txt.index('message Header {'),
      )
      self.assertLess(
          proto_txt.index('message Header {'),
          proto_txt.index('message JointState {'),
      )

      skill_cc = os.path.join(
          skills_dir, 'gripper_cmd_v2_skill', 'gripper_cmd_v2_skill.cc'
      )
      with open(skill_cc, 'r', encoding='utf-8') as f:
        cc_txt = f.read()
      self.assertIn('std::string key = "gripper/cmd-v2";', cc_txt)
      self.assertIn('GripperCmdV2Skill::Execute', cc_txt)

      # Conflicting short names referenced by the same skill must exit cleanly.
      conflict_types = [
          {'name': 'pkg_a/msg/Status', 'fields': ['int32 code']},
          {'name': 'pkg_b/msg/Status', 'fields': ['string detail']},
      ]
      conflict_inf = {
          'internal_interface': 'ros_service',
          'internal_name': '/check',
          'internal_type': 'pkg_a/srv/Check',
          'external_interface': 'zenoh_queryable_proto',
          'external_name': 'check_status',
          'type_hash': 'RIHS01_dummy',
          'request_fields': ['pkg_a/msg/Status req_status'],
          'response_fields': ['pkg_b/msg/Status res_status'],
      }
      with self.assertRaises(SystemExit):
        generate_skill(
            conflict_inf, skills_dir, tmpdir, types_list=conflict_types
        )

  def test_generate_prints_prune_patterns_and_preflight_checks(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      aq_dir = os.path.join(tmpdir, 'src', 'sdk-ros', 'aquarium')
      sdk_proto_dir = os.path.join(
          tmpdir, 'src', 'sdk', 'intrinsic', 'resources', 'proto'
      )
      os.makedirs(aq_dir)
      os.makedirs(sdk_proto_dir)
      with open(
          os.path.join(aq_dir, 'package.xml'), 'w', encoding='utf-8'
      ) as f:
        f.write('<package><name>aquarium</name></package>\n')
      with open(
          os.path.join(sdk_proto_dir, 'runtime_context.proto'),
          'w',
          encoding='utf-8',
      ) as f:
        f.write('syntax = "proto3";\npackage intrinsic_proto.config;\n')

      cfg_path = os.path.join(tmpdir, 'aquarium.yaml')
      with open(cfg_path, 'w', encoding='utf-8') as f:
        f.write(
            'target:\n'
            '  type: "node"\n'
            '  package: "demo_nodes_cpp"\n'
            '  name: "add_two_ints_server"\n'
            '  prune:\n'
            '    patterns:\n'
            '      - "/usr/include"\n'
            'interfaces:\n'
            '  - internal_interface: "ros_service"\n'
            '    internal_name: "/add_two_ints"\n'
            '    internal_type: "example_interfaces/srv/AddTwoInts"\n'
            '    external_interface: "zenoh_queryable_proto"\n'
            '    external_name: "add_two_ints"\n'
            '    type_hash: "RIHS01_dummy"\n'
            '    request_fields: ["int64 a", "int64 b"]\n'
            '    response_fields: ["int64 sum"]\n'
        )

      out_dir = os.path.join(tmpdir, 'build', 'aquarium_images')
      buf = io.StringIO()
      old_cwd = os.getcwd()
      try:
        os.chdir(tmpdir)
        with mock.patch('sys.stdout', buf):
          run_generate(cfg_path, output_dir=out_dir)
      finally:
        os.chdir(old_cwd)
      self.assertEqual(
          buf.getvalue().count('--build-arg PRUNE_PATTERNS="/usr/include"'), 2
      )

      # Missing buildah / protoc should exit cleanly instead of raising.
      old_cwd = os.getcwd()
      try:
        os.chdir(tmpdir)
        with mock.patch('aquarium.builder.shutil.which', return_value=None):
          with self.assertRaises(SystemExit):
            run_build(cfg_path)
      finally:
        os.chdir(old_cwd)

      txtpb = os.path.join(out_dir, 'demo-nodes-cpp-add-two-ints-server')
      txtpb += '_default_config.textproto'
      with mock.patch('aquarium.config.shutil.which', return_value=None):
        with self.assertRaises(SystemExit):
          run_config(txtpb, workspace_dir=tmpdir)


if __name__ == '__main__':
  unittest.main()
