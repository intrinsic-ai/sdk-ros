#!/usr/bin/python3
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

CLUSTER_ZENOH_ENDPOINT = (
    'connect/endpoints='
    '["tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447"]'
)
LOCAL_ZENOH_ENDPOINT = 'connect/endpoints=["tcp/127.0.0.1:7447"]'


def is_running_in_cluster():
  if os.environ.get('KUBERNETES_SERVICE_HOST'):
    return True
  if os.path.exists('/etc/intrinsic/runtime_config.pb'):
    return True
  try:
    with open('/etc/resolv.conf', 'rt', encoding='utf-8') as f:
      for line in f:
        if line.startswith('search') and 'cluster.local' in line:
          return True
  except OSError:
    pass
  return False


if 'ZENOH_CONFIG_OVERRIDE' not in os.environ:
  if is_running_in_cluster():
    os.environ['ZENOH_CONFIG_OVERRIDE'] = CLUSTER_ZENOH_ENDPOINT
  else:
    os.environ['ZENOH_CONFIG_OVERRIDE'] = LOCAL_ZENOH_ENDPOINT

print(
    'SKILL ENTRYPOINT: ZENOH_CONFIG_OVERRIDE set to:'
    f" {os.environ.get('ZENOH_CONFIG_OVERRIDE')}",
    flush=True,
)

# Chain to /ros_entrypoint.sh passing all arguments.
# If arguments start with "-", prepend the default skill executable.
default_cmd = [
    '/skills/skill_service',
    '--skill_service_config_filename=/skills/skill_service_config.proto.bin',
]
if len(sys.argv) > 1 and sys.argv[1].startswith('-'):
  args = default_cmd + sys.argv[1:]
else:
  args = sys.argv[1:] if len(sys.argv) > 1 else default_cmd

cmd = ['/ros_entrypoint.sh'] + args
os.execvp(cmd[0], cmd)
