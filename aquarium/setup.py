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

from setuptools import find_packages
from setuptools import setup

package_name = 'aquarium'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    package_data={
        'aquarium': [
            'resources/*',
            'templates/*',
        ],
    },
    include_package_data=True,
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=False,
    maintainer='Morgan Quigley',
    maintainer_email='morganquigley@intrinsic.ai',
    description=(
        'Dynamic ROS 2 to Intrinsic Protobuf gateway, setup assistant TUI, '
        'and CLI orchestrator.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aquarium = aquarium.cli:main',
        ],
    },
)
