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

from setuptools import find_packages, setup

package_name = 'intrinsic_sdk_bundle_library_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', [
            'resource/skill.Dockerfile',
            'resource/service.Dockerfile',
            'resource/sdk_version.json'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='William Woodall',
    maintainer_email='wjwwood@intrinsic.ai',
    description=(
        'Python build and bundle utilities for the Intrinsic SDK, used to create '
        'container images and bundles for custom skills and services.'
    ),
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'intrinsic_sdk_build = intrinsic_sdk_bundle_library_py.build:main',
        ],
    },
)
