import os
import subprocess
import sys

if not os.path.exists('pybind11_abseil'):
    subprocess.check_call(['git', 'clone', 'https://github.com/pybind/pybind11_abseil.git', '-b', 'v202402.0'])

subprocess.call(' '.join([
        'diff', '-u', 'pybind11_abseil/CMakeLists.txt', 'CMakeLists.txt', '>', '../patches/cmakelists.patch'
    ]),
    shell=True)

# subprocess.call(' '.join([
#         'diff', '-u', '/dev/null', 'pybind11_abseilConfig.cmake', '>', '../patches/pybind11_abseilConfig.patch'
#     ]),
#     shell=True)

# subprocess.call(' '.join([
#         'diff', '-u', '/dev/null', 'Config.cmake.in', '>', '../patches/Config.patch'
#     ]),
#     shell=True)
