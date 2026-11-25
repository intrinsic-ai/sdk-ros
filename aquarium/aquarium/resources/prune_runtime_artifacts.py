#!/usr/bin/env python3
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
"""Prunes non-runtime files from install trees using regex patterns.

This script removes build tools, static archives, header files, CMake metadata,
test binaries, and documentation from an install directory before copying it
into a distributable runtime container image.
"""

import os
import re
import shutil
import subprocess
import sys
from typing import List, Tuple

# Regex patterns targeting the largest unneeded files first.
IGNORE_PATTERNS: List[Tuple[str, str]] = [
    (
        'Build/compiler binaries',
        r'^(.*/)?(inbuild|bazelisk.*|flatc|protoc-gen-.*)$',
    ),
    (
        'Static archives and objects',
        r'.*\.([ao]|gch)$',
    ),
    (
        'Header directories and files',
        r'(.*/include(/.*)?$|.*\.(h|hpp|hxx|inl|inc)$)',
    ),
    (
        'CMake and pkgconfig metadata',
        r'(.*/(cmake|pkgconfig)(/.*)?$|.*\.(cmake|pc)$)',
    ),
    (
        'Test suites and test binaries',
        r'(.*/(test|tests)(/.*)?$|.*/test_[^/]+$|.*_test$|.*_gtest$)',
    ),
    (
        'Documentation and manual pages',
        r'(.*/(doc|docs|man|manual)(/.*)?$|.*/(README.*|CHANGELOG.*)$)',
    ),
    (
        'Python bytecode caches',
        r'(.*/__pycache__(/.*)?$|.*\.(pyc|pyo)$)',
    ),
]


def prune_directory(root_dir: str) -> None:
  """Walks root_dir and removes files and directories matching IGNORE_PATTERNS.

  Args:
    root_dir: Absolute or relative path to the directory to prune.
  """
  if not os.path.exists(root_dir):
    print(f'Target directory {root_dir} does not exist, skipping.')
    return

  compiled_patterns = [(desc, re.compile(pat)) for desc, pat in IGNORE_PATTERNS]

  total_files_removed = 0
  total_dirs_removed = 0
  total_bytes_freed = 0

  # First pass: remove matching directories bottom-up
  for dirpath, dirnames, _ in os.walk(root_dir, topdown=False):
    for dname in list(dirnames):
      full_dir_path = os.path.join(dirpath, dname)
      rel_path = os.path.relpath(full_dir_path, root_dir)
      matched = any(
          pattern.search(rel_path) or pattern.search(full_dir_path)
          for _, pattern in compiled_patterns
      )
      if matched:
        try:
          size = 0
          for p, _, files in os.walk(full_dir_path):
            for f in files:
              try:
                size += os.path.getsize(os.path.join(p, f))
              except OSError:
                pass
          shutil.rmtree(full_dir_path, ignore_errors=True)
          total_dirs_removed += 1
          total_bytes_freed += size
          dirnames.remove(dname)
        except OSError:
          pass

  # Second pass: remove matching files
  for dirpath, _, filenames in os.walk(root_dir):
    for fname in filenames:
      full_file_path = os.path.join(dirpath, fname)
      rel_path = os.path.relpath(full_file_path, root_dir)
      matched = any(
          pattern.search(rel_path) or pattern.search(full_file_path)
          for _, pattern in compiled_patterns
      )
      if matched:
        try:
          size = os.path.getsize(full_file_path)
          os.remove(full_file_path)
          total_files_removed += 1
          total_bytes_freed += size
        except OSError:
          pass

  # Third pass: strip remaining shared objects and binaries
  stripped_count = 0
  strip_bin = shutil.which('strip')
  if strip_bin:
    for dirpath, _, filenames in os.walk(root_dir):
      for fname in filenames:
        fpath = os.path.join(dirpath, fname)
        if os.path.islink(fpath):
          continue
        is_shlib = fname.endswith('.so') or '.so.' in fname
        if is_shlib or os.access(fpath, os.X_OK):
          try:
            res = subprocess.run(
                [strip_bin, '--strip-unneeded', fpath],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if res.returncode == 0:
              stripped_count += 1
          except OSError:
            pass

  freed_mb = total_bytes_freed / (1024 * 1024)
  print(
      f'PRUNER: Pruned {total_files_removed} files, '
      f'{total_dirs_removed} directories ({freed_mb:.1f} MB freed) '
      f'from {root_dir}.'
  )
  print(f'PRUNER: Stripped debug symbols from {stripped_count} binaries.')


def main() -> None:
  """CLI entry point for pruning directories."""
  if len(sys.argv) < 2:
    print(f'Usage: {sys.argv[0]} <directory_path> [...]', file=sys.stderr)
    sys.exit(1)

  for path in sys.argv[1:]:
    prune_directory(path)


if __name__ == '__main__':
  main()
