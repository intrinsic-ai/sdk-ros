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

import os
import re
import sys

def generate_echo_chain(requirements_content):
    lines = requirements_content.splitlines()
    echo_lines = []
    for i, line in enumerate(lines):
        # Escape single quotes for shell safety
        escaped_line = line.replace("'", "'\\''")
        if i == 0:
            echo_lines.append(f"RUN echo '{escaped_line}' > /tmp/requirements.txt \\")
        else:
            if i == len(lines) - 1:
                echo_lines.append(f" && echo '{escaped_line}' >> /tmp/requirements.txt")
            else:
                echo_lines.append(f" && echo '{escaped_line}' >> /tmp/requirements.txt \\")
    return "\n".join(echo_lines)

def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    requirements_path = os.path.join(repo_root, 'resources', 'pip_requirements', 'requirements.txt')

    if not os.path.exists(requirements_path):
        print(f"Error: requirements.txt not found at {requirements_path}")
        sys.exit(1)

    with open(requirements_path, 'r') as f:
        requirements_content = f.read().strip()

    dockerfiles = [
        os.path.join(repo_root, 'resources', 'Dockerfile.skill'),
        os.path.join(repo_root, 'intrinsic_sdk_bundle_library_py', 'resource', 'skill.Dockerfile')
    ]

    # Use robust sentinel comments to locate the block
    pattern = re.compile(r"(# BEGIN EMBEDDED REQUIREMENTS\n)(.*?)(\n# END EMBEDDED REQUIREMENTS)", re.DOTALL)

    # Fallback pattern to migrate from the old 'cat' format if still present
    fallback_pattern = re.compile(r"(RUN cat << 'EOF' > /tmp/requirements.txt\n)(.*?)(\nEOF)", re.DOTALL)

    replacement_content = generate_echo_chain(requirements_content)

    any_changed = False
    for dockerfile_path in dockerfiles:
        if not os.path.exists(dockerfile_path):
            print(f"Warning: Dockerfile not found at {dockerfile_path}, skipping.")
            continue

        with open(dockerfile_path, 'r') as f:
            content = f.read()

        has_sentinels = True
        match = pattern.search(content)
        if not match:
            # Try fallback to migrate
            match = fallback_pattern.search(content)
            if not match:
                print(f"Error: Could not find embedded requirements block or sentinels in {dockerfile_path}")
                sys.exit(1)
            has_sentinels = False

        existing_block = match.group(2).strip()

        # If we already have sentinels and the content matches, we are done
        if has_sentinels and existing_block == replacement_content:
            print(f"No changes needed for {os.path.relpath(dockerfile_path, repo_root)}")
            continue

        print(f"Updating embedded requirements in {os.path.relpath(dockerfile_path, repo_root)}")

        if has_sentinels:
            new_content = pattern.sub(lambda m: m.group(1) + replacement_content + m.group(3), content)
        else:
            # Migrate from old format to new format with sentinels
            print(f"Migrating {os.path.relpath(dockerfile_path, repo_root)} to sentinel comments format...")
            new_block = f"# BEGIN EMBEDDED REQUIREMENTS\n{replacement_content}\n# END EMBEDDED REQUIREMENTS"
            new_content = fallback_pattern.sub(new_block, content)

        with open(dockerfile_path, 'w') as f:
            f.write(new_content)
        any_changed = True

    if any_changed:
        print("Sync completed. Some files were updated.")
    else:
        print("Sync completed. All files are already up-to-date.")

if __name__ == '__main__':
    main()
