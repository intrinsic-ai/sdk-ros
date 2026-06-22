#!/usr/bin/env python3
"""
Helper script to translate the bazel build files for //incode/icon/no_absl to
CMake.

Running this script is the first step to export the absl-free shared-memory
comms layer from insrc to a Github repo.

# Inputs

## BUILD file hierarchy
The script reads the BUILD files in //incode/icon/no_absl and its
subdirectories. It does not require any special syntax.

## dependencies.json
The script requires a `dependencies.json` file to manage "external"
dependencies, i.e. those outside of the //incode/icon/no_absl directory.

This file must be a JSON object that maps each of the external dependencies to
an object that contains

* package_name: The name of the dependency to use with `find_package()`
* target_name: The name of the dependency to use with `target_link_libraries()`

Example `dependencies.json`:

```json
"@com_gitlab_libeigen_eigen//:eigen": {
  "package_name": "Eigen3",
  "target_name": "Eigen3::Eigen"
},
"@com_google_googletest//:gtest": {
  "package_name": "GTest",
  "target_name": "GTest::gtest"
},
```

# Outputs

The script creates a number of files:

* //CMakeLists.txt

  The main CMake config file, which

  * finds dependencies
  * sets global compiler options (C++ standard version, `-fPIE`)
  * creates Version.cmake and Config.cmake files to let other packages find the
    exported `icon_shared_memory` package
  * sets up package installation

* //flatbuffer_definitions/targets.cmake

  A CMake config file that contains targets for the few flatbuffer definitions
  **outside** the //incode/icon/no_absl tree that //incode/icon/no_absl depends
  on.

  These files are not duplicated in the //incode/icon/no_absl to prevent version
  skew.

* For each `BUILD` file under //incode/icon/no_absl, a `targets.cmake` file that
  contains the target(s) from that `BUILD` file

# Example use:

```bash
cd /path/to/insrc/incode/icon/no_absl
# Make sure to have a clean git workspace before this, so you can remove the
# generated CMake files after exporting.
python3 bazel_to_cmake.py

# Now run Copybara
cd /path/to/insrc
copybara \
  copy.bara.sky \
  no_absl_with_fbs \
  . \
  --folder-dir=/tmp/icon_no_absl_export

# Build the CMake package
# This requires:
# * Eigen3
# * GTest
# * flatbuffers
# * tl::expected
cd /tmp/icon_no_absl_export
mkdir build
cd build
cmake ..
make
```
"""

from collections import defaultdict
from dataclasses import dataclass
import json
import os
import subprocess
import sys


@dataclass
class BazelTarget:
  pkg_path: str
  target_type: str
  label: str
  srcs: list[str]
  hdrs: list[str]
  linkopts: list[str]
  deps: list[str]


@dataclass
class BazelTargetData:
  all_targets: dict[str, BazelTarget]
  targets_by_package_path: dict[str, list[str]]


@dataclass
class DependencyMapping:
  package_name: str
  target_name: str


EXTERNAL_FBS_TARGET_NAME = "icon_shared_memory_external_fbs_cc"


def handle_google3_flatbuffers(no_absl_root):
  # This command writes (to stdout) the paths of all .fbs files from //google3
  # that the targets in //incode/icon/no_absl depend on.
  #
  # N.B. these paths are relative to the workspace root
  bazel_cmd = [
      "bazel",
      "cquery",
      "--output",
      "files",
      (
          'filter(".*\\.fbs", filter("//google3/...",'
          ' deps("//...")))'
      ),
  ]

  # Sort the paths alphabetically and filter out any empty output lines
  fbs_file_paths = [
      p
      for p in sorted(
          subprocess.run(
              bazel_cmd,
              capture_output=True,
              text=True,
              check=True,
          ).stdout.split("\n")
      )
      if p
  ]

  # Drop the flatbuffer paths into flatbuffer_files.bara.sky
  flatbuffer_files_bara_sky_contents = ["FLATBUFFER_FILES = ["]
  flatbuffer_files_bara_sky_contents.extend(
      [f'    "{path}",' for path in fbs_file_paths]
  )
  flatbuffer_files_bara_sky_contents.append("]")
  bara_sky_path = os.path.join(no_absl_root, "flatbuffer_files.bara.sky")
  print(f"Writing {bara_sky_path}...")
  with open(bara_sky_path, "w", encoding="utf-8") as out:
    out.write("\n".join(flatbuffer_files_bara_sky_contents) + "\n")

  # Create flatbuffer_definitions/targets.cmake, with a *single* target that
  # converts *all* flatbuffer messages to headers.
  # This saves us the trouble of keeping track of dependency hierarchies between
  # flatbuffer definitions.
  fbs_cmake_lines = [
      "find_package(FlatBuffers REQUIRED)",
      "find_program(FLATC_EXECUTABLE flatc REQUIRED)",
      "",
      "if(NOT INSRC_ROOT)",
      (
          "  get_filename_component(INSRC_ROOT"
          ' "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)'
      ),
      "endif()",
      "",
  ]

  fbs_header_paths = []
  for fbs_file in sorted(fbs_file_paths):
    rel_fbs_path = fbs_file.removeprefix("flatbuffer_definitions/")
    pkg_path = os.path.dirname(rel_fbs_path)
    schema_name = os.path.basename(rel_fbs_path).removesuffix(".fbs")

    target_name = f"{schema_name}_fbs_cc"
    out_header = (
        f"${{CMAKE_CURRENT_BINARY_DIR}}/flatbuffer_definitions/{rel_fbs_path}.h"
    )

    # We need one invocation of `flatc` per flatbuffer definition, because
    # otherwise all of the headers would end up in the top-level
    # `flatbuffer_definitions` directory...
    fbs_cmake_lines.extend([
        "add_custom_command(",
        f'  OUTPUT "{out_header}"',
        (
            '  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs'
            " --include-prefix flatbuffer_definitions"
            " --keep-prefix"
            " --reflect-names"
            " --scoped-enums"
            " --gen-mutable"
            " --filename-ext h"
        ),
        (
            "          -o"
            f' "${{CMAKE_CURRENT_BINARY_DIR}}/flatbuffer_definitions/{pkg_path}"'
        ),
        '          -I "${CMAKE_CURRENT_LIST_DIR}/.."',
        f'          "${{CMAKE_CURRENT_LIST_DIR}}/{rel_fbs_path}"',
        f'  DEPENDS "${{CMAKE_CURRENT_LIST_DIR}}/{rel_fbs_path}"',
        f'  COMMENT "Generating C++ Flatbuffers headers for {schema_name}.fbs"',
        ")",
        f"add_library({target_name} INTERFACE)",
        f"target_include_directories({target_name} INTERFACE",
        '  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"',
        '  "$<INSTALL_INTERFACE:include>"',
        ")",
        f"target_sources({target_name} PRIVATE",
        f'  "{out_header}"',
        ")",
        f"target_link_libraries({target_name} INTERFACE",
        "  flatbuffers::flatbuffers",
        ")",
        f"install(TARGETS {target_name} EXPORT icon_shared_memoryTargets)",
        f'install(FILES "{out_header}"',
        f'        DESTINATION "include/flatbuffer_definitions/{pkg_path}"',
        ")",
        "",
    ])
    fbs_header_paths.append(out_header)

  # Finally, add "meta" build target that contains all flatbuffer headers
  fbs_cmake_lines.extend([
      f"add_library({EXTERNAL_FBS_TARGET_NAME} INTERFACE)",
      f"target_include_directories({EXTERNAL_FBS_TARGET_NAME} INTERFACE",
      '  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"',
      '  "$<INSTALL_INTERFACE:include>"',
      ")",
      f"target_sources({EXTERNAL_FBS_TARGET_NAME} PRIVATE",
  ])

  fbs_cmake_lines.extend(
      [f"    {fbs_header}" for fbs_header in fbs_header_paths]
  )
  fbs_cmake_lines.extend([
      ")",
      f"target_link_libraries({EXTERNAL_FBS_TARGET_NAME} INTERFACE",
      "  flatbuffers::flatbuffers",
      ")",
      (
          f"install(TARGETS {EXTERNAL_FBS_TARGET_NAME} EXPORT"
          " icon_shared_memoryTargets)"
      ),
  ])

  flatbuffers_dir = os.path.join(no_absl_root, "flatbuffer_definitions")
  os.makedirs(flatbuffers_dir, exist_ok=True)
  fbs_cmake_path = os.path.join(flatbuffers_dir, "targets.cmake")
  print(f"Writing {fbs_cmake_path}...")
  with open(fbs_cmake_path, "w", encoding="utf-8") as out:
    out.write("\n".join(fbs_cmake_lines))


def load_dependency_map(dependency_map_path) -> dict[str, DependencyMapping]:
  if not os.path.exists(dependency_map_path):
    print(
        f"Error: Mapping file {dependency_map_path} does not exist.",
        file=sys.stderr,
    )
    sys.exit(1)

  with open(dependency_map_path, encoding="utf-8") as f:
    return {
        k: DependencyMapping(
            package_name=v.get("package_name", ""),
            target_name=v.get("target_name", ""),
        )
        for (k, v) in json.load(f).items()
    }


def get_package_name_prefix(bazel_start_dir, bazel_workspace_root_dir):
  """
  Returns the common Bazel package name prefix for targets in `bazel_start_dir`, for a workspace rooted in `bazel_workspace_root_dir`.

  The return value does NOT have a final '/' character.
  """

  relative_start_dir = os.path.relpath(
      path=bazel_start_dir, start=bazel_workspace_root_dir
  )
  if relative_start_dir.startswith(".."):
    raise ValueError(
        f"bazel_start_dir ({bazel_start_dir}) must be below bazel_root_dir"
        f" ({bazel_workspace_root_dir}), but the path relative to"
        f" bazel_root_dir is {relative_start_dir}"
    )
  return ("//" + relative_start_dir).rstrip("/")


def collect_bazel_targets(bazel_start_dir, package_name_prefix):
  """Parses the BUILD files below `bazel_start_dir` to find targets we want to export.

  Supported target types:
  * cc_library
  * cc_binary
  * cc_test
  * flatbuffers_library
  * cc_flatbuffers_library

  This function uses `exec` to parse Bazel's Starlark configuration format,
  because the syntax of that is a subset of Python.

  Returns:
  A BazelTargetData object that contains an entry for each supported target.
  """

  def make_parsing_environment(package_path, target_data: BazelTargetData):
    def register_target_impl(type_name, *args, **kwargs):
      name = kwargs.get("name")
      if not name:
        return
      label = canonicalize_bazel_label(package_path, name, package_name_prefix)
      target_data.all_targets[label] = BazelTarget(
          pkg_path=package_path,
          target_type=type_name,
          label=label,
          srcs=kwargs.get("srcs", []),
          hdrs=kwargs.get("hdrs", []),
          linkopts=kwargs.get("linkopts", []),
          deps=[
              canonicalize_bazel_label(package_path, dep, package_name_prefix)
              for dep in kwargs.get("deps", [])
          ],
      )
      if not package_path in target_data.targets_by_package_path:
        target_data.targets_by_package_path[package_path] = []
      target_data.targets_by_package_path[package_path].append(label)

    def register_target(target_type_name):
      return lambda *args, **kwargs: register_target_impl(
          target_type_name, *args, **kwargs
      )

    def no_op(*args, **kwargs):
      pass

    return defaultdict(
        lambda: no_op,
        {
            "cc_library": register_target("cc_library"),
            "cc_binary": register_target("cc_binary"),
            "cc_test": register_target("cc_test"),
            "flatbuffers_library": register_target("flatbuffers_library"),
            "cc_flatbuffers_library": register_target("cc_flatbuffers_library"),
        },
    )

  target_data = BazelTargetData(all_targets={}, targets_by_package_path={})
  # Walk directory to find and parse all BUILD files
  for current_dir, _, files in os.walk(bazel_start_dir):
    if "BUILD" not in files:
      continue

    build_path = os.path.join(current_dir, "BUILD")
    # Calculate package path relative to no_absl_root
    relative_package_path = os.path.relpath(current_dir, bazel_start_dir)
    if relative_package_path == ".":
      relative_package_path = ""

    with open(build_path, "r", encoding="utf-8") as f:
      build_content = f.read()

    parsing_environment = make_parsing_environment(
        relative_package_path, target_data
    )
    try:
      # Execute BUILD file as python code in our mock environment
      # pylint: disable-next=exec-used
      exec(build_content, parsing_environment)
    # pylint: disable-next=broad-exception-caught
    except Exception as e:
      print(f"Error parsing {build_path}: {e}", file=sys.stderr)
      sys.exit(1)
  return target_data


def canonicalize_bazel_label(package_path, label, package_name_prefix):
  """
  Converts a Bazel label to its fully-qualified canonical form relative to the
  workspace root.

  Examples:
    - ':status' under 'icon/utils' -> '//icon/utils:status'
    - 'status' under 'icon/utils' -> '//icon/utils:status'
    - '//icon/utils' -> '//icon/utils:utils'
    - '//google3/foo:bar' -> '//google3/foo:bar'
  """
  if label.startswith(package_name_prefix):
    # Label is qualified and inside the directory we're exporting, but may omit
    # the target name if it matches the package name
    parts = label.split(":")
    if len(parts) == 2:
      return label
    target_name = label.split("/")[-1]
    return f"{label}:{target_name}"
  if label.startswith("//") or label.startswith("@"):
    # Label is fully qualified but outside the directory we're exporting. Leave
    # unchanged so that `dependencies.json` substitutions work.
    return label
  if label.startswith(":"):
    # Local label reference, add package_name_prefix and package path
    if package_path:
      return f"{package_name_prefix}/{package_path}{label}"
    return f"{package_name_prefix}{label}"

  # Local label name, add package_name_prefix and package path, **and** add
  # colon before the label.
  if package_path:
    return f"{package_name_prefix}/{package_path}:{label}"
  return f"{package_name_prefix}:{label}"


def label_to_cmake_target(
    canonical_label: str,
    package_name_prefix: str,
    external_dependency_map: dict[str, DependencyMapping],
) -> str:
  """
  Translates a canonical Bazel label to the corresponding CMake target name.

  Rules:
    - If the label is mapped in dependencies.json, return the mapped target name.
    - If the label is a local target (in the subtree), map it to the form
      'icon_shared_memory_<pkg_path_underscores>_<target_name>'.
      If target_name is identical to the last package path component, simplify to
      'icon_shared_memory_<pkg_path_underscores>'.
  """
  if canonical_label in external_dependency_map:
    cmake_target = external_dependency_map[canonical_label].target_name
    return cmake_target
  if canonical_label.startswith(
      "//flatbuffer_definitions/"
  ) and canonical_label.endswith("_fbs_cc"):
    # All external flatbuffers become one big library target (see
    # `handle_google3_flatbuffers()`)
    return EXTERNAL_FBS_TARGET_NAME
  if canonical_label.startswith(package_name_prefix):
    parts = (
        canonical_label.removeprefix(package_name_prefix).lstrip("/").split(":")
    )
    package = parts[0]
    target = parts[1]
    package_underscores = package.replace("/", "_")
    last_package_component = package.split("/")[-1]
    if target == last_package_component:
      return f"icon_shared_memory_{package_underscores}"
    else:
      return f"icon_shared_memory_{package_underscores}_{target}"
  else:
    raise ValueError(
        f"External target {canonical_label} is not mapped in dependencies.json!"
    )


def package_names_in_topological_order(
    target_data: BazelTargetData, package_name_prefix: str
):
  package_name_to_deps = defaultdict(set)
  for target in target_data.all_targets.values():
    pkg = target.pkg_path
    for dep in target.deps:
      # Only consider packages in the subtree we're exporting.
      # External deps, including the flatbuffers we deal with in
      # `handle_google3_flatbuffers()`, are loaded first anyway, and do not have
      # reverse deps into the subtree.
      if not dep.startswith(package_name_prefix):
        continue
      dep_pkg = target_data.all_targets[dep].pkg_path
      if dep_pkg != pkg:
        package_name_to_deps[pkg].add(dep_pkg)

  # Topological sort DFS
  sorted_packages = []
  UNVISITED = 0
  VISITING = 1
  VISITED = 2
  visited = {}

  def dfs(p):
    visited[p] = VISITING
    for dep_pkg in sorted(package_name_to_deps[p]):
      visited_this = visited.get(dep_pkg, UNVISITED)
      if visited_this == VISITING:
        print(
            f"Warning: Circular dependency detected between packages {p} and"
            f" {dep_pkg}",
            file=sys.stderr,
        )
      elif visited_this != VISITED:
        dfs(dep_pkg)
    visited[p] = VISITED
    sorted_packages.append(p)

  # Sort packages alphabetically first for determinism
  for p in sorted(target_data.targets_by_package_path.keys()):
    if visited.get(p, UNVISITED) != VISITED:
      dfs(p)
  return sorted_packages


def generate_cmake_targets_file(
    package_name: str,
    target_data: BazelTargetData,
    bazel_start_dir: str,
    package_name_prefix: str,
    dependency_map: dict[str, DependencyMapping],
):
  package_targets = target_data.targets_by_package_path.get(package_name, [])
  if not package_targets:
    return

  cmake_lines = []
  cmake_lines.append(
      f"# Automatically generated from BUILD in {package_name or "."}"
  )
  cmake_lines.append("")

  for label in package_targets:
    target = target_data.all_targets[label]
    t_type = target.target_type
    srcs = target.srcs
    hdrs = target.hdrs
    deps = target.deps
    linkopts = target.linkopts

    target_name = label_to_cmake_target(
        label, package_name_prefix, dependency_map
    )

    # Map deps
    cmake_deps = []
    for dep in deps:
      cmake_deps.append(
          label_to_cmake_target(dep, package_name_prefix, dependency_map)
      )
    for opt in linkopts:
      if opt.startswith("-l"):
        cmake_deps.append(opt[2:])
      else:
        cmake_deps.append(opt)

    # Destination directory path for header installation
    install_dest = f"include/{package_name}".rstrip("/")

    cpp_extensions = (".cc", ".cpp", ".c", ".cxx", ".C")

    if t_type == "cc_library":
      # Check header-only
      cpp_srcs = [s for s in srcs if s.endswith(cpp_extensions)]
      is_header_only = not cpp_srcs

      if is_header_only:
        cmake_lines.append(f"add_library({target_name} INTERFACE)")
        if hdrs:
          cmake_lines.append(f"target_sources({target_name} PRIVATE")
          for h in sorted(hdrs):
            cmake_lines.append(f'  "${{CMAKE_CURRENT_LIST_DIR}}/{h}"')
          cmake_lines.append(")")
        cmake_lines.append(
            f"target_include_directories({target_name} INTERFACE"
        )
        cmake_lines.append('  "$<BUILD_INTERFACE:${INSRC_ROOT}>"')
        cmake_lines.append('  "$<INSTALL_INTERFACE:include>"')
        cmake_lines.append(")")
        if cmake_deps:
          cmake_lines.append(f"target_link_libraries({target_name} INTERFACE")
          for d in cmake_deps:
            cmake_lines.append(f"  {d}")
          cmake_lines.append(")")
      else:
        all_files = sorted(srcs + hdrs)
        cmake_lines.append(f"add_library({target_name} STATIC")
        for f in all_files:
          cmake_lines.append(f'  "${{CMAKE_CURRENT_LIST_DIR}}/{f}"')
        cmake_lines.append(")")
        cmake_lines.append(f"target_include_directories({target_name} PUBLIC")
        cmake_lines.append('  "$<BUILD_INTERFACE:${INSRC_ROOT}>"')
        cmake_lines.append('  "$<INSTALL_INTERFACE:include>"')
        cmake_lines.append(")")
        if cmake_deps:
          cmake_lines.append(f"target_link_libraries({target_name} PUBLIC")
          for d in cmake_deps:
            cmake_lines.append(f"  {d}")
          cmake_lines.append(")")

      # Install target
      cmake_lines.append(f"install(TARGETS {target_name}")
      cmake_lines.append("        EXPORT icon_shared_memoryTargets")
      cmake_lines.append("        LIBRARY DESTINATION lib")
      cmake_lines.append("        ARCHIVE DESTINATION lib")
      cmake_lines.append("        RUNTIME DESTINATION bin")
      cmake_lines.append("        INCLUDES DESTINATION include")
      cmake_lines.append(")")

      # Install associated headers
      if hdrs:
        cmake_lines.append("install(FILES")
        for h in sorted(hdrs):
          cmake_lines.append(f'        "${{CMAKE_CURRENT_LIST_DIR}}/{h}"')
        cmake_lines.append(f'        DESTINATION "{install_dest}"')
        cmake_lines.append(")")

      cmake_lines.append("")

    elif t_type == "cc_binary":
      all_files = sorted(srcs + hdrs)
      cmake_lines.append(f"add_executable({target_name}")
      for f in all_files:
        cmake_lines.append(f'  "${{CMAKE_CURRENT_LIST_DIR}}/{f}"')
      cmake_lines.append(")")
      cmake_lines.append(
          f'target_include_directories({target_name} PRIVATE "${{INSRC_ROOT}}")'
      )
      if cmake_deps:
        cmake_lines.append(f"target_link_libraries({target_name} PRIVATE")
        for d in cmake_deps:
          cmake_lines.append(f"  {d}")
        cmake_lines.append(")")
      cmake_lines.append(
          f"install(TARGETS {target_name} RUNTIME DESTINATION bin)"
      )
      cmake_lines.append("")

    elif t_type == "cc_test":
      all_files = sorted(srcs + hdrs)
      cmake_lines.append("if(BUILD_TESTING)")
      cmake_lines.append(f"  add_executable({target_name}")
      for f in all_files:
        cmake_lines.append(f'    "${{CMAKE_CURRENT_LIST_DIR}}/{f}"')
      cmake_lines.append("  )")
      cmake_lines.append(
          f"  target_include_directories({target_name} PRIVATE"
          ' "${INSRC_ROOT}")'
      )
      if cmake_deps:
        cmake_lines.append(f"  target_link_libraries({target_name} PRIVATE")
        for d in cmake_deps:
          cmake_lines.append(f"    {d}")
        cmake_lines.append("  )")
      cmake_lines.append(f"  gtest_add_tests(TARGET {target_name})")
      cmake_lines.append("endif()")
      cmake_lines.append("")

    elif t_type == "flatbuffers_library":
      # Just install original schemas
      if srcs:
        cmake_lines.append("install(FILES")
        for s in sorted(srcs):
          cmake_lines.append(f'        "${{CMAKE_CURRENT_LIST_DIR}}/{s}"')
        cmake_lines.append(f'        DESTINATION "{install_dest}"')
        cmake_lines.append(")")
        cmake_lines.append("")

    elif t_type == "cc_flatbuffers_library":
      # Compile schemas of dependencies
      fbs_srcs = []
      for dep in deps:
        dep_target = target_data.all_targets.get(dep)
        if dep_target and dep_target.target_type == "flatbuffers_library":
          fbs_srcs.extend(dep_target.srcs)

      # Define custom commands and output targets for each .fbs schema file
      outputs = []
      for fbs in sorted(fbs_srcs):
        fbs_base = os.path.splitext(fbs)[0]
        out_header = f"${{CMAKE_CURRENT_BINARY_DIR}}/{package_name}/{fbs_base}.fbs.h"
        outputs.append(out_header)

        cmake_lines.append("add_custom_command(")
        cmake_lines.append(f'  OUTPUT "{out_header}"')
        cmake_lines.append(
            '  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs'
            " --keep-prefix"
            " --reflect-names"
            " --scoped-enums"
            " --gen-mutable"
            " --filename-ext h"
        )
        cmake_lines.append(
            "          -o"
            f' "${{CMAKE_CURRENT_BINARY_DIR}}/{package_name}"'
        )
        cmake_lines.append('          -I "${INSRC_ROOT}"')
        cmake_lines.append(f'          "${{CMAKE_CURRENT_LIST_DIR}}/{fbs}"')
        cmake_lines.append(f'  DEPENDS "${{CMAKE_CURRENT_LIST_DIR}}/{fbs}"')
        cmake_lines.append(
            f'  COMMENT "Generating C++ Flatbuffers headers for {fbs}"'
        )
        cmake_lines.append(")")

      # Interface Library Target
      cmake_lines.append(f"add_library({target_name} INTERFACE)")
      cmake_lines.append(f"target_include_directories({target_name} INTERFACE")
      cmake_lines.append('  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"')
      cmake_lines.append('  "$<INSTALL_INTERFACE:include>"')
      cmake_lines.append(")")
      if outputs:
        cmake_lines.append(f"target_sources({target_name} PRIVATE")
        for o in outputs:
          cmake_lines.append(f'  "{o}"')
        cmake_lines.append(")")

      cmake_lines.append(f"target_link_libraries({target_name} INTERFACE")
      cmake_lines.append("  flatbuffers::flatbuffers")
      cmake_lines.append(")")
      cmake_lines.append(f"install(TARGETS {target_name}")
      cmake_lines.append("        EXPORT icon_shared_memoryTargets")
      cmake_lines.append(")")

      # Install generated flatbuffers headers
      if outputs:
        cmake_lines.append("install(FILES")
        for o in outputs:
          cmake_lines.append(f'        "{o}"')
        cmake_lines.append(f'        DESTINATION "{install_dest}"')
        cmake_lines.append(")")

      cmake_lines.append("")

  # Write targets.cmake in the subdirectory
  pkg_dir = os.path.join(bazel_start_dir, package_name)
  os.makedirs(pkg_dir, exist_ok=True)
  cmake_file_path = os.path.join(pkg_dir, "targets.cmake")
  print(f"Writing {cmake_file_path}...")
  with open(cmake_file_path, "w", encoding="utf-8") as out:
    out.write("\n".join(cmake_lines))


def main():
  script_dir = os.path.dirname(os.path.abspath(__file__))
  no_absl_root = script_dir

  handle_google3_flatbuffers(no_absl_root)

  # 1. Load dependencies mapping
  deps_map_path = os.path.join(no_absl_root, "dependencies.json")
  external_dependency_map = load_dependency_map(deps_map_path)

  package_name_prefix = get_package_name_prefix(
      no_absl_root, no_absl_root + "/../../.."
  )

  target_data = collect_bazel_targets(no_absl_root, package_name_prefix)

  sorted_packages = package_names_in_topological_order(
      target_data, package_name_prefix
  )

  # 5. Generate targets.cmake for each package
  for package_name in sorted_packages:
    generate_cmake_targets_file(
        package_name,
        target_data,
        no_absl_root,
        package_name_prefix,
        external_dependency_map,
    )

  # 7. Add find_package() lines for external dependencies
  packages_to_find = set()

  for t in sorted(
      [dep.package_name for dep in external_dependency_map.values()]
  ):
    packages_to_find.add(t)

  packages_to_find.add("FlatBuffers")

  # 8. Generate the central CMakeLists.txt
  root_cmake_lines = [
      "cmake_minimum_required(VERSION 3.19)",
      "project(icon_shared_memory CXX)",
      "",
      "set(CMAKE_CXX_STANDARD 20)",
      "set(CMAKE_CXX_STANDARD_REQUIRED ON)",
      "",
      "include(CMakePackageConfigHelpers)",
      "include(GNUInstallDirs)",
      "",
      (
          "# -fPIC is required if these libraries are going to be linked into a"
          " plugin"
      ),
      "set(CMAKE_POSITION_INDEPENDENT_CODE ON)",
      "",
      "# 1. Setup packages and tools",
  ]

  for package_name in sorted(packages_to_find):
    root_cmake_lines.append(f"find_package({package_name} REQUIRED)")
  root_cmake_lines.append("find_program(FLATC_EXECUTABLE flatc REQUIRED)")

  root_cmake_lines.append(
      'include("${CMAKE_CURRENT_SOURCE_DIR}/flatbuffer_definitions/targets.cmake")',
  )

  root_cmake_lines.extend([
      "",
      "enable_testing()",
      "include(GoogleTest)",
      "",
      "# Define repository root",
      (
          'get_filename_component(INSRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}"'
          " ABSOLUTE)"
      ),
      "",
      "# 2. Include targets in topological dependency order",
  ])

  for pkg in sorted_packages:
    if pkg:
      root_cmake_lines.append(
          f'include("${{CMAKE_CURRENT_LIST_DIR}}/{pkg}/targets.cmake")'
      )
    else:
      root_cmake_lines.append(
          'include("${CMAKE_CURRENT_LIST_DIR}/targets.cmake")'
      )

  root_cmake_lines.extend([
      "",
      "# 3. Generate & Install CMake Package Files",
      "write_basic_package_version_file(",
      '  "${CMAKE_CURRENT_BINARY_DIR}/icon_shared_memoryConfigVersion.cmake"',
      "  VERSION 1.0.0",
      "  COMPATIBILITY AnyNewerVersion",
      ")",
      "",
  ])

  # Generate config file dynamically mapping packages_to_find and internal_fbs_packages
  config_file_lines = ["include(CMakeFindDependencyMacro)"]
  for package_name in sorted(packages_to_find):
    config_file_lines.append(f"find_dependency({package_name})")

  config_file_lines.append(
      # We need to escape the quotes because this line is passed to CMake's
      # `file()` function.
      # We also need to escape the variable expansion, because we want to
      # evaluate CMAKE_CURRENT_LIST_DIR when `icon_shared_memoryConfig.cmake`
      # is evaluated, not when CMakeLists.txt is.
      'include(\\"\\$\\{CMAKE_CURRENT_LIST_DIR\\}/icon_shared_memoryTargets.cmake\\")'
  )

  root_cmake_lines.append(
      'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/icon_shared_memoryConfig.cmake"'
  )
  for cl in config_file_lines:
    root_cmake_lines.append(f'  "{cl}\\n"')
  root_cmake_lines.append(")")

  root_cmake_lines.extend([
      "",
      "install(EXPORT icon_shared_memoryTargets",
      "        FILE icon_shared_memoryTargets.cmake",
      "        DESTINATION lib/cmake/icon_shared_memory",
      ")",
      "",
      "install(FILES",
      '  "${CMAKE_CURRENT_BINARY_DIR}/icon_shared_memoryConfig.cmake"',
      '  "${CMAKE_CURRENT_BINARY_DIR}/icon_shared_memoryConfigVersion.cmake"',
      "  DESTINATION lib/cmake/icon_shared_memory",
      ")",
  ])

  root_cmake_path = os.path.join(no_absl_root, "CMakeLists.txt")
  print(f"Writing {root_cmake_path}...")
  with open(root_cmake_path, "w", encoding="utf-8") as out:
    out.write("\n".join(root_cmake_lines))


if __name__ == "__main__":
  main()
