#!/usr/bin/env python3
from collections import defaultdict
import json
import os
import sys


def main():
  script_dir = os.path.dirname(os.path.abspath(__file__))
  no_absl_root = script_dir

  # 1. Load dependencies mapping
  deps_map_path = os.path.join(no_absl_root, "dependencies.json")
  if not os.path.exists(deps_map_path):
    print(
        f"Error: Mapping file {deps_map_path} does not exist.", file=sys.stderr
    )
    sys.exit(1)

  with open(deps_map_path) as f:
    external_mapping = json.load(f)

  # Global target and package registry
  # targets maps canonical label to target kwargs
  targets = {}
  # packages maps relative package path to list of canonical labels in it
  packages = defaultdict(list)

  # Helper function to create mock environment for exec
  def make_mock_env(pkg_path):
    def register_target(type_name, **kwargs):
      name = kwargs.get("name")
      if not name:
        return
      label = (
          f"//{pkg_path}:{name}"
          if pkg_path
          else f"//incode/icon/no_absl:{name}"
      )
      kwargs["type"] = type_name
      kwargs["pkg_path"] = pkg_path
      kwargs["label"] = label
      targets[label] = kwargs
      packages[pkg_path].append(label)

    return {
        "cc_library": lambda **kwargs: register_target("cc_library", **kwargs),
        "cc_binary": lambda **kwargs: register_target("cc_binary", **kwargs),
        "cc_test": lambda **kwargs: register_target("cc_test", **kwargs),
        "flatbuffers_library": lambda **kwargs: register_target(
            "flatbuffers_library", **kwargs
        ),
        "cc_flatbuffers_library": lambda **kwargs: register_target(
            "cc_flatbuffers_library", **kwargs
        ),
        "load": lambda *args, **kwargs: None,
        "package": lambda *args, **kwargs: None,
        "package_group": lambda *args, **kwargs: None,
    }

  # 2. Walk directory to find and parse all BUILD files
  for root, _, files in os.walk(no_absl_root):
    if "BUILD" in files:
      build_path = os.path.join(root, "BUILD")
      # Calculate package path relative to no_absl_root
      rel_pkg_path = os.path.relpath(root, no_absl_root)
      if rel_pkg_path == ".":
        rel_pkg_path = ""

      with open(build_path, "r") as f:
        build_content = f.read()

      mock_env = make_mock_env(rel_pkg_path)
      try:
        # Execute BUILD file as python code in our mock environment
        exec(build_content, mock_env)
      except Exception as e:
        print(f"Error parsing {build_path}: {e}", file=sys.stderr)
        sys.exit(1)

  # 3. Target Label Resolution & Naming Helpers
  def canonicalize_label(package_path, label):
    """
    Converts a Bazel label to its fully-qualified canonical form relative to the
    workspace root.

    Examples:
      - ':status' under 'icon/utils' -> '//icon/utils:status'
      - 'status' under 'icon/utils' -> '//icon/utils:status'
      - '//icon/utils' -> '//icon/utils:utils'
      - '//google3/foo:bar' -> '//google3/foo:bar'
    """
    if label.startswith("//"):
      parts = label.removeprefix("//").split(":")
      if len(parts) == 1:
        pkg = parts[0]
        target = pkg.split("/")[-1]
      else:
        pkg, target = parts[0], parts[1]
      return f"//{pkg}:{target}"
    elif label.startswith("//") or label.startswith("@"):
      return label
    elif label.startswith(":"):
      target = label[1:]
      return (
          f"//{package_path}:{target}"
          if package_path
          else f"//incode/icon/no_absl:{target}"
      )
    else:
      return (
          f"//{package_path}:{label}"
          if package_path
          else f"//incode/icon/no_absl:{label}"
      )

  # Set to collect external CMake targets that are actually referenced
  used_external_target_package_names = set()

  def label_to_cmake_target(canonical_label):
    """
    Translates a canonical Bazel label to the corresponding CMake target name.

    Rules:
      - If the label is mapped in dependencies.json, return the mapped value and track it.
      - If the label is a local target (in the subtree), map it to the form
        'icon_shared_memory_<pkg_path_underscores>_<target_name>'.
        If target_name is identical to the last package path component, simplify to
        'icon_shared_memory_<pkg_path_underscores>'.
    """
    if canonical_label in external_mapping:
      cmake_target = external_mapping[canonical_label]["target_name"]
      used_external_target_package_names.add(
          external_mapping[canonical_label]["package_name"]
      )
      return cmake_target
    if canonical_label.startswith("//"):
      parts = canonical_label.removeprefix("//").split(":")
      pkg = parts[0]
      target = parts[1]
      pkg_clean = pkg.replace("/", "_")
      last_pkg_component = pkg.split("/")[-1]
      if target == last_pkg_component:
        return f"icon_shared_memory_{pkg_clean}"
      else:
        return f"icon_shared_memory_{pkg_clean}_{target}"
    else:
      raise ValueError(
          f"External target {canonical_label} is not mapped in"
          " dependencies.json!"
      )

  def canonical_label_to_cmake_alias(canonical_label):
    """
    Generates a modern C++ namespace-like alias for local targets.

    Example:
      - '//icon/utils:status' -> 'icon::utils::status'
    """
    if canonical_label.startswith("//"):
      parts = canonical_label.removeprefix("//").split(":")
      pkg = parts[0]
      target = parts[1]
      pkg_alias = pkg.replace("/", "::")
      last_pkg_component = pkg.split("/")[-1]
      if target == last_pkg_component:
        return f"{pkg_alias}"
      else:
        return f"{pkg_alias}::{target}"
    return canonical_label

  # 4. Topological Sort of Packages (Subdirectories)
  # Construct package dependency graph and collect external flatbuffers files
  external_fbs_files = set()
  pkg_deps = defaultdict(set)
  for label, target in targets.items():
    pkg = target["pkg_path"]
    for dep in target.get("deps", []):
      canonical_dep = canonicalize_label(pkg, dep)
      if canonical_dep.startswith("//"):
        dep_pkg = targets[canonical_dep]["pkg_path"]
        if dep_pkg != pkg:
          pkg_deps[pkg].add(dep_pkg)
      elif canonical_dep.startswith(
          "//flatbuffer_definitions/"
      ) and canonical_dep.endswith("_fbs_cc"):
        dep_without_slash = canonical_dep.removeprefix("//")
        parts = dep_without_slash.split(":")
        pkg_path = parts[0]
        target_name = parts[1]
        schema_name = target_name.removesuffix("_fbs_cc")
        fbs_file = f"{pkg_path}/{schema_name}.fbs"
        external_fbs_files.add(fbs_file)

  # Write flatbuffer_files.bara.sky
  bara_sky_lines = ["FLATBUFFER_FILES = ["]
  for f in sorted(external_fbs_files):
    bara_sky_lines.append(f'    "{f}",')
  bara_sky_lines.append("]")

  bara_sky_path = os.path.join(no_absl_root, "flatbuffer_files.bara.sky")
  print(f"Writing {bara_sky_path}...")
  with open(bara_sky_path, "w") as out:
    out.write("\n".join(bara_sky_lines) + "\n")

  # Topological sort DFS
  sorted_packages = []
  visited = {}  # None: unvisited, 1: visiting, 2: visited

  def dfs(p):
    visited[p] = 1
    for dep_pkg in sorted(pkg_deps[p]):
      if visited.get(dep_pkg) == 1:
        print(
            f"Warning: Circular dependency detected between packages {p} and"
            f" {dep_pkg}",
            file=sys.stderr,
        )
      elif visited.get(dep_pkg) != 2:
        dfs(dep_pkg)
    visited[p] = 2
    sorted_packages.append(p)

  # Sort packages alphabetically first for determinism
  for p in sorted(packages.keys()):
    if visited.get(p) != 2:
      dfs(p)

  # Track if flatbuffers compiler is actually needed locally
  needs_flatbuffers = False

  # 5. Generate targets.cmake for each package
  cpp_extensions = (".cc", ".cpp", ".c", ".cxx", ".C")

  for pkg in sorted_packages:
    package_targets = packages[pkg]
    if not package_targets:
      continue

    cmake_lines = []
    cmake_lines.append(f"# Automatically generated from BUILD in {pkg or '.'}")
    cmake_lines.append("")

    for label in package_targets:
      target = targets[label]
      t_type = target["type"]
      name = target["name"]
      srcs = target.get("srcs", [])
      hdrs = target.get("hdrs", [])
      deps = target.get("deps", [])
      linkopts = target.get("linkopts", [])

      target_name = label_to_cmake_target(label)

      # Map deps
      cmake_deps = []
      for d in deps:
        canonical_d = canonicalize_label(pkg, d)
        cmake_deps.append(label_to_cmake_target(canonical_d))
      for opt in linkopts:
        if opt.startswith("-l"):
          cmake_deps.append(opt[2:])
        else:
          cmake_deps.append(opt)

      # Destination directory path for header installation
      install_dest = (
          f"include/{pkg}"
          if pkg
          else "include/incode/icon/no_absl"
      )

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

        # Define modern namespace alias
        alias_name = canonical_label_to_cmake_alias(label)
        if alias_name != target_name:
          cmake_lines.append(f"add_library({alias_name} ALIAS {target_name})")
        cmake_lines.append("")

      elif t_type == "cc_binary":
        all_files = sorted(srcs + hdrs)
        cmake_lines.append(f"add_executable({target_name}")
        for f in all_files:
          cmake_lines.append(f'  "${{CMAKE_CURRENT_LIST_DIR}}/{f}"')
        cmake_lines.append(")")
        cmake_lines.append(
            f"target_include_directories({target_name} PRIVATE"
            ' "${INSRC_ROOT}")'
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
        needs_flatbuffers = True
        # Compile schemas of dependencies
        fbs_srcs = []
        for d in deps:
          canonical_d = canonicalize_label(pkg, d)
          dep_target = targets.get(canonical_d)
          if dep_target and dep_target["type"] == "flatbuffers_library":
            fbs_srcs.extend(dep_target.get("srcs", []))

        # Define custom commands and output targets for each .fbs schema file
        outputs = []
        for fbs in sorted(fbs_srcs):
          fbs_base = os.path.splitext(fbs)[0]
          out_header = f"${{CMAKE_CURRENT_BINARY_DIR}}/{pkg}/{fbs_base}.fbs.h"
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
              f' "${{CMAKE_CURRENT_BINARY_DIR}}/{pkg}"'
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
        cmake_lines.append(
            f"target_include_directories({target_name} INTERFACE"
        )
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

        alias_name = canonical_label_to_cmake_alias(label)
        if alias_name != target_name:
          cmake_lines.append(f"add_library({alias_name} ALIAS {target_name})")
        cmake_lines.append("")

    # Write targets.cmake in the subdirectory
    pkg_dir = os.path.join(no_absl_root, pkg)
    os.makedirs(pkg_dir, exist_ok=True)
    cmake_file_path = os.path.join(pkg_dir, "targets.cmake")
    print(f"Writing {cmake_file_path}...")
    with open(cmake_file_path, "w") as out:
      out.write("\n".join(cmake_lines))

  # 6. Generate flatbuffers/targets.cmake for exported flatbuffers
  if external_fbs_files:
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

    for fbs_file in sorted(external_fbs_files):
      rel_fbs_path = fbs_file.removeprefix("flatbuffer_definitions/")
      pkg_path = os.path.dirname(rel_fbs_path)
      schema_name = os.path.basename(rel_fbs_path).removesuffix(".fbs")

      target_name = f"{schema_name}_fbs_cc"
      out_header = f"${{CMAKE_CURRENT_BINARY_DIR}}/flatbuffer_definitions/{pkg_path}/{schema_name}.fbs.h"

      fbs_cmake_lines.extend([
          f"# Target: {target_name}",
          "add_custom_command(",
          f'  OUTPUT "{out_header}"',
          (
              '  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs'
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
          (
              '  COMMENT "Generating C++ Flatbuffers headers for'
              f' {schema_name}.fbs"'
          ),
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
          (
              f"add_library({pkg_path.replace('/', '::')}::{schema_name}_fbs_cc"
              f" ALIAS {target_name})"
          ),
          "",
      ])

    flatbuffers_dir = os.path.join(no_absl_root, "flatbuffer_definitions")
    os.makedirs(flatbuffers_dir, exist_ok=True)
    fbs_cmake_path = os.path.join(flatbuffers_dir, "targets.cmake")
    print(f"Writing {fbs_cmake_path}...")
    with open(fbs_cmake_path, "w") as out:
      out.write("\n".join(fbs_cmake_lines))

  # 7. Resolve Packages dynamically based on used external targets
  packages_to_find = set()
  internal_fbs_packages = set()

  for t in sorted(used_external_target_package_names):
    if t.endswith("_fbs_cc"):
      internal_fbs_packages.add(t)
    else:
      packages_to_find.add(t)

  if needs_flatbuffers or internal_fbs_packages:
    packages_to_find.add("flatbuffers")

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
      "# 1. Setup packages and tools",
  ]

  for pkg_find in sorted(packages_to_find):
    root_cmake_lines.append(f"find_package({pkg_find} REQUIRED)")
  if needs_flatbuffers or internal_fbs_packages:
    root_cmake_lines.append("find_program(FLATC_EXECUTABLE flatc REQUIRED)")

  if internal_fbs_packages:
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
  for pkg_find in sorted(packages_to_find):
    config_file_lines.append(f"find_dependency({pkg_find})")

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
  with open(root_cmake_path, "w") as out:
    out.write("\n".join(root_cmake_lines))


if __name__ == "__main__":
  main()
