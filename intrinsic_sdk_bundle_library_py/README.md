# Intrinsic SDK Bundle Library (Python)

This package provides Python utilities, a CLI tool, and a `colcon` verb extension to simplify building container images and Intrinsic SDK bundles for custom ROS skills and services.

---

## Installation & Setup

1. Build the library package in your ROS 2 workspace:
   ```bash
   colcon build --packages-select intrinsic_sdk_bundle_library_py
   ```
2. Source your workspace to register the entry points and register the `colcon` extension:
   ```bash
   source install/setup.bash
   ```

---

## Usage Case 1: Colcon Extension (Recommended)

The `colcon intrinsic_bundle` command automatically scans your workspace, discovers packages containing manifests (`*.manifest.textproto`), determines package types, and handles both building and bundling in a single command.

### Run Bundling
To build and bundle a specific package:
```bash
colcon intrinsic_bundle --packages-select <your_package_name>
```

### Options
* `--bundle-dir <path>`: Directory to save generated container images and bundles (default: `./intrinsic_asset_bundles`).
* `--ros-distro <distro>`: The target ROS distro (default: `jazzy`).
* `--no-cache`: Do not use cache when building container images.
* `--builder-name <name>`: Custom Docker buildx builder name.

---

## Usage Case 2: Command Line CLI Tool (`intrinsic_sdk_build`)

For finer-grained control or non-colcon environments, you can invoke the packaging steps manually using the direct CLI script.

### 1. Build the Container Image
Creates a compressed container image from the package source:
```bash
intrinsic_sdk_build container \
  --skill_name <skill_name> \
  --skill_package <package_name> \
  --skill_type <cpp|python>
```
*Outputs: `./intrinsic_asset_bundles/<skill_name>/<skill_name>.tar`*

### 2. Build the Bundle
Bundles the container image along with its manifest using the `inbuild` compiler:
```bash
intrinsic_sdk_build bundle \
  --skill_name <skill_name> \
  --skill_package <package_name> \
  --manifest_path <path_to_manifest>
```
*Outputs: `./intrinsic_asset_bundles/<skill_name>/<skill_name>.bundle.tar`*

---

## Architecture & Configuration

### `inbuild` Resolution
The package requires the `inbuild` binary to build bundles. It resolves `inbuild` using the following order:
1. **Host PATH**: Checks the host system's `PATH` for `inbuild` (e.g., using `shutil.which`).
2. **GitHub Releases Fallback**: If not found on the host, it downloads the correct architecture release binary from the GitHub repository (`https://github.com/intrinsic-ai/sdk`).
3. **Tag Mapping**: Includes an internal version mapper to translate local/CI SDK development tags to valid patch releases on GitHub containing pre-compiled binaries (e.g., `v1.31.20260427` mapping to `v1.31.20260427.1`).
