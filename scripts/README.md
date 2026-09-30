# Migration Guide: Transitioning to `intrinsic_sdk_build`

The legacy bash scripts in this directory are **deprecated** and will be removed in a future release:
- `build_container.sh` (Use `intrinsic_sdk_build container`)
- `build_skill_container.sh` (Use `intrinsic_sdk_build container`)
- `build_bundle.sh` (Use `intrinsic_sdk_build bundle`)

Please transition your workflows to using the command-line utility `intrinsic_sdk_build` provided by the `intrinsic_sdk_bundle_library_py` Python library.

---

## 1. Installing the New Tool

To use the new command-line tool, you must first install the `intrinsic_sdk_bundle_library_py` package.

### Option A: Using Colcon (Recommended for ROS developers)
From the root of your colcon workspace, run:
```bash
colcon build --packages-select intrinsic_sdk_bundle_library_py
source install/setup.bash
```

### Option B: Using pip
Alternatively, from the root of the repository, run:
```bash
pip install ./src/sdk-ros/intrinsic_sdk_bundle_library_py
```
*(Or `pip install -e ./src/sdk-ros/intrinsic_sdk_bundle_library_py` to install in editable mode).*

---

## 2. Command Mapping

### Building Containers
Instead of:
```bash
./src/sdk-ros/scripts/build_container.sh \
  --ros_distro "jazzy" \
  --service_name my_service \
  --service_package my_service_pkg
```
Use:
```bash
intrinsic_sdk_build container \
  --ros_distro "jazzy" \
  --service_name my_service \
  --service_package my_service_pkg
```

### Building Bundles
Instead of:
```bash
./src/sdk-ros/scripts/build_bundle.sh \
  --service_name my_service \
  --service_package my_service_pkg \
  --manifest_path my_service.manifest.textproto
```
Use:
```bash
intrinsic_sdk_build bundle \
  --service_name my_service \
  --service_package my_service_pkg \
  --manifest_path my_service.manifest.textproto
```

---

## 3. New Features

### Parameterizing ROS Distribution
Both container and bundle build workflows now fully support custom ROS distributions via the `--ros_distro` argument (which defaults to `jazzy`).

```bash
intrinsic_sdk_build container --ros_distro rolling ...
```
