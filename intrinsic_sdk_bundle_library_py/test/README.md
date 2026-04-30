# Tests for Intrinsic SDK Bundle Library (Python)

This directory contains tests for the `intrinsic_sdk_bundle_library_py` package.

## 1. Unit Tests and Linting
These tests verify the logic of the package and check code style.

### Running all tests
To run all tests (including unit tests and linting), use `colcon test` from the workspace root:
```bash
colcon test --packages-select intrinsic_sdk_bundle_library_py
```
Or run `pytest` directly in this directory (if dependencies are installed):
```bash
pytest test/
```

### Running specific tests
To run only the unit tests for `build.py`:
```bash
pytest test/test_build.py
```

## 2. Functional Tests
To verify that the build and bundle workflow works correctly for both C++ and Python, we use test skills and services located in the [functional_tests_bundle_library_py](../../functional_tests_bundle_library_py) directory.

### Python Skill Test
To test building and bundling a Python skill:
```bash
# 1. Build container (OCI image)
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py container \
  --skill_name test_python_skill \
  --skill_package test_python_skill \
  --skill_type python

# 2. Build bundle
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py bundle \
  --skill_name test_python_skill \
  --skill_package test_python_skill \
  --manifest_path src/sdk-ros/functional_tests_bundle_library_py/test_python_skill/test_python_skill.manifest.textproto
```

### C++ Skill Test
To test building and bundling a C++ skill:
```bash
# 1. Build container
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py container \
  --skill_name test_cpp_skill \
  --skill_package test_cpp_skill \
  --skill_type cpp

# 2. Build bundle
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py bundle \
  --skill_name test_cpp_skill \
  --skill_package test_cpp_skill \
  --manifest_path src/sdk-ros/functional_tests_bundle_library_py/test_cpp_skill/test_cpp_skill.manifest.textproto
```

### Python Service Test
To test building and bundling a Python service:
```bash
# 1. Build container
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py container \
  --service_name test_python_service \
  --service_package test_python_service

# 2. Build bundle
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py bundle \
  --service_name test_python_service \
  --service_package test_python_service \
  --manifest_path src/sdk-ros/functional_tests_bundle_library_py/test_python_service/test_python_service.manifest.textproto \
  --default_config src/sdk-ros/functional_tests_bundle_library_py/test_python_service/test_python_service_default_config.pbtxt
```


### C++ Service Test
To test building and bundling a C++ service:
```bash
# 1. Build container
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py container \
  --service_name test_cpp_service \
  --service_package test_cpp_service

# 2. Build bundle
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py bundle \
  --service_name test_cpp_service \
  --service_package test_cpp_service \
  --manifest_path src/sdk-ros/functional_tests_bundle_library_py/test_cpp_service/test_cpp_service.manifest.textproto \
  --default_config src/sdk-ros/functional_tests_bundle_library_py/test_cpp_service/test_cpp_service_default_config.pbtxt
```

## 3. Installation and Verification

Once you have built the bundle, you can install and verify it on the platform.

### Python Skill
To install the Python skill bundle:
```bash
inctl skill install images/test_python_skill/test_python_skill.bundle.tar \
  --org=<YOUR_ORGANIZATION> \
  --solution=<YOUR_SOLUTION_ID>
```

To verify that it works correctly, check the skill logs for:
- `Handling Predict call` and successful return of `PredictResult`.
- `Executing TestPythonSkill` when the skill is executed.
- `Received input_data: <value>` to verify parameter passing.
- `Published to ROS topic: Skill received: <value>` to verify ROS publishing.

### C++ Skill
To install the C++ skill bundle:
```bash
inctl skill install images/test_cpp_skill/test_cpp_skill.bundle.tar \
  --org=<YOUR_ORGANIZATION> \
  --solution=<YOUR_SOLUTION_ID>
```

To verify that it works correctly, check the skill logs for:
- `TestCppSkill::Execute` when the skill is executed.

### Python Service
To install the Python service bundle:
```bash
inctl asset install images/test_python_service/test_python_service.bundle.tar \
  --org=<YOUR_ORGANIZATION> \
  --solution=<YOUR_SOLUTION_ID>
```

To verify that it works correctly:
1. Shell into the pod via `k9s` (or `kubectl exec`).
2. Source the workspace: `source /opt/ros/overlay/install/setup.bash`.
3. Call the service: `ros2 service call /set_bool std_srvs/srv/SetBool "{data: true}"`.
4. Check the logs for: `Received request in Python: data=True`.

### C++ Service
To install the C++ service bundle:
```bash
inctl asset install images/test_cpp_service/test_cpp_service.bundle.tar \
  --org=<YOUR_ORGANIZATION> \
  --solution=<YOUR_SOLUTION_ID>
```

To verify that it works correctly, check the logs for:
- `Test service node started!`

