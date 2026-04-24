# Tests for Intrinsic SDK Bundle Library (Python)

This directory contains tests for the `intrinsic_sdk_bundle_library_py` package.

## 1. Standard Linting Tests
These tests check code style, copyright headers, and docstrings.

To run these tests, use `colcon test` from the workspace root:
```bash
colcon test --packages-select intrinsic_sdk_bundle_library_py
```
Or run `pytest` directly in this directory (if dependencies are installed):
```bash
pytest test/
```

## 2. Functional Tests
To verify that the build and bundle workflow works correctly for both C++ and Python skills, we use test skills located in the `intrinsic_sdk_cmake` package.

### Python Skill Test
To test building and bundling a Python skill:
```bash
# 1. Build container (OCI image)
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py container \
  --skill_name test_python_skill \
  --skill_package intrinsic_sdk_cmake \
  --skill_type python

# 2. Build bundle
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py bundle \
  --skill_name test_python_skill \
  --skill_package intrinsic_sdk_cmake \
  --manifest_path src/sdk-ros/intrinsic_sdk_cmake/test/test_python_skill/test_python_skill.manifest.textproto
```

### C++ Skill Test
To test building and bundling a C++ skill:
```bash
# 1. Build container
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py container \
  --skill_name test_cpp_skill \
  --skill_package intrinsic_sdk_cmake \
  --skill_type cpp

# 2. Build bundle
python3 src/sdk-ros/intrinsic_sdk_bundle_library_py/intrinsic_sdk_bundle_library_py/build.py bundle \
  --skill_name test_cpp_skill \
  --skill_package intrinsic_sdk_cmake \
  --manifest_path src/sdk-ros/intrinsic_sdk_cmake/test/test_cpp_skill/test_cpp_skill.manifest.textproto
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

### C++ Skill
To install the C++ skill bundle:
```bash
inctl skill install images/test_cpp_skill/test_cpp_skill.bundle.tar \
  --org=<YOUR_ORGANIZATION> \
  --solution=<YOUR_SOLUTION_ID>
```

To verify that it works correctly, check the skill logs for:
- `TestCppSkill::Execute` when the skill is executed.
```
