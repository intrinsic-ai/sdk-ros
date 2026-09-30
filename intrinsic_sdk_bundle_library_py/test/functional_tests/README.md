# Functional Tests for Intrinsic SDK Bundle Library (Python)

This directory contains test skills and services used to verify that the build and bundle workflow in `intrinsic_sdk_bundle_library_py` works correctly.

These tests are located outside the library package to match the real-world use case where users create their own skills in their own workspaces.

## Contents
- `test_python_skill`: A Python skill that reads parameters and publishes to a ROS topic.
- `test_cpp_skill`: A C++ skill that acts as a minimal smoke test.
- `test_python_service`: A Python service server implementing `SetBool`.
- `test_cpp_service`: A C++ service node.

For instructions on how to run these tests, see the [main README](../intrinsic_sdk_bundle_library_py/test/README.md).
