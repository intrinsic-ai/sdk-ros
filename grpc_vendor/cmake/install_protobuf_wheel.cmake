# Read the filename from the text file
file(READ "bazel-bin/python/dist/binary_wheel.name" WHEEL_NAME)
string(STRIP "${WHEEL_NAME}" WHEEL_NAME)

# Execute the pip command
execute_process(
    COMMAND python3 -m pip install "bazel-bin/python/dist/${WHEEL_NAME}" --prefix="${INSTALL_DIR}" --root=/ --no-deps --no-build-isolation
    RESULT_VARIABLE result
)

if(NOT result EQUAL 0)
    message(FATAL_ERROR "pip install failed")
endif()
