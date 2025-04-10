# Extract the bazel-bin directory for use later.
execute_process(
  COMMAND
    "${bazelisk_vendor_EXECUTABLE}"
    --nohome_rc
    --quiet
    info
      --experimental_convenience_symlinks=ignore
      bazel-bin
  WORKING_DIRECTORY
    "${intrinsic_sdk_SOURCE_DIR}/intrinsic"
  OUTPUT_VARIABLE
    intrinsic_sdk_BAZEL_BIN_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# TODO(wjwwood): this warning is not appropriate because it happens the first time
#   you use bazel on a system, like in a docker build, so we need a better sanity check here.
# if(NOT EXISTS ${intrinsic_sdk_BAZEL_BIN_DIR})
#   message(WARNING
#     "The detected bazel-bin directory does not exist: ${intrinsic_sdk_BAZEL_BIN_DIR}")
# endif()
