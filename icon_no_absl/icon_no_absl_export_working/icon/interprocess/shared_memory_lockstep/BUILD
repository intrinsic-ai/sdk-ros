load("//google3/bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shared_memory_lockstep",
    srcs = [
        "shared_memory_lockstep.cc",
    ],
    hdrs = [
        "shared_memory_lockstep.h",
    ],
    deps = [
        "//icon/interprocess/shared_memory_manager",
        "//icon/interprocess/shared_memory_manager:domain_socket_utils",
        "//icon/interprocess/shared_memory_manager:memory_segment",
        "//icon/utils:attributes",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_macros",
        "//util/thread:lockstep",
    ],
)

# intrinsic:*:strip_begin
cc_test(
    name = "shared_memory_lockstep_test",
    srcs = ["shared_memory_lockstep_test.cc"],
    deps = [
        ":shared_memory_lockstep",
        "//icon/interprocess/shared_memory_manager",
        "//icon/interprocess/shared_memory_manager:memory_segment",
        "//icon/interprocess/shared_memory_manager/testing:unique_segment_name",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//icon/utils:time",
        "//util/thread",
        "//util/thread:lockstep",
        "@com_google_googletest//:gtest",
    ],
)
# intrinsic:*:strip_end
