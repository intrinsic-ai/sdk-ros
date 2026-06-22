load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//google3/bazel:cc_macros.bzl", "cc_library")

package(default_visibility = [
    "//incode/icon/no_absl:__subpackages__",
])

cc_library(
    name = "remote_trigger_server",
    srcs = [
        "remote_trigger_server.cc",
    ],
    hdrs = [
        "remote_trigger_constants.h",
        "remote_trigger_server.h",
    ],
    deps = [
        "//icon/interprocess:binary_futex",
        "//icon/interprocess/shared_memory_manager",
        "//icon/interprocess/shared_memory_manager:memory_segment",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_macros",
        "//icon/utils:time",
        "//util/thread",
        "@tl-expected//:tl-expected",
    ],
)

cc_library(
    name = "remote_trigger_test_common",
    testonly = True,
    hdrs = ["remote_trigger_test_common.h"],
    deps = [
        ":remote_trigger_server",
    ],
)

cc_test(
    name = "remote_trigger_server_test",
    srcs = ["remote_trigger_server_test.cc"],
    deps = [
        ":remote_trigger_server",
        ":remote_trigger_test_common",
        "//icon/interprocess/shared_memory_manager",
        "//icon/interprocess/shared_memory_manager/testing:unique_segment_name",
        "//icon/utils:status_and_expected_test_macros",
        "//util/thread",
        "@com_google_googletest//:gtest",
        "@tl-expected//:tl-expected",
    ],
)
