load("//google3/bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = [
    "//visibility:public",
])

cc_library(
    name = "joint_limits",
    srcs = ["joint_limits.cc"],
    hdrs = ["joint_limits.h"],
    deps = [
        "//eigenmath",
        "//icon/utils:log",
        "//icon/utils:status",
        "@tl-expected//:tl-expected",
    ],
)

cc_test(
    name = "joint_limits_test",
    srcs = ["joint_limits_test.cc"],
    deps = [
        ":joint_limits",
        "//eigenmath",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "@com_google_googletest//:gtest",
    ],
)
