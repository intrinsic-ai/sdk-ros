load("//google3/bazel:cc_macros.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "thread",
    srcs = ["thread.cc"],
    hdrs = [
        "thread.h",
    ],
    deps = [
        ":stop_token",
        "//icon/utils:realtime_guard",
        "//icon/utils:status",
    ],
)

cc_library(
    name = "stop_token",
    hdrs = ["stop_token.h"],
    deps = [
        "//icon/utils:attributes",
    ],
)

cc_library(
    name = "lockstep",
    srcs = ["lockstep.cc"],
    hdrs = ["lockstep.h"],
    deps = [
        "//icon/interprocess:binary_futex",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:time",
    ],
)
