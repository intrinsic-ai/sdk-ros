load("//google3/bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = [
    "//incode/icon/no_absl:__subpackages__",
])

cc_library(
    name = "attributes",
    hdrs = ["attributes.h"],
)

cc_library(
    name = "cleanup",
    hdrs = ["cleanup.h"],
    deps = [":attributes"],
)

cc_test(
    name = "cleanup_test",
    size = "small",
    srcs = ["cleanup_test.cc"],
    deps = [
        ":cleanup",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "current_cycle",
    srcs = ["current_cycle.cc"],
    hdrs = ["current_cycle.h"],
)

cc_test(
    name = "current_cycle_test",
    size = "small",
    srcs = ["current_cycle_test.cc"],
    deps = [
        ":current_cycle",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "format",
    hdrs = ["format.h"],
)

cc_test(
    name = "format_test",
    size = "small",
    srcs = ["format_test.cc"],
    deps = [
        ":format",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "log",
    hdrs = ["log.h"],
)

cc_test(
    name = "log_test",
    size = "small",
    srcs = ["log_test.cc"],
    deps = [
        ":log",
        ":mock_log_sink",
        ":time",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "mock_log_sink",
    testonly = True,
    hdrs = ["mock_log_sink.h"],
    deps = [
        ":attributes",
        ":log",
    ],
)

cc_library(
    name = "realtime_guard",
    hdrs = ["realtime_guard.h"],
)

cc_library(
    name = "status",
    hdrs = ["status.h"],
    deps = [":attributes"],
)

cc_library(
    name = "status_and_expected_macros",
    hdrs = ["status_and_expected_macros.h"],
    deps = [
        ":status",
        "@tl-expected//:tl-expected",
    ],
)

cc_test(
    name = "status_and_expected_macros_test",
    size = "small",
    srcs = ["status_and_expected_macros_test.cc"],
    deps = [
        ":status_and_expected_macros",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "status_and_expected_test_macros",
    testonly = True,
    hdrs = ["status_and_expected_test_macros.h"],
    deps = [
        ":status",
        ":status_and_expected_macros",
        "@tl-expected//:tl-expected",
    ],
)

cc_library(
    name = "strerror",
    hdrs = ["strerror.h"],
)

cc_test(
    name = "strerror_test",
    size = "small",
    srcs = ["strerror_test.cc"],
    deps = [
        ":strerror",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "time",
    srcs = ["time.cc"],
    hdrs = ["time.h"],
)

cc_test(
    name = "time_test",
    size = "small",
    srcs = ["time_test.cc"],
    deps = [
        ":time",
        "@com_google_googletest//:gtest",
    ],
)
