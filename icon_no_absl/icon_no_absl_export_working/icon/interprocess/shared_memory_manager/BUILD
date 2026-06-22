load("//google3/bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "segment_header",
    srcs = [
        "segment_header.cc",
    ],
    hdrs = [
        "segment_header.h",
    ],
    deps = [
        "//icon/utils:attributes",
        "//icon/utils:log",
        "//icon/utils:time",
    ],
)

# intrinsic:*:strip_begin
cc_test(
    name = "segment_header_test",
    srcs = ["segment_header_test.cc"],
    deps = [
        ":segment_header",
        "//icon/utils:mock_log_sink",
        "//icon/utils:time",
        "@com_google_googletest//:gtest",
    ],
)
# intrinsic:*:strip_end

cc_library(
    name = "segment_info_utils",
    srcs = ["segment_info_utils.cc"],
    hdrs = ["segment_info_utils.h"],
    deps = [
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/flatbuffers:fixed_string",
        "//icon/utils:status",
        "@tl-expected//:tl-expected",
    ],
)

# intrinsic:*:strip_begin
cc_test(
    name = "segment_info_utils_test",
    srcs = ["segment_info_utils_test.cc"],
    deps = [
        ":segment_info_utils",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/flatbuffers:flatbuffer_utils",
        "@com_google_googletest//:gtest",
    ],
)
# intrinsic:*:strip_end

cc_library(
    name = "shared_memory_manager",
    srcs = ["shared_memory_manager.cc"],
    hdrs = ["shared_memory_manager.h"],
    # With gcc we explicitly have to link against librt,
    # providing POSIX functions for shared memory, such as shm_open.
    # Clang automatically links against it,
    # but won't complain if specified explicitly here.
    linkopts = [
        "-lrt",
    ],
    deps = [
        ":domain_socket_utils",
        ":memory_segment",
        ":segment_header",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/flatbuffers:flatbuffer_utils",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_macros",
    ],
)

#intrinsic:*:strip_begin
cc_test(
    name = "shared_memory_manager_test",
    srcs = ["shared_memory_manager_test.cc"],
    deps = [
        ":domain_socket_server",
        ":domain_socket_utils",
        ":memory_segment",
        ":segment_header",
        ":segment_info_utils",
        ":shared_memory_manager",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/flatbuffers:flatbuffer_utils",
        "//icon/interprocess/shared_memory_manager/testing:unique_segment_name",
        "//icon/utils:mock_log_sink",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "@com_google_googletest//:gtest",
    ],
)
#intrinsic:*:strip_end

cc_library(
    name = "memory_segment",
    srcs = ["memory_segment.cc"],
    hdrs = ["memory_segment.h"],
    deps = [
        ":domain_socket_utils",
        ":segment_header",
        "//icon/utils:format",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:strerror",
        "//icon/utils:time",
    ],
)

cc_library(
    name = "domain_socket_server",
    srcs = ["domain_socket_server.cc"],
    hdrs = ["domain_socket_server.h"],
    deps = [
        ":domain_socket_utils",
        ":shared_memory_manager",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/flatbuffers:fixed_string",
        "//icon/hal:hardware_interface_registry",
        "//icon/utils:cleanup",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_macros",
        "//icon/utils:time",
        "//util/thread",
        "//util/thread:stop_token",
    ],
)

cc_library(
    name = "domain_socket_utils",
    srcs = ["domain_socket_utils.cc"],
    hdrs = ["domain_socket_utils.h"],
    deps = [
        ":segment_info_utils",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/flatbuffers:flatbuffer_utils",
        "//icon/utils:cleanup",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:strerror",
        "//icon/utils:time",
    ],
)

#intrinsic:*:strip_begin

cc_test(
    name = "memory_segment_test",
    srcs = ["memory_segment_test.cc"],
    deps = [
        ":memory_segment",
        ":segment_header",
        ":shared_memory_manager",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/interprocess/shared_memory_manager/testing:unique_segment_name",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//icon/utils:time",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "domain_socket_utils_test",
    srcs = ["domain_socket_utils_test.cc"],
    deps = [
        ":domain_socket_utils",
        "//icon/utils:mock_log_sink",
        "//icon/utils:status",
        "//icon/utils:time",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "domain_socket_server_test",
    srcs = ["domain_socket_server_test.cc"],
    deps = [
        ":domain_socket_server",
        ":domain_socket_utils",
        ":shared_memory_manager",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/flatbuffers:flatbuffer_utils",
        "//icon/hal:hardware_interface_registry",
        "//icon/interprocess/shared_memory_manager/testing:unique_segment_name",
        "//icon/utils:cleanup",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//icon/utils:time",
        "@com_google_googletest//:gtest",
    ],
)
#intrinsic:*:strip_end
