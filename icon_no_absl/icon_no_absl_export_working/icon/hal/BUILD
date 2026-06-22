load("//google3/bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = [
    "//incode/icon/no_absl:__subpackages__",
])

cc_library(
    name = "hardware_interface_traits",
    hdrs = ["hardware_interface_traits.h"],
)

cc_library(
    name = "hardware_interface_handle",
    hdrs = ["hardware_interface_handle.h"],
    deps = [
        ":icon_state_register",
        "//flatbuffer_definitions/icon/hal/interfaces:icon_state_fbs_cc",
        "//icon/interprocess/shared_memory_manager:memory_segment",
        "//icon/utils:current_cycle",
        "//icon/utils:status",
        "//icon/utils:time",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "hardware_interface_registry",
    srcs = ["hardware_interface_registry.cc"],
    hdrs = [
        "get_hardware_interface.h",
        "hardware_interface_registry.h",
    ],
    deps = [
        ":hardware_interface_handle",
        ":hardware_interface_traits",
        ":icon_state_register",
        "//flatbuffer_definitions/icon/hal/interfaces:icon_state_fbs_cc",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/interprocess/shared_memory_manager",
        "//icon/interprocess/shared_memory_manager:domain_socket_utils",
        "//icon/interprocess/shared_memory_manager:memory_segment",
        "//icon/interprocess/shared_memory_manager:segment_header",
        "//icon/interprocess/shared_memory_manager:segment_info_utils",
        "//icon/utils:attributes",
        "//icon/utils:log",
        "//icon/utils:realtime_guard",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_macros",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "hardware_module_interface",
    hdrs = [
        "hardware_module_interface.h",
    ],
    deps = [
        "//icon/control:realtime_clock_interface",
        "//icon/utils:status",
    ],
)

cc_library(
    name = "icon_state_register",
    hdrs = ["icon_state_register.h"],
    deps = [
        ":hardware_interface_traits",
        "//flatbuffer_definitions/icon/hal/interfaces:icon_state_fbs_cc",
        "//icon/hal/interfaces:icon_state_fbs_utils",
    ],
)

cc_library(
    name = "realtime_clock",
    srcs = ["realtime_clock.cc"],
    hdrs = ["realtime_clock.h"],
    deps = [
        "//icon/control:realtime_clock_interface",
        "//icon/interprocess/shared_memory_lockstep",
        "//icon/interprocess/shared_memory_manager",
        "//icon/interprocess/shared_memory_manager:memory_segment",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:time",
        "//util/thread:lockstep",
    ],
)

# intrinsic:*:strip_begin

cc_test(
    name = "hardware_interface_registry_test",
    srcs = ["hardware_interface_registry_test.cc"],
    deps = [
        ":hardware_interface_handle",
        ":hardware_interface_registry",
        ":hardware_interface_traits",
        ":icon_state_register",
        "//flatbuffer_definitions/icon/hal/interfaces:icon_state_fbs_cc",
        "//flatbuffer_definitions/icon/hal/interfaces:joint_limits_fbs_cc",
        "//icon/hal/interfaces:joint_limits_fbs_utils",
        "//icon/interprocess/shared_memory_manager",
        "//icon/interprocess/shared_memory_manager:segment_header",
        "//icon/interprocess/shared_memory_manager/testing:unique_segment_name",
        "//icon/utils:current_cycle",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//icon/utils:time",
        "@com_github_google_flatbuffers//:runtime_cc",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "get_hardware_interface_test",
    srcs = ["get_hardware_interface_test.cc"],
    deps = [
        ":hardware_interface_registry",
        ":icon_state_register",
        "//flatbuffer_definitions/icon/hal/interfaces:icon_state_fbs_cc",
        "//flatbuffer_definitions/icon/hal/interfaces:joint_limits_fbs_cc",
        "//flatbuffer_definitions/icon/interprocess/shared_memory_manager:segment_info_fbs_cc",
        "//icon/flatbuffers:flatbuffer_utils",
        "//icon/hal/interfaces:icon_state_fbs_utils",
        "//icon/hal/interfaces:joint_limits_fbs_utils",
        "//icon/interprocess/shared_memory_manager:segment_header",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "@com_github_google_flatbuffers//:runtime_cc",
        "@com_google_googletest//:gtest",
    ],
)
# intrinsic:*:strip_end
