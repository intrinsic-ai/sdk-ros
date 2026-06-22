load("//google3/bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = [
    "//incode/icon/no_absl:__subpackages__",
])

cc_library(
    name = "hardware_module_state_fbs_utils",
    srcs = ["hardware_module_state_utils.cc"],
    hdrs = [
        "hardware_module_state_utils.h",
    ],
    deps = [
        "//flatbuffer_definitions/icon/hal/interfaces:hardware_module_state_fbs_cc",
        "//icon/utils:attributes",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "icon_state_fbs_utils",
    srcs = ["icon_state_utils.cc"],
    hdrs = [
        "icon_state_utils.h",
    ],
    deps = [
        "//flatbuffer_definitions/icon/hal/interfaces:icon_state_fbs_cc",
        "//icon/utils:attributes",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "joint_limits_fbs_utils",
    srcs = ["joint_limits_utils.cc"],
    hdrs = [
        "joint_limits_utils.h",
    ],
    deps = [
        "//flatbuffer_definitions/icon/hal/interfaces:joint_limits_fbs_cc",
        "//icon/hal:hardware_interface_handle",
        "//icon/utils:attributes",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_macros",
        "//kinematics/types:joint_limits",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "joint_command_fbs_utils",
    srcs = ["joint_command_utils.cc"],
    hdrs = [
        "joint_command_utils.h",
    ],
    deps = [
        "//flatbuffer_definitions/icon/hal/interfaces:joint_command_fbs_cc",
        "//icon/flatbuffers:flatbuffer_utils",
        "//icon/utils:attributes",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_macros",
        "@com_github_google_flatbuffers//:runtime_cc",
    ],
)

# intrinsic:*:strip_begin
cc_test(
    name = "joint_limits_utils_test",
    srcs = ["joint_limits_utils_test.cc"],
    deps = [
        ":joint_limits_fbs_utils",
        "//flatbuffer_definitions/icon/hal/interfaces:joint_limits_fbs_cc",
        "//icon/hal:hardware_interface_handle",
        "//icon/hal:hardware_interface_registry",
        "//icon/hal:hardware_interface_traits",
        "//icon/interprocess/shared_memory_manager",
        "//icon/interprocess/shared_memory_manager/testing:unique_segment_name",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//kinematics/types:joint_limits",
        "@com_github_google_flatbuffers//:runtime_cc",
        "@com_google_googletest//:gtest",
    ],
)
# intrinsic:*:strip_end

cc_library(
    name = "joint_state_fbs_utils",
    srcs = ["joint_state_utils.cc"],
    hdrs = [
        "joint_state_utils.h",
    ],
    deps = [
        "//flatbuffer_definitions/icon/hal/interfaces:joint_state_fbs_cc",
        "//icon/utils:attributes",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)
