load("//google3/bazel:cc_macros.bzl", "cc_library")

package(
    default_visibility = [
        "//incode/icon/no_absl:__subpackages__",
    ],
)

# intrinsic:*:strip_begin
package_group(
    name = "icon_rtcl",
    packages = [
        "//flatbuffer_definitions/icon/control/...",
        "//flatbuffer_definitions/icon/proto/...",
        "//flatbuffer_definitions/icon/tools/configmaker/...",
    ],
)

package_group(
    name = "icon",
    packages = [
        "//flatbuffer_definitions/apps/...",
        "//flatbuffer_definitions/icon/...",
        "//flatbuffer_definitions/skills/...",
    ],
)

# intrinsic:*:strip_end

cc_library(
    name = "realtime_clock_interface",
    srcs = ["realtime_clock_interface.cc"],
    hdrs = ["realtime_clock_interface.h"],
    visibility = [
        "//incode/icon/no_absl:__subpackages__",
    ],
    deps = [
        "//icon/utils:status",
        "//icon/utils:time",
    ],
)
