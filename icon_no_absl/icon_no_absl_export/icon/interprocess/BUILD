load("//google3/bazel:cc_macros.bzl", "cc_library")

package(default_visibility = [
    "//icon:__subpackages__",
    "//util/thread:__subpackages__",  # intrinsic:*:strip
])

cc_library(
    name = "binary_futex",
    srcs = ["binary_futex.cc"],
    hdrs = [
        "binary_futex.h",
    ],
    deps = [
        "//icon/utils:status",
        "//icon/utils:strerror",
        "//icon/utils:time",
    ],
)
