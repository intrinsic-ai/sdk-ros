load("//google3/bazel:cc_macros.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "unique_segment_name",
    testonly = True,
    srcs = ["unique_segment_name.cc"],
    hdrs = ["unique_segment_name.h"],
    deps = [
        "//icon/interprocess/shared_memory_manager:memory_segment",
    ],
)
