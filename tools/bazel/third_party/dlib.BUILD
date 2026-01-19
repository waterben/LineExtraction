"""BUILD file for dlib - using http_archive in MODULE.bazel"""

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

# dlib as header-only with minimal compilation
cc_library(
    name = "dlib",
    srcs = ["dlib/all/source.cpp"],
    hdrs = glob(
        [
            "dlib/**/*.h",
        ],
        allow_empty = True,
    ),
    copts = [
        "-DDLIB_NO_GUI_SUPPORT",  # Optional: disable GUI if not needed
    ],
    includes = ["."],
    linkopts = [
        "-lpthread",
        "-lX11",
    ],
)
