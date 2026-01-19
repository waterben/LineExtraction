"""BUILD file for managed OpenCV (built by CMake)"""

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opencv",
    srcs = glob(
        [
            "lib/libopencv_*.a",
            "3rdparty/**/*.a",
        ],
        allow_empty = True,
    ),
    hdrs = glob(
        [
            "opencv2/**/*.h",
            "opencv2/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["."],
    linkopts = [
        "-lpthread",
        "-ldl",
        "-lz",
    ],
)
