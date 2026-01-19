"""Eigen - C++ template library for linear algebra"""

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "eigen",
    hdrs = glob(
        [
            "Eigen/**",
            "unsupported/Eigen/**",
        ],
        allow_empty = True,
    ),
    includes = ["."],
)
