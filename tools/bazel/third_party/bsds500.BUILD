"""BSDS500 - Berkeley Segmentation Dataset 500

This BUILD file exposes the BSDS500 images for use as Bazel data dependencies.
Source: https://github.com/BIDS/BSDS500
"""

package(default_visibility = ["//visibility:public"])

# All training images
filegroup(
    name = "train",
    srcs = glob(["BSDS500/data/images/train/*.jpg"]),
)

# All test images
filegroup(
    name = "test",
    srcs = glob(["BSDS500/data/images/test/*.jpg"]),
)

# All validation images
filegroup(
    name = "val",
    srcs = glob(["BSDS500/data/images/val/*.jpg"]),
)

# All images combined (train + test + val)
filegroup(
    name = "all",
    srcs = [
        ":test",
        ":train",
        ":val",
    ],
)

# Ground truth boundaries
filegroup(
    name = "groundTruth_train",
    srcs = glob(["BSDS500/data/groundTruth/train/*.mat"]),
)

filegroup(
    name = "groundTruth_test",
    srcs = glob(["BSDS500/data/groundTruth/test/*.mat"]),
)

filegroup(
    name = "groundTruth_val",
    srcs = glob(["BSDS500/data/groundTruth/val/*.mat"]),
)

filegroup(
    name = "groundTruth_all",
    srcs = [
        ":groundTruth_test",
        ":groundTruth_train",
        ":groundTruth_val",
    ],
)
