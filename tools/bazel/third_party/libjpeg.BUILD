"""
BUILD file for system libjpeg-turbo library.
This avoids the SIMD linking issues in BCR libjpeg-turbo 3.1.2.
"""

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "jpeg",
    linkopts = [
        "-ljpeg",
    ],
)
