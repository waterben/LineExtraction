"""Build rules for the Rerun C++ SDK v0.29.2

The rerun_cpp_sdk.zip contains:
  - src/         C++ source and header files (the SDK wrapper layer)
  - lib/         Pre-compiled librerun_c platform-specific static libraries
  - CMakeLists.txt + download_and_build_arrow.cmake
                 CMake build that downloads and builds Apache Arrow

Strategy:
  1. cc_import provides the pre-compiled Rust FFI layer (librerun_c)
  2. cmake() from rules_foreign_cc builds librerun_sdk.a and libArrow.a
     (download_and_build_arrow.cmake is patched to use Unix Makefiles instead
     of Ninja presets, so no ninja-build package is required)
  3. The resulting cmake() target exposes headers from include/ and the .a files
"""

load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

package(default_visibility = ["//visibility:public"])

# All SDK sources - passed to the cmake() rule as lib_source
filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
)

# Pre-compiled Rust FFI layer (platform-specific)
# The CMake build links against this automatically via RERUN_C_LIB auto-detection.
# We expose it here so dependent cc_libraries can link against it directly.
cc_import(
    name = "rerun_core",
    static_library = "lib/librerun_c__linux_x64.a",
)

# Build the rerun C++ SDK and Apache Arrow via CMake.
#
# CMake downloads and builds Arrow 18.0.0 as an ExternalProject during the build.
# This requires internet access (not fully sandboxed). Arrow is cached after the
# first build, so subsequent builds are fast.
#
# generate_args uses Unix Makefiles (-GUnix Makefiles) instead of Ninja so that
# ninja-build does not need to be installed on the host.
cmake(
    name = "rerun_sdk",
    cache_entries = {
        "CMAKE_BUILD_TYPE": "RelWithDebInfo",
        # Link Arrow statically to avoid runtime .so dependencies
        "RERUN_ARROW_LINK_SHARED": "OFF",
    },
    # Disable ccache: the Bazel sandbox uses a read-only filesystem for the
    # ccache cache directory, which causes the Arrow ExternalProject build to fail.
    env = {
        "CCACHE_DISABLE": "1",
    },
    generate_args = ["-GUnix Makefiles"],
    lib_source = ":all_srcs",
    # Required system libraries on Linux
    linkopts = [
        "-lm",
        "-ldl",
        "-lpthread",
    ],
    out_static_libs = [
        "librerun_sdk.a",
        "libarrow.a",
        "libarrow_bundled_dependencies.a",
    ],
    # Arrow downloads happen during cmake ExternalProject_Add build:
    # requires network access at build time (first build only)
    tags = ["requires-network"],
    deps = [":rerun_core"],
)
