"""Compiler options for LineExtraction libraries."""

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

# Core warning flags - compatible with external headers
# NOTE: -Wcast-qual removed because OpenCV headers trigger it extensively
LE_COPTS_SAFE = [
    "-Wall",
    "-Wextra",
    "-Wpedantic",
    "-Wmissing-field-initializers",
    "-Woverloaded-virtual",
    "-Wdangling-else",
    "-Wformat-security",
    "-Wshadow",
    "-Wsign-promo",
    "-Wundef",
    "-Wno-write-strings",
    "-Wreorder",
    "-Wdelete-non-virtual-dtor",
    "-Wno-comment",
    "-Wnoexcept-type",
    "-Wnon-virtual-dtor",
]

# GCC-only warning flags (cast-align triggers false positives in Clang with OpenCV)
LE_COPTS_GCC_ONLY = [
    "-Wcast-align",
]

# Clang-specific suppressions for OpenCV header compatibility
LE_COPTS_CLANG_ONLY = [
    "-Wno-cast-align",  # OpenCV Mat::at/ptr triggers this in template instantiation
    "-Wno-c11-extensions",  # OpenCV uses _Atomic in some headers
]

# Strict warning flags - these match CMake configuration
LE_COPTS_STRICT = [
    "-Wconversion",
    "-Wsign-conversion",
    "-Weffc++",
    "-Wzero-as-null-pointer-constant",
    "-Wold-style-cast",
    "-Wno-error=old-style-cast",  # OpenCV headers use C-style casts extensively
]

# Combined: all warnings + error enforcement (matching CMake)
LE_COPTS = LE_COPTS_SAFE + LE_COPTS_STRICT + ["-Werror"]

# Legacy: reduced warnings for compatibility (not recommended)
LE_COPTS_COMPAT = LE_COPTS_SAFE + ["-Werror"]

# Compiler-specific variants (use with select())
LE_COPTS_GCC = LE_COPTS_COMPAT + LE_COPTS_GCC_ONLY
LE_COPTS_CLANG = LE_COPTS_COMPAT + LE_COPTS_CLANG_ONLY

# Warning flags for third-party C++ code (qplot, qplot3d)
# Based on CMake configuration - keep most warnings but disable problematic ones
LE_THIRD_PARTY_COPTS = [
    "-Wall",
    "-Wextra",
    "-Werror",
    # Disable warnings that third-party code triggers
    "-Wno-effc++",
    "-Wno-conversion",
    "-Wno-sign-conversion",
    "-Wno-float-conversion",
    "-Wno-unused-parameter",
    "-Wno-deprecated-declarations",
    "-Wno-old-style-cast",
    "-Wno-zero-as-null-pointer-constant",
]

# Warning flags for third-party C code (gl2ps.c in qplot3d)
LE_THIRD_PARTY_C_COPTS = [
    "-Wall",
    "-Wextra",
    "-Werror",
    "-Wno-conversion",
    "-Wno-sign-conversion",
    "-Wno-unused-parameter",
    "-Wno-implicit-fallthrough",  # gl2ps.c has intentional fallthroughs
]

# Warning suppressions needed for arpack++ headers (legacy C++98 code)
# arpack++ has: shadow warnings, overloaded-virtual, implicit-fallthrough
ARPACKPP_COPTS = [
    "-Wno-shadow",
    "-Wno-overloaded-virtual",
    "-Wno-implicit-fallthrough",
]

def le_cc_library(copts = [], **kwargs):
    """cc_library wrapper with LineExtraction warning flags.

    Uses full warning configuration matching CMake.
    Note: -Wold-style-cast is enabled but not an error due to OpenCV headers.
    """
    cc_library(
        copts = LE_COPTS + copts + select({
            "//bazel:compiler_clang_env": LE_COPTS_CLANG_ONLY,
            "//conditions:default": LE_COPTS_GCC_ONLY,
        }),
        **kwargs
    )

def le_cc_test(copts = [], **kwargs):
    """cc_test wrapper with LineExtraction warning flags.

    Uses full warning configuration matching CMake.
    Note: -Wold-style-cast is enabled but not an error due to OpenCV headers.
    """
    cc_test(
        copts = LE_COPTS + copts + select({
            "//bazel:compiler_clang_env": LE_COPTS_CLANG_ONLY,
            "//conditions:default": LE_COPTS_GCC_ONLY,
        }),
        **kwargs
    )
