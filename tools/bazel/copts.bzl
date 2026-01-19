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

# Strict warning flags - may trigger on external headers
LE_COPTS_STRICT = [
    "-Wold-style-cast",  # OpenCV headers use C-style casts
    "-Wconversion",
    "-Wsign-conversion",
    "-Weffc++",
    "-Wzero-as-null-pointer-constant",
]

# Combined: all warnings + error enforcement
LE_COPTS = LE_COPTS_SAFE + LE_COPTS_STRICT + ["-Werror"]

# For use when strict flags cause issues with external headers
LE_COPTS_COMPAT = LE_COPTS_SAFE + ["-Werror"]

# Compiler-specific variants (use with select())
LE_COPTS_GCC = LE_COPTS_COMPAT + LE_COPTS_GCC_ONLY
LE_COPTS_CLANG = LE_COPTS_COMPAT + LE_COPTS_CLANG_ONLY

# Reduced warning flags for third-party code (qplot, qplot3d)
LE_THIRD_PARTY_COPTS = [
    "-w",  # Disable all warnings for third-party code
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

    Uses LE_COPTS_COMPAT by default with compiler-specific adjustments to avoid
    issues with external headers (esp. OpenCV) that don't use -isystem includes.
    """
    cc_library(
        copts = LE_COPTS_COMPAT + copts + select({
            "//bazel:compiler_clang_env": LE_COPTS_CLANG_ONLY,
            "//conditions:default": LE_COPTS_GCC_ONLY,
        }),
        **kwargs
    )

def le_cc_test(copts = [], **kwargs):
    """cc_test wrapper with LineExtraction warning flags.

    Uses LE_COPTS_COMPAT by default with compiler-specific adjustments to avoid
    issues with external headers (esp. OpenCV) that don't use -isystem includes.
    """
    cc_test(
        copts = LE_COPTS_COMPAT + copts + select({
            "//bazel:compiler_clang_env": LE_COPTS_CLANG_ONLY,
            "//conditions:default": LE_COPTS_GCC_ONLY,
        }),
        **kwargs
    )
