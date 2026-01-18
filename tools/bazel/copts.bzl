"""Compiler options for LineExtraction libraries."""

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

# Core warning flags - compatible with external headers
# NOTE: -Wcast-qual removed because OpenCV headers trigger it extensively
LE_COPTS_SAFE = [
    "-Wall",
    "-Wextra",
    "-Wpedantic",
    "-Wmissing-field-initializers",
    "-Wcast-align",
    # "-Wcast-qual",  # Disabled: OpenCV headers have const-cast issues
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

    Uses LE_COPTS_COMPAT by default to avoid issues with external headers that
    don't use -isystem includes. Override with LE_COPTS directly if needed.
    """
    cc_library(
        copts = LE_COPTS_COMPAT + copts,
        **kwargs
    )

def le_cc_test(copts = [], **kwargs):
    """cc_test wrapper with LineExtraction warning flags.

    Uses LE_COPTS_COMPAT by default to avoid issues with external headers that
    don't use -isystem includes. Override with LE_COPTS directly if needed.
    """
    cc_test(
        copts = LE_COPTS_COMPAT + copts,
        **kwargs
    )
