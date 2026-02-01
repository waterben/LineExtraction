"""Compiler options for LineExtraction libraries."""

# Core warning flags - standard warnings that work well with any codebase
LE_COPTS_BASE = [
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

# Strict warning flags - aggressive checks for code quality
# These are safe because external headers (OpenCV, Eigen) use -isystem
LE_COPTS_STRICT = [
    "-Wconversion",
    "-Wsign-conversion",
    "-Weffc++",
    "-Wzero-as-null-pointer-constant",
    "-Wold-style-cast",
]

# Main compiler options - USE THIS for all library code
# Combines base + strict warnings with -Werror
LE_COPTS = LE_COPTS_BASE + LE_COPTS_STRICT + ["-Werror"]

# Compatibility alias for examples (deprecated, use LE_COPTS)
LE_COPTS_COMPAT = LE_COPTS

# Warning flags for third-party C++ code (qplot, qplot3d)
LE_THIRD_PARTY_COPTS = [
    "-Wall",
    "-Wextra",
    "-Werror",
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
    "-Wno-implicit-fallthrough",
]

# Warning suppressions for arpack++ headers (legacy C++98 code)
ARPACKPP_COPTS = [
    "-Wno-shadow",
    "-Wno-overloaded-virtual",
    "-Wno-implicit-fallthrough",
]
