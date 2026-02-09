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

# Relaxed warnings for examples and legacy code
# Base warnings without strict conversion checks
LE_COPTS_COMPAT = LE_COPTS_BASE + ["-Werror"]

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

# Warning flags for benchmarks (Google Benchmark uses deprecated patterns)
LE_BENCHMARK_COPTS = LE_COPTS_BASE + [
    "-Werror",
    "-Wno-deprecated-declarations",  # benchmark::DoNotOptimize deprecation
]

# Warning flags for pybind11 binding code
# Relaxed warnings because pybind11 macros generate code that triggers
# -Weffc++, -Wold-style-cast, -Wconversion, etc.
LE_PYBIND_COPTS = LE_COPTS_BASE + [
    "-Werror",
    "-Wno-effc++",
    "-Wno-old-style-cast",
    "-Wno-conversion",
    "-Wno-sign-conversion",
    "-Wno-zero-as-null-pointer-constant",
    "-Wno-unused-parameter",
]

# Warning suppressions for arpack++ headers (legacy C++98 code)
# arpack++ is not a system include, so we need to suppress warnings manually
ARPACKPP_COPTS = [
    "-Wno-shadow",
    "-Wno-overloaded-virtual",
    "-Wno-implicit-fallthrough",
    "-Wno-conversion",
    "-Wno-sign-conversion",
    "-Wno-old-style-cast",
    "-Wno-zero-as-null-pointer-constant",
    "-Wno-effc++",
]
