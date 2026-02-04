cmake_minimum_required(VERSION 3.5...3.27)

# ============================================================================
# Third-Party Contributions
# ============================================================================
# This directory contains third-party code and auto-generated implementations
# that are NOT part of the main MIT-licensed LineExtraction codebase.
#
# Each subdirectory contains:
# - Its own CMakeLists.txt and BUILD.bazel
# - README.md with licensing and attribution information
#
# See individual READMEs for details:
# - matlab_coder/README.md - MATLAB Coder generated code (Academic License)
# - lsd_external/README.md - External LSD implementations (AGPL and others)
# ============================================================================

# Include subdirectories with their own build configurations
add_subdirectory(matlab_coder)
add_subdirectory(lsd_external)

# Create alias targets for convenience
add_library(contrib::matlab_coder ALIAS contrib_matlab_coder)
add_library(contrib::lsd_external ALIAS contrib_lsd_external)
