cmake_minimum_required(VERSION 3.5...3.27)

# ============================================================================
# External Image Processing Algorithm Implementations
# ============================================================================
# Core algorithm implementations extracted from research papers under their
# original copyrights. Header-only library used by libs/imgproc.
#
# Contents:
# - rcmg_core.hpp: RCMG edge detector (Evans, 2005-2007, All rights reserved)
# - susan_core.hpp: SUSAN edge detector (Crown Copyright 1995-1999)
#
# See README.md for detailed licensing information.
# ============================================================================

file(GLOB IMGPROC_EXTERNAL_HEADERS "include/*.h" "include/*.hpp")

add_library(contrib_imgproc_external INTERFACE)

target_include_directories(contrib_imgproc_external
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(contrib_imgproc_external
    INTERFACE
        le::opencv
)

target_sources(contrib_imgproc_external INTERFACE ${IMGPROC_EXTERNAL_HEADERS})
