cmake_minimum_required(VERSION 3.5...3.27)

# ============================================================================
# External LSD Implementations Library
# ============================================================================
# Contains implementations from external sources with various licenses:
# - lsd_fgioi_impl.cpp: Grompone von Gioi's LSD (AGPL)
# - lsd_edlz_impl.cpp: EDLines Zero
# - kht_impl.cpp: Kernel-based Hough Transform
#
# See README.md for detailed information about sources and licensing.
# ============================================================================

file(GLOB LSD_EXTERNAL_SOURCES "src/*.cpp")
file(GLOB LSD_EXTERNAL_HEADERS "include/*.h" "include/*.hpp")

add_library(contrib_lsd_external ${LSD_EXTERNAL_SOURCES} ${LSD_EXTERNAL_HEADERS})

target_include_directories(contrib_lsd_external
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(contrib_lsd_external
    PUBLIC
        le::opencv
)

# Disable warnings for external code
target_compile_options(contrib_lsd_external PRIVATE
    -Wno-old-style-cast
    -Wno-conversion
    -Wno-sign-conversion
    -Wno-effc++
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-sign-compare
)
