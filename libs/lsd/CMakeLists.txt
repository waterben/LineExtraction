cmake_minimum_required(VERSION 3.5...3.27)

# LSD library with automatic source detection or specific sources
le_add_library(lib_lsd
    AUTO_TESTS
    # Use automatic source detection from src/ directory
    PUBLIC_DEPS
        lib_imgproc
        lib_edge
        lib_geometry
        lib_utility
        le::opencv
    PUBLIC_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    TEST_DEPS
        ${CMAKE_THREAD_LIBS_INIT}
)
