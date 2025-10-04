cmake_minimum_required(VERSION 3.5...3.27)

# LFD (Line Feature Detector) library with automatic source detection
le_add_library(lib_lfd
    AUTO_TESTS
    PUBLIC_DEPS
        lib_lsd
        lib_geometry
        lib_utility
        ${OpenCV_LIBS}
    PUBLIC_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIR}
    TEST_DEPS
        ${CMAKE_THREAD_LIBS_INIT}
)
