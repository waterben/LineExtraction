cmake_minimum_required(VERSION 3.5...3.27)

# Create utility library using automatic source detection
# This is the base utility library - avoid dependencies on other LineExtraction libs
le_add_library(lib_utility
    AUTO_TESTS
    PUBLIC_DEPS
        le::opencv
        Eigen3::Eigen
    PUBLIC_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    TEST_DEPS
        ${CMAKE_THREAD_LIBS_INIT}
)
