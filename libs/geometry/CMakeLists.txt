cmake_minimum_required(VERSION 3.5...3.27)

# Create geometry library using automatic source detection
le_add_library(lib_geometry
    AUTO_TESTS
    PUBLIC_DEPS
        lib_utility
        le::opencv
        Eigen3::Eigen
    PUBLIC_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    TEST_DEPS
        ${CMAKE_THREAD_LIBS_INIT}
)
