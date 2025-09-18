cmake_minimum_required(VERSION 3.5...3.27)

# Create edge library using automatic source detection
# Edge depends on both imgproc and geometry
le_add_library(lib_edge
    AUTO_TESTS
    PUBLIC_DEPS 
        lib_imgproc
        lib_geometry
        ${OpenCV_LIBS}
    PUBLIC_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIR}
    TEST_DEPS
        ${CMAKE_THREAD_LIBS_INIT}
)
