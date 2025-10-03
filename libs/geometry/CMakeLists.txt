cmake_minimum_required(VERSION 3.5...3.27)

# Create geometry library using automatic source detection
le_add_library(lib_geometry
    AUTO_TESTS
#    EXCLUDED_SOURCES
#        render_gl.cpp
    PUBLIC_DEPS
        lib_utility
        ${OpenCV_LIBS}
        Eigen3::Eigen
        $<$<BOOL:${OPENGL_FOUND}>:${OPENGL_LIBRARIES}>
        $<$<BOOL:${GLUT_FOUND}>:${GLUT_LIBRARIES}>
    PUBLIC_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIR}
    TEST_DEPS
        ${CMAKE_THREAD_LIBS_INIT}
)
