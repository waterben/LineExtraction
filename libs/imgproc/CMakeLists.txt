cmake_minimum_required(VERSION 3.5...3.27)

# Create imgproc library using automatic source detection
le_add_library(lib_imgproc
    AUTO_TESTS
    PUBLIC_DEPS
        lib_utility
        lib_geometry
        le::opencv
        contrib_matlab_coder  # MATLAB Coder generated code
    PUBLIC_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    TEST_DEPS
        ${CMAKE_THREAD_LIBS_INIT}
        contrib_matlab_coder  # Tests need direct access to PhaseCong.hpp
)
