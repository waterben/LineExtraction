# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 3.5...3.27)

# Evaluation library - only build if evaluation is enabled
if(ENABLE_EVALUATION)
    # Create eval library using automatic source detection
    le_add_library(lib_eval
        AUTO_TESTS
        PUBLIC_DEPS
            lib_utility
            lib_edge
            lib_imgproc
        PUBLIC_INCLUDES
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        TEST_DEPS
            ${CMAKE_THREAD_LIBS_INIT}
    )
endif()
