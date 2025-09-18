# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 3.0)

# Evaluation library
if (ENABLE_EVALUATION)
    # Add source directory (this creates the library with dependencies)
    add_subdirectory(src)

    # Enable testing if requested and directory exists
    if(ENABLE_UNIT_TEST AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
        add_subdirectory(tests)
    endif()
endif()