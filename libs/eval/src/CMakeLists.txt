# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 3.5...3.27)

# Add source code properties
file(GLOB eval_hpp "../include/eval/*.hpp")
file(GLOB eval_cpp "*.cpp")

# Create the library
add_library(lib_eval ${LIBRARY_TYPE} ${eval_cpp} ${eval_hpp})

# Set linker language
SET_TARGET_PROPERTIES(lib_eval PROPERTIES LINKER_LANGUAGE CXX)

# Include directories
target_include_directories(lib_eval 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Dependencies
target_link_libraries(lib_eval lib_utility)
