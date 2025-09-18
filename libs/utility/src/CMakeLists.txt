# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 3.5...3.27)

# Add source code properties

file(GLOB utility_hpp "../../include/utility/*.hpp")
file(GLOB utility_cpp "*.cpp")
add_library(lib_utility ${LIBRARY_TYPE} ${utility_cpp} ${utility_hpp})

SET_TARGET_PROPERTIES(lib_utility PROPERTIES LINKER_LANGUAGE CXX)
