# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties

file(GLOB utility_hpp "../../include/utility/*.hpp")
file(GLOB utility_cpp "*.cpp")
add_library(lib_utility ${LIBRARY_TYPE} ${utility_cpp} ${utility_hpp})
