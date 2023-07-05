# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties

file(GLOB edge_hpp "../../include/edge/*.hpp")
add_library(lib_edge ${LIBRARY_TYPE} ${edge_hpp})

SET_TARGET_PROPERTIES(lib_edge PROPERTIES LINKER_LANGUAGE CXX)
