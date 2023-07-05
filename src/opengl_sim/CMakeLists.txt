# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties

file(GLOB opengl_sim "../../include/opengl_sim/*.hpp")
add_library(lib_opengl_sim ${LIBRARY_TYPE} ${opengl_sim})

SET_TARGET_PROPERTIES(lib_opengl_sim PROPERTIES LINKER_LANGUAGE CXX)
