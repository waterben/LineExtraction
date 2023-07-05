# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties

file(GLOB geometry_hpp "../../include/geometry/*.hpp")
add_library(lib_geometry ${LIBRARY_TYPE} ${geometry_hpp} render_gl.cpp tr.c)

SET_TARGET_PROPERTIES(lib_geometry PROPERTIES LINKER_LANGUAGE CXX)
