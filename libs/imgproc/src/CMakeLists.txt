# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 3.0)

# Add source code properties

file(GLOB imgproc_hpp "../../include/imgproc/*.hpp")
add_library(lib_imgproc ${LIBRARY_TYPE} ${imgproc_hpp})

SET_TARGET_PROPERTIES(lib_imgproc PROPERTIES LINKER_LANGUAGE CXX)
