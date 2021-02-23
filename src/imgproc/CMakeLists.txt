# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties

file(GLOB imgproc_hpp "../../include/imgproc/*.hpp")
add_library(lib_imgproc ${LIBRARY_TYPE} ${imgproc_hpp})