# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 3.0)

# Add source code properties

file(GLOB imgproc_hpp "../../include/lsd/*.hpp")

add_library(lib_lsd ${LIBRARY_TYPE} lsd_fgioi_impl.cpp kht_impl.cpp lsd_edlz_impl.cpp ${imgproc_hpp})

SET_TARGET_PROPERTIES(lib_lsd PROPERTIES LINKER_LANGUAGE CXX)

