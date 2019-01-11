# Copyright (c) 2014 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties

file(GLOB imgproc_hpp "../../include/lsd/*.hpp")

add_library(lib_lsd ${LIBRARY_TYPE} lsd_fgioi_impl.cpp kht_impl.cpp lsd_edlz_impl.cpp ${imgproc_hpp})


