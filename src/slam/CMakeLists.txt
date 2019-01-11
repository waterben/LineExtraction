# Copyright (c) 2014 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties
file(GLOB slam_hpp "../../include/slam/*.hpp")
add_library(lib_slam ${LIBRARY_TYPE} ${slam_hpp})

IF (WIN32)
SET_TARGET_PROPERTIES(lib_slam PROPERTIES LINKER_LANGUAGE C++)
ENDIF (WIN32)
