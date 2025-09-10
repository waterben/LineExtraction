# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 3.0)

# Add source code properties

file(GLOB eval_hpp "../../include/eval/*.hpp")
file(GLOB eval_cpp "*.cpp")
add_library(lib_eval ${LIBRARY_TYPE} ${eval_cpp} ${eval_hpp})

SET_TARGET_PROPERTIES(lib_eval PROPERTIES LINKER_LANGUAGE CXX)
