# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties
add_library(lib_qplot ${LIBRARY_TYPE} qcustomplot.cpp PlotWindow.cpp PlotManager.cpp)
