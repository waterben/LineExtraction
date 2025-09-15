cmake_minimum_required(VERSION 3.0)

add_library(lib_qplot ${LIBRARY_TYPE} src/qcustomplot.cpp src/PlotWindow.cpp src/PlotManager.cpp)

qt5_use_modules(lib_qplot Core Gui Widgets PrintSupport)
SET_TARGET_PROPERTIES(lib_qplot PROPERTIES LINKER_LANGUAGE CXX)

