# Copyright (c) 2014 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties
add_library(lib_qplot3d ${LIBRARY_TYPE} qwt3d_axis.cpp qwt3d_color.cpp qwt3d_coordsys.cpp qwt3d_drawable.cpp 
qwt3d_mousekeyboard.cpp qwt3d_movements.cpp qwt3d_lighting.cpp qwt3d_colorlegend.cpp qwt3d_plot.cpp qwt3d_label.cpp 
qwt3d_types.cpp qwt3d_enrichment_std.cpp qwt3d_autoscaler.cpp qwt3d_io_reader.cpp qwt3d_io.cpp qwt3d_scale.cpp
qwt3d_gridmapping.cpp qwt3d_parametricsurface.cpp qwt3d_function.cpp qwt3d_surfaceplot.cpp qwt3d_gridplot.cpp 
qwt3d_meshplot.cpp qwt3d_io_gl2ps.cpp gl2ps.c)
