cmake_minimum_required(VERSION 3.5...3.27)

add_library(lib_qplot3d ${LIBRARY_TYPE}
    src/qwt3d_axis.cpp src/qwt3d_color.cpp src/qwt3d_coordsys.cpp src/qwt3d_drawable.cpp
    src/qwt3d_mousekeyboard.cpp src/qwt3d_movements.cpp src/qwt3d_lighting.cpp src/qwt3d_colorlegend.cpp src/qwt3d_plot.cpp src/qwt3d_label.cpp
    src/qwt3d_types.cpp src/qwt3d_enrichment_std.cpp src/qwt3d_autoscaler.cpp src/qwt3d_io_reader.cpp src/qwt3d_io.cpp src/qwt3d_scale.cpp
    src/qwt3d_gridmapping.cpp src/qwt3d_parametricsurface.cpp src/qwt3d_function.cpp src/qwt3d_surfaceplot.cpp src/qwt3d_gridplot.cpp
    src/qwt3d_meshplot.cpp src/qwt3d_io_gl2ps.cpp src/gl2ps.c)

set_target_properties(lib_qplot3d PROPERTIES
    AUTOMOC ON
)

target_include_directories(lib_qplot3d PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(lib_qplot3d PUBLIC Qt5::OpenGL Qt5::Widgets Qt5::Gui Qt5::Core GL GLU)
SET_TARGET_PROPERTIES(lib_qplot3d PROPERTIES LINKER_LANGUAGE CXX)
