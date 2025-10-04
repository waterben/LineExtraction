cmake_minimum_required(VERSION 3.5...3.27)

add_library(lib_qplot ${LIBRARY_TYPE}
    src/qcustomplot.cpp
    src/PlotWindow.cpp
    src/PlotManager.cpp
    include/qplot/qcustomplot.h
    include/qplot/PlotWindow.h
)

set_target_properties(lib_qplot PROPERTIES
    AUTOMOC ON
)

target_include_directories(lib_qplot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIR}
)
target_link_libraries(lib_qplot PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport ${OpenCV_LIBS} lib_geometry)
SET_TARGET_PROPERTIES(lib_qplot PROPERTIES LINKER_LANGUAGE CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lib_qplot PRIVATE
        -Wno-effc++
        -Wno-float-conversion
        -Wno-sign-conversion
        -Wno-conversion
        -Wno-unused-parameter
        -Wno-deprecated-declarations
    )
endif()
