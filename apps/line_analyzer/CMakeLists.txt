cmake_minimum_required(VERSION 3.5...3.27)

# Set CMake policies for Qt
if(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)  # Automatically link Qt executables to qtmain target on Windows
endif()
if(POLICY CMP0043)
    cmake_policy(SET CMP0043 NEW)  # Ignore COMPILE_DEFINITIONS_<Config> properties
endif()

# Include paths for headers spread across src/ and extensions/
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/accuracy
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/connectionoptimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/continuityoptimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/detectorprofile
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/imageanalyzer
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/lineanalyser2d
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/pofuncplot
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/precisionoptimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/profileanalyzer
)

# Line Analyzer Qt Application
le_add_executable(app_line_analyzer
    QT_APP
    SOURCES
        # Core
        src/main.cpp
        src/helpers.cpp
        src/controlwindow.cpp
        src/preprocessing.cpp
        src/quiver.cpp
        # Extensions
        extensions/accuracy/accuracypanel.cpp
        extensions/connectionoptimizer/connectionoptimizer.cpp
        extensions/continuityoptimizer/continuityoptimizer.cpp
        extensions/detectorprofile/detectorprofilepanel.cpp
        extensions/imageanalyzer/imageanalyzerpanel.cpp
        extensions/lineanalyser2d/analyseroptions.cpp
        extensions/lineanalyser2d/lineanalyser2d.cpp
        extensions/pofuncplot/pofuncplot.cpp
        extensions/precisionoptimizer/precisionoptimizer.cpp
        extensions/profileanalyzer/profileanalyzer.cpp
    HEADERS
        # Core
        src/helpers.h
        src/controlwindow.h
        src/preprocessing.h
        src/quiver.h
        # Extensions
        extensions/accuracy/accuracypanel.h
        extensions/connectionoptimizer/connectionoptimizer.h
        extensions/continuityoptimizer/continuityoptimizer.h
        extensions/detectorprofile/detectorprofilepanel.h
        extensions/imageanalyzer/imageanalyzerpanel.h
        extensions/lineanalyser2d/analyseroptions.h
        extensions/lineanalyser2d/lineanalyser2d.h
        extensions/pofuncplot/pofuncplot.h
        extensions/precisionoptimizer/precisionoptimizer.h
        extensions/profileanalyzer/profileanalyzer.h
        # UI files
        ui/controlwindow.ui
        ui/preprocessing.ui
        ui/quiver.ui
        extensions/accuracy/accuracypanel.ui
        extensions/connectionoptimizer/connectionoptimizer.ui
        extensions/continuityoptimizer/continuityoptimizer.ui
        extensions/detectorprofile/detectorprofilepanel.ui
        extensions/imageanalyzer/imageanalyzerpanel.ui
        extensions/lineanalyser2d/analyseroptions.ui
        extensions/lineanalyser2d/lineanalyser2d.ui
        extensions/pofuncplot/pofuncplot.ui
        extensions/precisionoptimizer/precisionoptimizer.ui
        extensions/profileanalyzer/profileanalyzer.ui
    QT_MODULES Core Gui Widgets PrintSupport OpenGL
    DEPS
        lib_algorithm
        lib_lsd
        lib_imgproc
        lib_qplot
        lib_qplot3d
        ${OPENGL_LIBRARIES}
        le::opencv
        dlib::dlib
)

# Re-enable warnings to fix them properly
# target_compile_options(app_line_analyzer PRIVATE -Wno-old-style-cast)
