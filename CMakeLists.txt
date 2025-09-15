cmake_minimum_required(VERSION 3.0)

# undocumented directives -> use with care!
#set(CMAKE_DISABLE_SOURCE_CHANGES ON)
#set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project( LineExtraction LANGUAGES C CXX VERSION 1.0.0 )

# set(CMAKE_C_COMPILER "/usr/bin/gcc-10")
# set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")
# Configurable environment variables that are stored in CMakeCache. These 
# variables can be used to adjust the projects build behaviour (e.g. ccmake).
set(VERBOSE_MAKEFILE ON CACHE BOOL "Overwrite cmake verbose makefile")
set(BUILD_DEBUG OFF CACHE BOOL "Build in debug mode")
set(SEPARATE_FOLDER_BUILDS ON CACHE BOOL "Generate separate folders for release and debug builds")
set(BUILD_STATIC ON CACHE BOOL "Build static library instead of shared")
set(ENABLE_UNIT_TEST ON CACHE BOOL "Enable google unit test")
set(ENABLE_DOCS ON CACHE BOOL "Enable doxygen documentation genereation")
set(ENABLE_EVALUATION ON CACHE BOOL "Enable building of algorithm evaluation")
set(ENABLE_APPS ON CACHE BOOL "Enable building of applications")
set(ENABLE_EXAMPLES ON CACHE BOOL "Enable building of code examples")
set(ENABLE_QT ON CACHE BOOL "Enable building of qt code")

# Control CUDA usage in managed OpenCV build
option(WITH_CUDA "Build OpenCV with CUDA support" OFF)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(${BUILD_DEBUG})
    MESSAGE(STATUS "Setting build type to debug.")
    set(CMAKE_BUILD_TYPE DEBUG)
    if (NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-write-strings -pg")
    endif()
else()
    MESSAGE(STATUS "Setting build type to release.")
    set(CMAKE_BUILD_TYPE RELEASE)
    if (NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-write-strings -O3 -ffast-math")
    endif()
endif()


if(${BUILD_STATIC})
    MESSAGE(STATUS "Setting library type to static.")
    set(LIBRARY_TYPE STATIC)
    add_definitions(-D_STATIC)
else()
    MESSAGE(STATUS "Setting library type to dynamic.")
    set(LIBRARY_TYPE SHARED)
    add_definitions(-D_SHARED)
endif()


set(MAINFOLDER ${PROJECT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/tools/cmake")

# Detect and configure Eigen. Prefer our helper which supports
# system, extern, or managed (already vendored) sources.
include(extern_eigen)
# Make Eigen headers visible to all targets as before
include_directories(${EIGEN3_INCLUDE_DIRS})

# Some extern packages (e.g., OpenCV modules) expect the imported target
# `Eigen3::Eigen`. Provide a lightweight interface target when not found.
if (NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}"
    )
endif()

include(extern_dlib)
include_directories(${DLIB_INCLUDE_DIR})

include(extern_boost)
include_directories(${Boost_INCLUDE_DIR})

include(extern_opencv)
include_directories(${OpenCV_INCLUDE_DIR})

MESSAGE(STATUS "${OpenCV_INCLUDE_DIR}")
MESSAGE(STATUS "${OpenCV_LIBS}")

if(ENABLE_QT)
    #include(extern_qt)
    find_package (Qt5Core)
    if (Qt5Core_FOUND)
        MESSAGE(STATUS "Qt5 found")
        # Tell CMake to run moc when necessary
        set(CMAKE_AUTOMOC ON)
        # Tell CMake to run uic when necessary
        set(CMAKE_AUTOUIC ON)

        # As moc files are generated in the binary dir, tell CMake
        # to always look for includes there:
        set(CMAKE_INCLUDE_CURRENT_DIR ON)

        # not required, only for qt apps
        # place IF (Qt5Widgets_FOUND) around qt project, to add them only if
        # qt was added
        # for .ui files, you need to run uic from qt manually (uic file.ui -o ui_file.h)
        # or add execute_process(COMMAND uic path/file.ui -o path/ui_file.h)
        #find_package (Qt5Core)
        find_package (Qt5Gui)
        find_package (Qt5Widgets)
        find_package (Qt5PrintSupport)
        include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5PrintSupport_INCLUDE_DIRS})
        #MESSAGE(STATUS ${Qt5Gui_LIBRARY_DIRS})
    endif ()
endif ()

find_package(GLUT QUIET)
if (GLUT_FOUND)
message(STATUS "GLUT (OpenGL) found")
include_directories(${GLUT_INCLUDE_DIRS})
endif (GLUT_FOUND)

find_package(OpenGL QUIET)
if (OPENGL_FOUND)
message(STATUS "OpenGL found")
include_directories(${OPENGL_INCLUDE_DIR})
endif (OPENGL_FOUND)

find_package (LAPACKExtern QUIET)
if (NOT LAPACK_FOUND)
find_package (LAPACK)
SET(LAPACK_LIBS ${LAPACK_LIBRARIES})
endif(NOT LAPACK_FOUND)

find_package (SUPERLU)
find_package (ARPACK)

include_directories(
    "libs/utility/include"
    "libs/edge/include"
    "libs/imgproc/include"
    "libs/lsd/include"
    "libs/geometry/include"
    "libs/lfd/include"
    "libs/phase_cong/include"
    "libs/opengl_sim/include"
    "libs/qplot/include"
    "libs/qplot3d/include"
)
link_directories("lib")

add_subdirectory( "libs" )

# Build target for creating examples
if (${ENABLE_EXAMPLES})
    add_subdirectory( "examples" )
endif()

# Build target for creating apps
if (${ENABLE_APPS})
    add_subdirectory( "apps" )
endif()

# Build target for creating apps
if (${ENABLE_EVALUATION})
    add_subdirectory( "evaluation" )
endif()

# Build target for creating unit test
if( ${ENABLE_UNIT_TEST} )
    find_package(Threads REQUIRED)    
    include(extern_gtest)
    include_directories(${GTEST_INCLUDE_DIRS})    
    enable_testing()
    add_subdirectory( "test" )
endif()

# Build target for creating documentation
if (${ENABLE_DOCS})
    find_package(Doxygen)
    if( ${DOXYGEN_FOUND} )
        configure_file( ${PROJECT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY )
        add_custom_target( doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Creating documentation" VERBATIM )
        install_dir( "${CMAKE_CURRENT_BINARY_DIR}/doc/html" "${PROJECT_SOURCE_DIR}/doc" "documentation" )
    endif()
endif()
