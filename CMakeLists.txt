cmake_minimum_required(VERSION 3.0)

# undocumented directives -> use with care!
#set(CMAKE_DISABLE_SOURCE_CHANGES ON)
#set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project( LineExtraction LANGUAGES C CXX VERSION 1.0.0 )

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

if(${BUILD_DEBUG})
    MESSAGE(STATUS "Setting build type to debug.")
    set(CMAKE_BUILD_TYPE DEBUG)
    if (NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-write-strings -pg")
    endif()
else()
    MESSAGE(STATUS "Setting build type to release.")
    set(CMAKE_BUILD_TYPE RELEASE)
    if (NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-write-strings -O3 -ffast-math")
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

include(extern_eigen)
include_directories(${EIGEN3_INCLUDE_DIR})
include(extern_dlib)
include_directories(${DLIB_INCLUDE_DIR})
include(extern_boost)
include_directories(${Boost_INCLUDE_DIR})
include(extern_opencv)
include_directories(${OpenCV_INCLUDE_DIR})

# If Qt5 shall not be used e.g. for Graphical Interface then remove
set(USE_QT5 1)

if(EXISTS ${MAINFOLDER}/extern/qt)
    list (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}/extern/qt")
endif()

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

find_package(Ceres QUIET)
IF (CERES_FOUND)
find_package(Ceres)
ELSE(CERES_FOUND)
find_package(CERESExtern)
ENDIF (CERES_FOUND)

IF (CERES_FOUND)
include_directories(${CERES_INCLUDE_DIRS})
ENDIF (CERES_FOUND)


include_directories( "include" )
link_directories("lib")

add_subdirectory( "src" )

# Build target for creating evaluation
if (${ENABLE_EVALUATION})
    add_subdirectory( "evaluation" )
endif()

# Build target for creating examples
if (${ENABLE_EXAMPLES})
    add_subdirectory( "examples" )
endif()

# Build target for creating apps
if (${ENABLE_APPS})
    add_subdirectory( "apps" )
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


