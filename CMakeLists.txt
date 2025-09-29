cmake_minimum_required(VERSION 3.5...3.27)

# Set CMake policies to avoid warnings
cmake_policy(SET CMP0074 NEW)  # find_package uses <PackageName>_ROOT variables

# undocumented directives -> use with care!
#set(CMAKE_DISABLE_SOURCE_CHANGES ON)
#set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project(LineExtraction LANGUAGES C CXX VERSION 1.0.0)

#===============================================================================
# Project Configuration Options
#===============================================================================

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

# Auto-detect CUDA and control CUDA usage in managed OpenCV build
# First check if CUDA is available on the system
include(CheckLanguage)
check_language(CUDA)

if(CMAKE_CUDA_COMPILER)
    # CUDA is available - auto-enable by default, but allow user override
    option(WITH_CUDA "Build OpenCV with CUDA support (auto-detected: AVAILABLE)" ON)
    message(STATUS "CUDA detected: ${CMAKE_CUDA_COMPILER}")
    if(WITH_CUDA)
        enable_language(CUDA)
        message(STATUS "CUDA support enabled for OpenCV")
    else()
        message(STATUS "CUDA available but disabled by user")
    endif()
else()
    # CUDA not available - default to OFF and inform user
    option(WITH_CUDA "Build OpenCV with CUDA support (auto-detected: NOT AVAILABLE)" OFF)
    message(STATUS "CUDA not detected - CUDA support disabled")
    if(WITH_CUDA)
        message(WARNING "WITH_CUDA is ON but CUDA compiler not found. Disabling CUDA support.")
        set(WITH_CUDA OFF CACHE BOOL "Build OpenCV with CUDA support (auto-detected: NOT AVAILABLE)" FORCE)
    endif()
endif()

#===============================================================================
# CMake Setup and Utilities
#===============================================================================

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/tools/cmake")
include(LineExtractionUtils)

# Configure common settings
le_configure_common_settings()

#===============================================================================
# Dependencies Setup
#===============================================================================

# Eigen
include(extern_eigen)
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}"
    )
endif()

# dlib
include(extern_dlib)
if(NOT TARGET dlib::dlib)
    add_library(dlib::dlib INTERFACE IMPORTED)
    set_target_properties(dlib::dlib PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${DLIB_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${DLIB_LIBRARY}"
    )
endif()

# OpenCV
include(extern_opencv)

# Setup Qt dependencies if enabled - do this after OpenCV setup
if(ENABLE_QT)
    include(extern_qt)
endif()
le_setup_qt_dependencies()

# Setup testing framework
le_setup_testing()

# Enable CTest at top level
if(ENABLE_UNIT_TEST)
    enable_testing()
endif()

#===============================================================================
# Build Components
#===============================================================================

# Core libraries
add_subdirectory("libs")

# Third-party libraries (Qt plotting)
add_subdirectory("third-party")

# Examples
if(ENABLE_EXAMPLES)
    add_subdirectory("examples")
endif()

# Applications
if(ENABLE_APPS)
    add_subdirectory("apps")
endif()

# Evaluation tools
if(ENABLE_EVALUATION)
    add_subdirectory("evaluation")
endif()

#===============================================================================
# Documentation
#===============================================================================

# Build target for creating documentation
if(ENABLE_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${PROJECT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                         COMMENT "Creating documentation" VERBATIM)
        install_dir("${CMAKE_CURRENT_BINARY_DIR}/doc/html" "${PROJECT_SOURCE_DIR}/doc" "documentation")
    endif()
endif()
