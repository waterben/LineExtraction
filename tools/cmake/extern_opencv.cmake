# This file contains macros and functions for including remote source code repositories.
include(IncludeExternalProject)

# Helper function to check if OpenCV cached paths are valid for current environment
# This is needed when switching between Docker and WSL as absolute paths differ
function(opencv_check_cached_paths_valid config_path result_var)
    set(${result_var} TRUE PARENT_SCOPE)

    if(NOT EXISTS "${config_path}")
        set(${result_var} FALSE PARENT_SCOPE)
        return()
    endif()

    # Read the config file and extract include directories
    file(READ "${config_path}" _config_content)

    # Extract the __OpenCV_INCLUDE_DIRS line
    string(REGEX MATCH "set\\(__OpenCV_INCLUDE_DIRS \"([^\"]+)\"" _match "${_config_content}")
    if(_match)
        # Get the first path from the include dirs
        string(REGEX REPLACE "set\\(__OpenCV_INCLUDE_DIRS \"([^\"]+)\".*" "\\1" _first_path "${_match}")
        # Check if the first directory exists
        if(NOT EXISTS "${_first_path}")
            message(STATUS "OpenCV cached path invalid: ${_first_path}")
            message(STATUS "This usually happens when switching between Docker and WSL.")
            message(STATUS "The extern/managed_opencv directory will be rebuilt.")
            set(${result_var} FALSE PARENT_SCOPE)
        endif()
    endif()
endfunction()

# Helper function to clean stale OpenCV extern build
function(opencv_clean_stale_build extern_path)
    set(_opencv_build_dir "${extern_path}/managed_opencv")
    set(_opencv_contrib_dir "${extern_path}/managed_opencv_contrib")

    if(EXISTS "${_opencv_build_dir}")
        message(STATUS "Removing stale OpenCV build directory: ${_opencv_build_dir}")
        file(REMOVE_RECURSE "${_opencv_build_dir}")
    endif()

    if(EXISTS "${_opencv_contrib_dir}")
        message(STATUS "Removing stale OpenCV contrib directory: ${_opencv_contrib_dir}")
        file(REMOVE_RECURSE "${_opencv_contrib_dir}")
    endif()
endfunction()

macro(opencv_extern path)
    set(OpenCV_FIND_COMPONENTS "${OpenCVComponents}")
    set(OpenCV_FOUND FALSE)

    # Determine config file path
    set(_opencv_config_path "")
    if(EXISTS "${path}/OpenCVConfig.cmake")
        set(_opencv_config_path "${path}/OpenCVConfig.cmake")
    elseif(EXISTS "${path}/build/OpenCVConfig.cmake")
        set(_opencv_config_path "${path}/build/OpenCVConfig.cmake")
    endif()

    # Check if cached paths are valid (handles Docker <-> WSL switching)
    if(_opencv_config_path)
        opencv_check_cached_paths_valid("${_opencv_config_path}" _paths_valid)
        if(NOT _paths_valid)
            # Paths are stale, need to rebuild
            message(STATUS "OpenCV extern build has stale paths, triggering rebuild...")
            opencv_clean_stale_build("${PROJECT_SOURCE_DIR}/extern")
            set(_opencv_config_path "")
        endif()
    endif()

    # Include config if valid
    if(_opencv_config_path)
        include("${_opencv_config_path}")
    endif()

    #check version
    if (DEFINED OpenCV_VERSION)
        if(${OpenCV_VERSION} VERSION_LESS ${OpenCVVersion})
            set(OpenCV_VERSION_OK FALSE)
        else()
            set(OpenCV_VERSION_OK TRUE)
        endif()

        if(NOT ${OpenCV_VERSION_OK})
            message(FATAL_ERROR "OpenCV version ${OpenCV_VERSION} found in ${OpenCV_INCLUDE_DIRS}, "
                       "but at least version ${version} is required")
        endif()
        set(OpenCV_FOUND TRUE)
        message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    endif()
endmacro()

macro(opencv_repo extern_path)
    string(REPLACE ";" "," build_list "-DBUILD_LIST=${OpenCVComponents}")

    if (${OpenCVContrib})
        set(OpenCVCMakeArgs "${OpenCVCMakeArgs} -DOPENCV_EXTRA_MODULES_PATH=${extern_path}/managed_opencv_contrib/src/managed_opencv_contrib/modules")
        IncludeExternalProject( managed_opencv_contrib ${extern_path}
        GIT_REPOSITORY https://github.com/opencv/opencv_contrib.git
        GIT_TAG "${OpenCVVersion}"
        INSTALL_COMMAND ""
        BUILD_COMMAND ""
        CONFIGURE_COMMAND ""
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        CONF_ONLY ON
    )
    endif()

    foreach(flag ${OpenCVBuildFlags})
	    list(APPEND build_flags "-DCMAKE_CXX_FLAGS=${flag}")
    endforeach()
    # Map top-level WITH_CUDA option to OpenCV CMake flag
    if (WITH_CUDA)
        set(_OpenCV_WITH_CUDA "-DWITH_CUDA=ON")
    else()
        set(_OpenCV_WITH_CUDA "-DWITH_CUDA=OFF")
    endif()

    IncludeExternalProject( managed_opencv ${extern_path}
        GIT_REPOSITORY https://github.com/opencv/opencv.git
        GIT_TAG "${OpenCVVersion}"
        INSTALL_COMMAND ""
        CMAKE_ARGS "${OpenCVCMakeArgs}" "${build_list}" "${build_flags}" "${_OpenCV_WITH_CUDA}"
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        CONF_ONLY ON
    )

    opencv_extern("${extern_path}/managed_opencv/src/managed_opencv-build")
endmacro()

set(OpenCVDetectionModes Auto System Extern Managed)
set(OpenCVDetectionMode Auto CACHE STRING "OpenCV detection mode: Auto - try system, then extern, then managed; System - use lib from system; Extern - use lib from extern folder variable; Managed - get from repository")
set(OpenCVExternPath "${PROJECT_SOURCE_DIR}/extern/opencv/build" CACHE PATH "Path to extern lib")
set_property(CACHE OpenCVDetectionMode PROPERTY STRINGS ${OpenCVDetectionModes})
set(OpenCVVersion "4.12.0" CACHE STRING "Managed opencv version (synced with Bazel MODULE.bazel)")

# Configure OpenCV components based on CUDA availability
if (WITH_CUDA)
    # Include CUDA components when CUDA is enabled
    set(OpenCVComponents core imgproc highgui video videoio imgcodecs features2d objdetect photo xfeatures2d line_descriptor cudev cudaarithm cudaimgproc cudafilters ximgproc CACHE STRING "OpenCV components to include")
else()
    # Standard components without CUDA
    set(OpenCVComponents core imgproc highgui video videoio imgcodecs features2d objdetect photo xfeatures2d line_descriptor ximgproc CACHE STRING "OpenCV components to include")
endif()
set(OpenCVCMakeArgs "-DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_WITH_DEBUG_INFO=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF -DBUILD_NEW_PYTHON_SUPPORT=OFF -DBUILD_PACKAGE=OFF -DWITH_FFMPEG=ON -DWITH_IPP=OFF -DBUILD_PNG=OFF -DBUILD_JPEG=ON -DBUILD_ZLIB=ON" CACHE STRING "Custom cmake arguments")
set(OpenCVBuildFlags -std=c++11 -fPIC -ffast-math CACHE STRING "Use C++ compiler flags")

set(OpenCVContrib ON CACHE BOOL "Also get opencv contrib modules")

list(FIND OpenCVDetectionModes ${OpenCVDetectionMode} index)
if(index EQUAL -1)
    message(WARNING "No valid mode (${OpenCVDetectionModes}) selected. Mode is set to Auto")
    set(OpenCVDetectionMode Auto)
endif()

if (${OpenCVDetectionMode} STREQUAL System)
    find_package(OpenCV ${OpenCVVersion} REQUIRED)
elseif  (${OpenCVDetectionMode} STREQUAL Extern)
    opencv_extern(${OpenCVExternPath} ${OpenCVVersion} "${OpenCVComponents}" FALSE)
    if (NOT ${OpenCV_FOUND})
        message(FATAL_ERROR "OpenCV doesn't exist on extern path: ${OpenCVExternPath}")
    endif()
elseif (${OpenCVDetectionMode} STREQUAL Managed)
    opencv_repo("${PROJECT_SOURCE_DIR}/extern" ${OpenCVVersion} "${OpenCVComponents}" )
else ()
    find_package(OpenCV ${OpenCVVersion} QUIET COMPONENTS "${OpenCVComponents}")
    if (NOT ${OpenCV_FOUND})
        opencv_extern(${OpenCVExternPath} ${OpenCVVersion} "${OpenCVComponents}" TRUE)
    endif()
    if (NOT ${OpenCV_FOUND})
        opencv_repo("${PROJECT_SOURCE_DIR}/extern" ${OpenCVVersion} "${OpenCVComponents}")
    endif()
    if (NOT ${OpenCV_FOUND})
         message(FATAL_ERROR "Failed to detect opencv!")
    endif()
endif()

set(OpenCV_INCLUDE_DIR "${OpenCV_INCLUDE_DIRS}")
set(OpenCV_LIBRARY_DIRS "${OpenCV_INSTALL_PATH}/lib")
set(OpenCV_LIBRARY_DIR "${OpenCV_LIBRARY_DIRS}")
set(OpenCV_LIBRARIES "${OpenCV_LIBS}")
