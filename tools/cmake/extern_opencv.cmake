# This file contains macros and functions for including remote source code repositories.
include(IncludeExternalProject)

macro(opencv_extern path)
    set(OpenCV_FIND_COMPONENTS "${OpenCVComponents}")
    set(OpenCV_FOUND FALSE)
    if( EXISTS "${path}/OpenCVConfig.cmake" )
        include( "${path}/OpenCVConfig.cmake" )
    elseif (EXISTS "${path}/build/OpenCVConfig.cmake")
        include( "${path}/build/OpenCVConfig.cmake" )
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
	    list(APPEND build_flags "DCMAKE_CXX_FLAGS=${flag}")
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
set(OpenCVVersion "4.7.0" CACHE STRING "Managed opencv version")
# set(OpenCVComponents core imgproc highgui video videoio imgcodecs features2d objdetect photo xfeatures2d line_descriptor cudev cudaarithm cudaimgproc cudafilters ximgproc CACHE STRING "OpenCV components to include")
# Always include ximgproc from opencv_contrib regardless of CUDA availability
set(OpenCVComponents core imgproc highgui video videoio imgcodecs features2d objdetect photo xfeatures2d line_descriptor ximgproc CACHE STRING "OpenCV components to include")
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
