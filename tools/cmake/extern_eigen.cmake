# This file contains macros and functions for including remote source code repositories.
include(IncludeExternalProject)

macro(eigen3_check_version)
  file(READ "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen3_version_header)

  string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_world_version_match "${_eigen3_version_header}")
  set(EIGEN3_WORLD_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}")
  set(EIGEN3_MAJOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}")
  set(EIGEN3_MINOR_VERSION "${CMAKE_MATCH_1}")

  set(EIGEN3_VERSION ${EIGEN3_WORLD_VERSION}.${EIGEN3_MAJOR_VERSION}.${EIGEN3_MINOR_VERSION})
  if(${EIGEN3_VERSION} VERSION_LESS ${EigenVersion})
    set(EIGEN3_VERSION_OK FALSE)
  else()
    set(EIGEN3_VERSION_OK TRUE)
  endif()

  if(NOT EIGEN3_VERSION_OK)
    message(FATAL_ERROR "Eigen3 version ${EIGEN3_VERSION} found in ${EIGEN3_INCLUDE_DIR}, "
                   "but at least version ${EigenVersion} is required")
  endif(NOT EIGEN3_VERSION_OK)
endmacro()

macro(eigen3_extern path)
    set(EIGEN3_FOUND FALSE)
    unset(EIGEN3_INCLUDE_DIR)
    if(EXISTS "${path}/Eigen")
        set(EIGEN3_INCLUDE_DIR "${path}")
        eigen3_check_version()
        set(EIGEN3_FOUND ${EIGEN3_VERSION_OK})
    endif()
endmacro()

macro(eigen3_repo extern_path)
    IncludeExternalProject( managed_eigen ${extern_path}
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG "${EigenVersion}"
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        CONF_ONLY ON
    )

    add_library(eigen IMPORTED STATIC GLOBAL)
    add_dependencies(eigen managed_eigen)

    set(EIGEN3_INCLUDE_DIR "${extern_path}/managed_eigen/src/managed_eigen")
    set(EIGEN3_FOUND TRUE)
    set(EIGEN3_VERSION ${EigenVersion})
endmacro()

set(EigenDetectionModes Auto System Extern Managed)
set(EigenDetectionMode Auto CACHE STRING "Eigen detection mode: Auto - try system, then extern, then managed; System - use lib from system; Extern - use lib from extern folder varibale; Managed - get from repository")
set(EigenExternPath "${PROJECT_SOURCE_DIR}/extern/eigen" CACHE PATH "Path to extern lib")
set_property(CACHE EigenDetectionMode PROPERTY STRINGS ${EigenDetectionModes})
set(EigenVersion "3.4.0" CACHE STRING "Required minimal eigen version (synced with Bazel MODULE.bazel)")


list(FIND EigenDetectionModes ${EigenDetectionMode} index)
if(index EQUAL -1)
    message(WARNING "No valid mode (${EigenDetectionModes}) selected. Mode is set to Auto")
    set(EigenDetectionMode Auto)
endif()

if (${EigenDetectionMode} STREQUAL System)
    find_package(Eigen3 ${EigenVersion} REQUIRED)
elseif (${EigenDetectionMode} STREQUAL Extern)
    eigen3_extern(${EigenExternPath})
    if (NOT ${EIGEN3_FOUND})
        message(FATAL_ERROR "Eigen doesn't exist on extern path: ${EigenExternPath}")
    endif()
elseif (${EigenDetectionMode} STREQUAL Managed)
    eigen3_repo("${PROJECT_SOURCE_DIR}/extern")
else ()
    set(EIGEN3_FOUND FALSE)
    #find_package(Eigen3 ${EigenVersion} QUIET)
    if (NOT ${EIGEN3_FOUND})
        eigen3_extern(${EigenExternPath})
    endif()
    if (NOT ${EIGEN3_FOUND})
        eigen3_repo("${PROJECT_SOURCE_DIR}/extern")
    endif()
    if (NOT ${EIGEN3_FOUND})
         message(FATAL_ERROR "Failed to detect eigen!")
    endif()
endif()

set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
