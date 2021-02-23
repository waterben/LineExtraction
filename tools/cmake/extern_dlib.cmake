# This file contains macros and functions for including remote source code repositories.
include(IncludeExternalProject)
include(extern_tools)

macro(dlib_extern quiet)
    unset(DLIB_FOUND)
    unset(DLIB_INCLUDE_DIRS CACHE)
    unset(DLIB_LIBRARY CACHE)
    unset(DLIB_LIBRARY_DEBUG CACHE)
    set(my_list "${ARGN}")
    list(FIND my_list "PATH" index)
    if (NOT ${index} EQUAL -1)
        MATH(EXPR index "${index}+1")
        list(GET my_list ${index} path)
        set(DLIB_ROOT ${path})
    endif()
    if (${quiet})
        find_package (DLIB QUIET)
    else()
        find_package (DLIB REQUIRED)
    endif()
    if (${DLIB_FOUND})
        string(FIND ${DLIB_LIBRARY} "/" index REVERSE)
        if (NOT (${index} EQUAL -1))
            string(SUBSTRING ${DLIB_LIBRARY} 0 ${index} DLIB_LIBRARY_DIRS)
        endif()   
    endif()
endmacro()

macro(dlib_repo extern_path)
    IncludeExternalProject( managed_dlib ${extern_path}
        GIT_REPOSITORY https://github.com/davisking/dlib.git
        GIT_TAG "v${DLibVersion}"
        INSTALL_COMMAND ""
        CMAKE_ARGS ${DLibCMakeArgs}
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        CONF_ONLY ON
    )
    copy_files_glob("${extern_path}/managed_dlib/src/managed_dlib-build/dlib/*dlib*" "${extern_path}/managed_dlib/src/managed_dlib/lib")
    dlib_extern(FALSE PATH "${extern_path}/managed_dlib/src/managed_dlib")
endmacro()

set(DLibDetectionModes Auto System Extern Managed)
set(DLibDetectionMode Auto CACHE STRING "DLib detection mode: Auto - try extern, then managed; Extern - use lib from extern folder varibale; Managed - get from repository")
set(DLibExternPath "${PROJECT_SOURCE_DIR}/extern/dlib" CACHE PATH "Path to extern lib")
set_property(CACHE DLibDetectionMode PROPERTY STRINGS ${DLibDetectionModes})
set(DLibVersion "19.15" CACHE STRING "Managed dlib version")
set(DLibCMakeArgs "-DCMAKE_BUILD_TYPE=RELEASE -DUSE_AVX_INSTRUCTIONS=1 -DUSE_SSE4_INSTRUCTIONS=1 -DUSE_SSE2_INSTRUCTIONS=1" CACHE STRING "Custom cmake arguments")


list(FIND DLibDetectionModes ${DLibDetectionMode} index)
if(index EQUAL -1)
    message(WARNING "No valid mode (${DLibDetectionModes}) selected. Mode is set to Auto")
    set(DLibDetectionMode Auto)
endif()

if (${DLibDetectionMode} STREQUAL System)
    dlib_extern(FALSE)
elseif (${DLibDetectionMode} STREQUAL Extern)
    dlib_extern(FALSE PATH ${DLibExternPath})
elseif (${DLibDetectionMode} STREQUAL Managed)
    dlib_repo("${PROJECT_SOURCE_DIR}/extern")
else ()
    dlib_extern(TRUE)
    if (NOT ${DLIB_FOUND})
        dlib_extern(TRUE PATH ${DLibExternPath})
    endif()
    if (NOT ${DLIB_FOUND})
        dlib_repo("${PROJECT_SOURCE_DIR}/extern")
    endif()
endif()

set(DLIB_INCLUDE_DIR "${DLIB_INCLUDE_DIRS}")
set(DLIB_LIBRARY_DIR "${DLIB_LIBRARY_DIRS}")


