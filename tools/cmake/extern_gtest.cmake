# This file contains macros and functions for including remote source code repositories.
include(IncludeExternalProject)
include(extern_tools)

macro(gtest3_extern quiet)
    unset(GTEST_FOUND)
    unset(GTEST_INCLUDE_DIR CACHE)
    unset(GTEST_LIBRARY CACHE)
    unset(GTEST_LIBRARY_DEBUG CACHE)
    unset(GTEST_MAIN_LIBRARY CACHE)
    unset(GTEST_MAIN_LIBRARY_DEBUG CACHE)
    set(my_list "${ARGN}")
    list(FIND my_list "PATH" index)
    if (NOT ${index} EQUAL -1)
        MATH(EXPR index "${index}+1")
        list(GET my_list ${index} path)
        set(GTEST_ROOT ${path})
    endif()
    if (${quiet})
        find_package (GTest QUIET)
    else()
        find_package (GTest REQUIRED)
    endif()
    if (${GTEST_FOUND})
        string(FIND ${GTEST_LIBRARY} "/" index REVERSE)
        if (NOT (${index} EQUAL -1))
            string(SUBSTRING ${GTEST_LIBRARY} 0 ${index} GTEST_LIBRARY_DIR)
        endif()   
        set(GTEST_LIBRARIES "${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY}")
    endif()
endmacro()

macro(gtest3_repo extern_path)
    IncludeExternalProject( managed_gtest ${extern_path}
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG "release-${GTestVersion}"
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        CONF_ONLY ON
    )
    
    copy_files_glob("${extern_path}/managed_gtest/src/managed_gtest-build/libgtest*" "${extern_path}/managed_gtest/src/managed_gtest/lib")
    gtest3_extern(FALSE PATH "${extern_path}/managed_gtest/src/managed_gtest")
endmacro()

set(GTestDetectionModes Auto System Extern Managed)
set(GTestDetectionMode Auto CACHE STRING "GTest detection mode: Auto - try system, then extern, then managed; System - use lib from system; Extern - use lib from extern folder varibale; Managed - get from repository")
set(GTestExternPath "${PROJECT_SOURCE_DIR}/extern/gtest" CACHE PATH "Path to extern lib")
set_property(CACHE GTestDetectionMode PROPERTY STRINGS ${GTestDetectionModes})
set(GTestVersion "1.7.0" CACHE STRING "Managed gtest version")
set(GTestMakeArgs "-DCMAKE_BUILD_TYPE=RELEASE" CACHE STRING "Custom cmake arguments")

list(FIND GTestDetectionModes ${GTestDetectionMode} index)
if(index EQUAL -1)
    message(WARNING "No valid mode (${GTestDetectionModes}) selected. Mode is set to Auto")
    set(GTestDetectionMode Auto)
endif()

if (${GTestDetectionMode} STREQUAL System)
    gtest3_extern(FALSE)
elseif (${GTestDetectionMode} STREQUAL Extern)
    gtest3_extern(FALSE PATH ${GTestExternPath})
elseif (${GTestDetectionMode} STREQUAL Managed)
    gtest3_repo("${PROJECT_SOURCE_DIR}/extern")
else ()
    gtest3_extern(TRUE)
    if (NOT ${GTEST_FOUND})
        gtest3_extern(TRUE PATH ${GTestExternPath})
    endif()
    if (NOT ${GTEST_FOUND})
        gtest3_repo("${PROJECT_SOURCE_DIR}/extern")
    endif()
endif()

set(GTEST_INCLUDE_DIRS "${GTEST_INCLUDE_DIR}")
set(GTEST_LIBRARY_DIRS "${GTEST_LIBRARY_DIR}")

