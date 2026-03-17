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

    # Modern CMake config-mode sets GTest_FOUND and imported targets
    # (GTest::gtest, GTest::gtest_main) but not the legacy GTEST_* variables.
    # Bridge the gap so the rest of the build system works uniformly.
    if (NOT GTEST_FOUND AND (GTest_FOUND OR TARGET GTest::gtest))
        set(GTEST_FOUND TRUE)
    endif()

    if (${GTEST_FOUND})
        # Prefer imported targets when available (modern GTestConfig.cmake)
        if (TARGET GTest::gtest AND NOT GTEST_LIBRARY)
            set(GTEST_LIBRARY GTest::gtest)
            set(GTEST_MAIN_LIBRARY GTest::gtest_main)
            # Extract include dir from imported target
            get_target_property(_gtest_inc GTest::gtest INTERFACE_INCLUDE_DIRECTORIES)
            if (_gtest_inc)
                set(GTEST_INCLUDE_DIR "${_gtest_inc}")
            endif()
            unset(_gtest_inc)
        endif()

        if (NOT TARGET GTest::gtest)
            # Legacy path: derive library directory from GTEST_LIBRARY path
            string(FIND "${GTEST_LIBRARY}" "/" index REVERSE)
            if (NOT (${index} EQUAL -1))
                string(SUBSTRING "${GTEST_LIBRARY}" 0 ${index} GTEST_LIBRARY_DIR)
            endif()
        endif()
        set(GTEST_LIBRARIES "${GTEST_LIBRARY};${GTEST_MAIN_LIBRARY}")
    endif()
endmacro()

macro(gtest3_repo extern_path)
    # Note: GoogleTest changed tag naming from "release-X.Y.Z" to "vX.Y.Z" starting with 1.10.0
    IncludeExternalProject( managed_gtest ${extern_path}
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG "v${GTestVersion}"
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        CONF_ONLY ON
    )

    # GoogleTest builds libs into managed_gtest-build/lib/ subdirectory
    set(GTEST_ROOT "${extern_path}/managed_gtest/src/managed_gtest")
    set(GTEST_BUILD_DIR "${extern_path}/managed_gtest/src/managed_gtest-build")
    set(GTEST_INCLUDE_DIR "${GTEST_ROOT}/googletest/include")
    set(GTEST_LIBRARY "${GTEST_BUILD_DIR}/lib/libgtest.a")
    set(GTEST_MAIN_LIBRARY "${GTEST_BUILD_DIR}/lib/libgtest_main.a")
    set(GTEST_LIBRARY_DIR "${GTEST_BUILD_DIR}/lib")
    set(GTEST_LIBRARIES "${GTEST_LIBRARY};${GTEST_MAIN_LIBRARY}")
    set(GTEST_FOUND TRUE)
endmacro()

set(GTestDetectionModes Auto System Extern Managed)
set(GTestDetectionMode Auto CACHE STRING "GTest detection mode: Auto - try system, then extern, then managed; System - use lib from system; Extern - use lib from extern folder varibale; Managed - get from repository")
set(GTestExternPath "${PROJECT_SOURCE_DIR}/extern/gtest" CACHE PATH "Path to extern lib")
set_property(CACHE GTestDetectionMode PROPERTY STRINGS ${GTestDetectionModes})
set(GTestVersion "1.15.2" CACHE STRING "Managed gtest version (synced with Bazel MODULE.bazel)")
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
