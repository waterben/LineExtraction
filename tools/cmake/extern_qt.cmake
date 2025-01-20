# This file contains macros and functions for including remote source code repositories.
include(IncludeExternalProject)
include(extern_tools)

macro(qt_extern quiet)
    unset(Qt5Core_FOUND)
    set(my_list "${ARGN}")
    list(FIND my_list "PATH" index)
    if (NOT ${index} EQUAL -1)
        MATH(EXPR index "${index}+1")
        list(GET my_list ${index} path)
        set(QT_ROOT ${path})
    endif()
    if (${quiet})
        find_package (Qt5Core QUIET)
    else()
        find_package (Qt5Core REQUIRED)
    endif()
endmacro()

macro(qt_repo extern_path)
    IncludeExternalProject( managed_qt ${extern_path}
        GIT_REPOSITORY https://github.com/qt/qt5.git
        GIT_TAG "${QtVersion}"
        INSTALL_COMMAND ""
        CMAKE_ARGS ${QtCMakeArgs}
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        CONF_ONLY ON
    )
    qt_extern(FALSE PATH "${extern_path}/managed_qt/src/managed_qt")
endmacro()

set(QtDetectionModes Auto System Extern Managed)
set(QtDetectionMode Auto CACHE STRING "Qt detection mode: Auto - try extern, then managed; Extern - use lib from extern folder varibale; Managed - get from repository")
set(QtExternPath "${PROJECT_SOURCE_DIR}/extern/qt" CACHE PATH "Path to extern lib")
set_property(CACHE QtDetectionMode PROPERTY STRINGS ${QtDetectionModes})
set(QtVersion "5.15.2" CACHE STRING "Managed qt version")
set(QtCMakeArgs "" CACHE STRING "Custom cmake arguments")


list(FIND QtDetectionModes ${QtDetectionMode} index)
if(index EQUAL -1)
    message(WARNING "No valid mode (${QtDetectionModes}) selected. Mode is set to Auto")
    set(QtDetectionMode Auto)
endif()

if (${QtDetectionMode} STREQUAL System)
    qt_extern(FALSE)
elseif (${QtDetectionMode} STREQUAL Extern)
    qt_extern(FALSE PATH ${QtExternPath})
elseif (${QtDetectionMode} STREQUAL Managed)
    qt_repo("${PROJECT_SOURCE_DIR}/extern")
else ()
    qt_extern(TRUE)
    if (NOT ${Qt5Core_FOUND})
        qt_extern(TRUE PATH ${DLibExternPath})
    endif()
    if (NOT ${Qt5Core_FOUND})
        qt_repo("${PROJECT_SOURCE_DIR}/extern")
    endif()
endif()


