# This file contains macros and functions for including remote source code repositories.
include(IncludeExternalProject)

# clear unwanted cached variables
macro(clear_vars components)
    foreach(component ${components})
        string(TOUPPER ${component} COMP)
        unset("Boost_${COMP}_LIBRARY_DEBUG" CACHE)
        unset("Boost_${COMP}_LIBRARY_RELEASE" CACHE)
    endforeach()
endmacro()

macro(boost_extern path quiet)
    set(BOOST_ROOT "${path}")
    if (${quiet})
        find_package (Boost ${BoostVersion} QUIET COMPONENTS ${BoostComponents})
    else()
        find_package (Boost ${BoostVersion} REQUIRED COMPONENTS ${BoostComponents})
    endif()
endmacro()

macro(boost_repo extern_path)
    set( Boost_Bootstrap_Command )
    string(REPLACE "." "_" version_underscore ${BoostVersion})    

    if( UNIX )
        set( Boost_url "http://sourceforge.net/projects/boost/files/boost/${BoostVersion}/boost_${version_underscore}.tar.gz")
        set( Boost_Bootstrap_Command ./bootstrap.sh )
        set( Boost_b2_Command ./b2 )
    elseif( WIN32 )
        set( Boost_url "http://sourceforge.net/projects/boost/files/boost/${BoostVersion}/boost_${version_underscore}.zip")
        set( Boost_Bootstrap_Command cmd /C bootstrap.bat msvc )
        set( Boost_b2_Command b2.exe )
    endif()

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(Boost_address_model 64)
    else()
        set(Boost_address_model 32)
    endif()    

    if (${BoostToolSet} STREQUAL Auto)
        set(boost_toolset "")
    else()
        set(boost_toolset "--toolset=${BoostToolSet}")
    endif()

    if (${BoostStaticLibs})
        set(linking "static")
    else()
        set(linking "shared")
    endif()

    if (${BoostStaticRuntime})
        set(linking_runtime "static")	
    else()
        set(linking_runtime "shared")
    endif()

    if (${BoostBuildDebug})
        set(build_mode "debug")
    else()
        set(build_mode "release")
    endif()

    if (${BoostMultiThreaded})
        set(thread_mode "multi")
    else()
        set(thread_mode "single")
    endif()

    foreach(component ${BoostComponents})
	    list(APPEND with_components --with-${component})
    endforeach()

    foreach(flag ${BoostBuildFlags})
	    list(APPEND cxx_flags "cxxflags=${flag}")
    endforeach()

    #message(STATUS "${Boost_b2_Command} install -j8 --prefix=${extern_path}/managed_boost/boost_install ${with_components} --disable-icu threading=${thread_mode} address-model=${Boost_address_model} link=${linking} runtime-link=${linking_runtime} variant=${build_mode} ${boost_toolset}")
    IncludeExternalProject( managed_boost ${extern_path}
        #PREFIX "${extern_path}/managed_boost"
        BUILD_IN_SOURCE 1        
        URL ${Boost_url}
        #GIT_REPOSITORY https://github.com/boostorg/boost.git
        #GIT_TAG "boost-${version}"
        UPDATE_COMMAND ""
        PATCH_COMMAND ""
        CONFIGURE_COMMAND ${Boost_Bootstrap_Command}
        BUILD_COMMAND ${Boost_b2_Command} install -j${BoostBuildNumCPU} --prefix=${extern_path}/managed_boost/boost_install ${with_components} --disable-icu threading=${thread_mode} address-model=${Boost_address_model} link=${linking} runtime-link=${linking_runtime} variant=${build_mode} ${cxx_flags} ${boost_toolset}
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        CONF_ONLY ON
    )

    boost_extern("${extern_path}/managed_boost/boost_install" FALSE)
endmacro()

set(BoostDetectionModes Auto System Extern Managed)
set(BoostDetectionMode Auto CACHE STRING "Boost detection mode: Auto - try system, then extern, then managed; System - use lib from system; Extern - use lib from extern folder varibale; Managed - get from repository")
set(BoostExternPath "${PROJECT_SOURCE_DIR}/extern/boost" CACHE PATH "Path to extern lib")
set_property(CACHE BoostDetectionMode PROPERTY STRINGS ${BoostDetectionModes})
set(BoostVersion "1.74.0" CACHE STRING "Managed boost version")
set(BoostComponents "regex;thread;system;atomic;chrono;date_time;filesystem;program_options" CACHE STRING "Boost components to include")

set(BoostMultiThreaded ON CACHE BOOL "Build/Use multi threaded libraries")
set(BoostStaticLibs ON CACHE BOOL "Build/Use static libraries")
set(BoostStaticRuntime OFF CACHE BOOL "Build/Use libraries that are linked statically to c++ runtime")
set(BoostDebugRuntime OFF CACHE BOOL "Build/Use libraries that are linked to debug c++ runtime")
set(BoostBuildDebug OFF CACHE BOOL "Build boost with debug information (managed only)")
set(BoostToolSet Auto CACHE STRING "Build tool set (managed only)")
set(BoostBuildNumCPU 8 CACHE STRING "Number of CPUs for build")
set(BoostBuildFlags "-std=c++11;-fPIC" CACHE STRING "Use C++ compiler flags")

#set( Boost_NO_SYSTEM_PATHS ON CACHE BOOL "Do not search system for Boost" )

set(Boost_USE_STATIC_LIBS ${BoostStaticLibs})
set(Boost_USE_MULTITHREADED ${BoostMultiThreaded})
set(Boost_USE_STATIC_RUNTIME ${BoostStaticRuntime})
set(Boost_USE_DEBUG_RUNTIME ${BoostDebugRuntime})

clear_vars("${BoostComponents}")

list(FIND BoostDetectionModes ${BoostDetectionMode} index)
if(index EQUAL -1)
    message(WARNING "No valid mode (${BoostDetectionModes}) selected. Mode is set to Auto")
    set(BoostDetectionMode Auto)
endif()

if (${BoostDetectionMode} STREQUAL System)
    set( Boost_NO_SYSTEM_PATHS OFF)    
    find_package(Boost ${BoostVersion} REQUIRED COMPONENTS ${BoostComponents})
elseif  (${BoostDetectionMode} STREQUAL Extern)
    set( Boost_NO_SYSTEM_PATHS ON)
    boost_extern(${BoostExternPath} FALSE)
    if (NOT ${Boost_FOUND})
        message(FATAL_ERROR "Boost doesn't exist on extern path: ${BoostExternPath}")
    endif()
elseif (${BoostDetectionMode} STREQUAL Managed)
    set( Boost_NO_SYSTEM_PATHS ON)
    boost_repo("${PROJECT_SOURCE_DIR}/extern" )
else ()
    set( Boost_NO_SYSTEM_PATHS OFF)
    find_package(Boost ${BoostVersion} QUIET COMPONENTS ${BoostComponents})
    if (NOT ${Boost_FOUND})
        clear_vars("${BoostComponents}")
        set( Boost_NO_SYSTEM_PATHS ON)
        boost_extern(${BoostExternPath} TRUE)
    endif()
    if (NOT ${Boost_FOUND})
        clear_vars("${BoostComponents}")
        boost_repo("${PROJECT_SOURCE_DIR}/extern")
    endif()
    if (NOT ${Boost_FOUND})
         message(FATAL_ERROR "Failed to detect boost!")
    endif()        
endif()

set(Boost_INCLUDE_DIR "${Boost_INCLUDE_DIRS}")        
set(Boost_LIBRARY_DIR "${Boost_LIBRARY_DIRS}")


