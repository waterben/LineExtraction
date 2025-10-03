# This file contains macros and functions for including remote source code repositories.
include(ExternalProject)

set(BuildExternalCPUNum "8" CACHE STRING "Number of CPUs to use for building step")

function (compare_files fileA fileB result)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E compare_files ${fileA} ${fileB}
        RESULT_VARIABLE my_result
        OUTPUT_QUIET
        ERROR_QUIET
    )
#    message(STATUS "${my_result}")
    set(${result} ${my_result} PARENT_SCOPE)
endfunction()

function(_build_conf name path args)
    string(REPLACE ";;" " \"\" " output "cmake_minimum_required(VERSION 3.5...3.27)\nproject(conf_${name})\ninclude(ExternalProject)\nExternalProject_Add(${name} ${args})\n")
    string(REPLACE ";" " " output "${output}")
    set(DO_EXECUTE TRUE)
    #message(STATUS "${path}/${name}/CMakeLists.txt")
    if(EXISTS "${path}/${name}/CMakeLists.txt")
        file(WRITE "${path}/${name}/CMakeLists.tmp" ${output})
        compare_files("${path}/${name}/CMakeLists.tmp" "${path}/${name}/CMakeLists.txt" cmp_result)
#        message(STATUS "${cmp_result}")
        if ("${cmp_result}" STREQUAL "0")
            set(DO_EXECUTE FALSE)
        else()
            file(REMOVE "${path}/${name}/CMakeLists.tmp" ${output})
            file(WRITE "${path}/${name}/CMakeLists.txt" ${output})
        endif()
    else()
        file(WRITE "${path}/${name}/CMakeLists.txt" ${output})
    endif()
    if (${DO_EXECUTE})
        execute_process(
            COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY ${path}/${name}
            RESULT_VARIABLE result
        )
        if(result)
            message(FATAL_ERROR "CMake step ${name} failed: ${result}")
        endif()
        if (MSVC)
            execute_process(
                COMMAND ${CMAKE_COMMAND} --build . -- "/maxcpucount:${BuildExternalCPUNum}"
                WORKING_DIRECTORY ${path}/${name}
                RESULT_VARIABLE result
            )
        else()
            execute_process(
                COMMAND ${CMAKE_COMMAND} --build . -- -j "${BuildExternalCPUNum}"
                WORKING_DIRECTORY ${path}/${name}
                RESULT_VARIABLE result
            )
        endif()
        if(result)
            message(FATAL_ERROR "Build step for ${name} failed: ${result}")
        endif()
    endif()
endfunction()

function(_remove_conf_args args commands)
    foreach(command ${commands})
        list(FIND ${args} "CONF_${commands}" index)
        if (NOT (${index} EQUAL -1))
            list(REMOVE_AT args index)
            list(REMOVE_AT args index)
        endif()
	endforeach()
endfunction()

function(_check_add_args args commands has_conf_args)
    foreach(command ${commands})
        list(FIND ${args} "CONF_${commands}" index)
        if (NOT (${index} EQUAL -1))
            set(${has_conf_args} TRUE)
            list(REMOVE_AT args index)
            list(REMOVE_AT args index)
        endif()
	endforeach()
endfunction()

function(_check_conf_args args commands)
    foreach(command ${commands})
        list(FIND args_conf ${command} index)
        if (NOT (${index} EQUAL -1))
            list(REMOVE_AT args_conf ${index})
            list(REMOVE_AT args_conf ${index})
        endif()
        list(FIND args_conf "CONF_${command}" index)
        if (NOT (${index} EQUAL -1))
            list(REMOVE_AT args_conf ${index})
            list(INSERT args_conf ${index} ${command})
        endif()
	endforeach()
endfunction()

function(IncludeExternalProject name path)
    set(args_add "${ARGN}")
    #message(STATUS "init ${args_add}")
    list(FIND args_add "PREFIX" index)
    if (NOT (${index} EQUAL -1))
        list(REMOVE_AT args_add ${index})
    endif()

    set(commands DOWNLOAD_COMMAND UPDATE_COMMAND PATCH_COMMAND CONFIGURE_COMMAND BUILD_COMMAND INSTALL_COMMAND TEST_COMMAND)

    list(FIND args_add "CONF_ONLY" index)
    if (NOT (${index} EQUAL -1))
        list(REMOVE_AT args_add ${index})
        list(GET args_add ${index} value)
        list(REMOVE_AT args_add ${index})
        if (${value})
            list(APPEND args_add "PREFIX")
            list(APPEND args_add "./")
            _remove_conf_args(${args_add},${commands})
            #message(STATUS "conf only ${args_add}")
            _build_conf(${name} ${path} "${args_add}")
            return()
        endif()
    endif()

    set(args_conf "${args_add}")
    set(has_conf FALSE)

    _check_add_args(${args_add},${commands},${has_conf})

    if (${has_conf})
        _check_conf_args(${args_conf},${commands})
        list(APPEND args_add "PREFIX")
        list(APPEND args_add "./")

        #message(STATUS "has conf ${args_conf}")
        _build_conf(${name} ${path} "${args_conf}")
    endif()

    list(APPEND args_add "PREFIX")
    list(APPEND args_add "${path}/${name}")

    #message(STATUS "has add ${args_add}")
    ExternalProject_Add("${name}" "${args_add}")

endfunction()
