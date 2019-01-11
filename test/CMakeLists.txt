cmake_minimum_required(VERSION 3.0)

# Leave CMake file if testing is disabled
if( NOT ${ENABLE_UNIT_TEST} )
    return()
endif()

add_subdirectory( "example" )

