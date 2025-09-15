cmake_minimum_required(VERSION 3.0)

# Leave CMake file if testing is disabled
if( NOT ${ENABLE_UNIT_TEST} )
    return()
endif()

add_subdirectory( "example" )

# Auto-discover and build tests in test/tests with filenames starting with test_
# Run utility library tests (auto-discovers tests starting with test_)
add_subdirectory( "../libs/utility/tests" "libs_utility_tests" )
