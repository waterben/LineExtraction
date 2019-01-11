cmake_minimum_required(VERSION 3.0)


add_executable( test_example "example.cpp" )
target_link_libraries( test_example ${GTEST_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
add_test( NAME "occlusion::example" COMMAND test_example )

