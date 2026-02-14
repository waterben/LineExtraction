cmake_minimum_required(VERSION 3.5...3.27)

project(lib_algorithm)

# Collect sources
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/algorithm/*.hpp" "include/algorithm/*.h")

# Header-only check - if no sources, create interface library
if(NOT SOURCES)
    add_library(${PROJECT_NAME} INTERFACE)
    target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${PROJECT_NAME} INTERFACE
        lib_utility
        lib_geometry
        lib_imgproc
        lib_eval
        dlib::dlib
        ${OpenCV_LIBS}
    )
else()
    add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${PROJECT_NAME} PUBLIC
        lib_utility
        lib_geometry
        lib_imgproc
        lib_eval
        dlib::dlib
        ${OpenCV_LIBS}
    )
endif()

# Tests
if(BUILD_TESTS)
    file(GLOB TEST_SOURCES "tests/test_*.cpp")
    foreach(TEST_SRC ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
        add_executable(${TEST_NAME} ${TEST_SRC})
        target_link_libraries(${TEST_NAME} ${PROJECT_NAME} gtest gtest_main)
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()
endif()
