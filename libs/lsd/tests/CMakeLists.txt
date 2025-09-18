# LSD component tests
project(test_lsd_unit)

# Find test sources
file(GLOB TEST_SOURCES "test_*.cpp")

# Create test executable
add_executable(test_lsd_unit ${TEST_SOURCES})

# Link libraries
target_link_libraries(test_lsd_unit
    lib_lsd
    lib_geometry
    lib_imgproc
    ${OpenCV_LIBS}
    ${GTEST_LIBRARY}
    ${GTEST_MAIN_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
)

# Include directories
target_include_directories(test_lsd_unit PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/libs/geometry/include
    ${CMAKE_SOURCE_DIR}/libs/imgproc/include
    ${CMAKE_SOURCE_DIR}/libs/utility/include
)

# Add test to CTest
add_test(NAME test_lsd_unit COMMAND test_lsd_unit)