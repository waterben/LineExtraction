# LFD (Line Feature Descriptor) component tests
project(test_lfd)

# Find test sources
file(GLOB TEST_SOURCES "test_*.cpp")

# Create test executable
add_executable(test_lfd ${TEST_SOURCES})

# Link libraries
target_link_libraries(test_lfd
    lib_lfd
    lib_geometry
    lib_imgproc
    lib_utility
    ${OpenCV_LIBS}
    ${GTEST_LIBRARY}
    ${GTEST_MAIN_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
)

# Include directories
target_include_directories(test_lfd PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/libs/geometry/include
    ${CMAKE_SOURCE_DIR}/libs/imgproc/include
    ${CMAKE_SOURCE_DIR}/libs/utility/include
)

# Add test to CTest
add_test(NAME test_lfd COMMAND test_lfd)