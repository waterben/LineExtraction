# Phase Congruency component tests
project(test_phase_cong)

# Find test sources
file(GLOB TEST_SOURCES "test_*.cpp")

# Create test executable
add_executable(test_phase_cong ${TEST_SOURCES})

# Link libraries
target_link_libraries(test_phase_cong
    lib_phase_cong
    lib_imgproc
    lib_utility
    ${OpenCV_LIBS}
    ${GTEST_LIBRARY}
    ${GTEST_MAIN_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
)

# Include directories
target_include_directories(test_phase_cong PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/libs/imgproc/include
    ${CMAKE_SOURCE_DIR}/libs/utility/include
)

# Add test to CTest
add_test(NAME test_phase_cong COMMAND test_phase_cong)