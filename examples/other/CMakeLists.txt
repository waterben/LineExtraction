cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

add_executable(test_other_houghlines houghlines.cpp)
target_link_libraries(test_other_houghlines lib_geometry le::opencv)
