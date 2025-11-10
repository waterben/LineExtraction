cmake_minimum_required(VERSION 3.5...3.27)

# LSD Examples using standard CMake (can be converted to le_add_example later)
add_executable(test_lsd lsd.cpp)
target_link_libraries(test_lsd lib_lsd le::opencv)

add_executable(test_lsd_edge lsd_edge.cpp)
target_link_libraries(test_lsd_edge lib_lsd le::opencv)

add_executable(test_lsd_cc lsd_cc_test.cpp)
target_link_libraries(test_lsd_cc lib_lsd le::opencv)

add_executable(test_lsd_nfa lsd_nfa.cpp)
target_link_libraries(test_lsd_nfa lib_lsd le::opencv)

add_executable(test_lsd_graph lsd_graph.cpp)
target_link_libraries(test_lsd_graph lib_lsd le::opencv)
