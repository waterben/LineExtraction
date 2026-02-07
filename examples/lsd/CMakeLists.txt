cmake_minimum_required(VERSION 3.5...3.27)

# LSD Examples using standard CMake (can be converted to le_add_example later)
add_executable(test_lsd src/lsd.cpp)
target_link_libraries(test_lsd lib_lsd le::opencv)

add_executable(test_lsd_edge src/lsd_edge.cpp)
target_link_libraries(test_lsd_edge lib_lsd le::opencv)

add_executable(test_lsd_cc src/lsd_cc_test.cpp)
target_link_libraries(test_lsd_cc lib_lsd le::opencv)

add_executable(test_lsd_nfa src/lsd_nfa.cpp)
target_link_libraries(test_lsd_nfa lib_lsd le::opencv)

add_executable(test_lsd_graph src/lsd_graph.cpp)
target_link_libraries(test_lsd_graph lib_lsd le::opencv)

add_executable(test_lsd_video src/lsd_video.cpp)
target_link_libraries(test_lsd_video lib_lsd le::opencv)
