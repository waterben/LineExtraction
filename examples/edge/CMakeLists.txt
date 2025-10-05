cmake_minimum_required(VERSION 3.5...3.27)

add_executable(test_edge_nms nms_test.cpp)
target_link_libraries(test_edge_nms lib_edge lib_imgproc le::opencv)

add_executable(test_edge_nms_variants nms_variant_test.cpp)
target_link_libraries(test_edge_nms_variants lib_edge lib_imgproc le::opencv)

add_executable(test_edge_zc zc_test.cpp)
target_link_libraries(test_edge_zc lib_edge lib_imgproc le::opencv)

add_executable(test_edge_zc_variants zc_variant_test.cpp)
target_link_libraries(test_edge_zc_variants lib_edge lib_imgproc le::opencv)

add_executable(test_edge edge_test.cpp)
target_link_libraries(test_edge lib_edge le::opencv)

add_executable(test_edge_nfa nfa_test.cpp)
target_link_libraries(test_edge_nfa lib_edge le::opencv)

add_executable(test_edge_pattern pattern_test.cpp)
target_link_libraries(test_edge_pattern lib_edge le::opencv)

add_executable(test_edge_precision edge_precision.cpp)
target_link_libraries(test_edge_precision lib_edge le::opencv )
