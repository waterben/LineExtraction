cmake_minimum_required(VERSION 3.5...3.27)

add_executable(test_edge_nms src/nms_test.cpp)
target_link_libraries(test_edge_nms lib_edge lib_imgproc le::opencv)

add_executable(test_edge_nms_variants src/nms_variant_test.cpp)
target_link_libraries(test_edge_nms_variants lib_edge lib_imgproc le::opencv)

add_executable(test_edge_zc src/zc_test.cpp)
target_link_libraries(test_edge_zc lib_edge lib_imgproc le::opencv)

add_executable(test_edge_zc_variants src/zc_variant_test.cpp)
target_link_libraries(test_edge_zc_variants lib_edge lib_imgproc le::opencv)

add_executable(test_edge src/edge_test.cpp)
target_link_libraries(test_edge lib_edge le::opencv)

add_executable(test_edge_nfa src/nfa_test.cpp)
target_link_libraries(test_edge_nfa lib_edge le::opencv)

add_executable(test_edge_pattern src/pattern_test.cpp)
target_link_libraries(test_edge_pattern lib_edge le::opencv)

add_executable(test_edge_precision src/edge_precision.cpp)
target_link_libraries(test_edge_precision lib_edge le::opencv )
