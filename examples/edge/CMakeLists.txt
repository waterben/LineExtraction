cmake_minimum_required(VERSION 3.0)

add_executable(test_edge_nms nms_test.cpp)
target_link_libraries(test_edge_nms lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_edge_nms_variants nms_variant_test.cpp)
target_link_libraries(test_edge_nms_variants lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_edge_zc zc_test.cpp)
target_link_libraries(test_edge_zc lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_edge_zc_variants zc_variant_test.cpp)
target_link_libraries(test_edge_zc_variants lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_edge edge_test.cpp)
target_link_libraries(test_edge ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_edge_nfa nfa_test.cpp)
target_link_libraries(test_edge_nfa ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_edge_pattern pattern_test.cpp)
target_link_libraries(test_edge_pattern ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_edge_precision edge_precision.cpp)
target_link_libraries(test_edge_precision ${Boost_LIBRARIES} ${OpenCV_LIBS} )
