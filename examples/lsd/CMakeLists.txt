cmake_minimum_required(VERSION 3.0)

add_executable(test_lsd lsd.cpp)
target_link_libraries(test_lsd lib_lsd ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_lsd_edge lsd_edge.cpp)
target_link_libraries(test_lsd_edge lib_lsd ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_lsd_cc lsd_cc_test.cpp)
target_link_libraries(test_lsd_cc lib_lsd ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_lsd_nfa lsd_nfa.cpp)
target_link_libraries(test_lsd_nfa lib_lsd ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_lsd_graph lsd_graph.cpp)
target_link_libraries(test_lsd_graph lib_lsd ${Boost_LIBRARIES} ${OpenCV_LIBS})

