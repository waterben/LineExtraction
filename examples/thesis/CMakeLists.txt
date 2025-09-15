cmake_minimum_required(VERSION 3.0)


add_executable(derivative derivative_filters.cpp)
target_link_libraries(derivative ${OpenCV_LIBS})

add_executable(pyramid pyramid.cpp)
target_link_libraries(pyramid ${OpenCV_LIBS})

add_executable(thesis_imgproc_threshold_examples threshold.cpp)
target_link_libraries(thesis_imgproc_threshold_examples lib_utility lib_phase_cong ${OpenCV_LIBS})

add_executable(thesis_edge_segment_examples edge_seg.cpp)
target_link_libraries(thesis_edge_segment_examples lib_utility lib_phase_cong ${OpenCV_LIBS})

add_executable(thesis_zc_examples zc_example.cpp)
target_link_libraries(thesis_zc_examples lib_utility lib_phase_cong ${OpenCV_LIBS})
