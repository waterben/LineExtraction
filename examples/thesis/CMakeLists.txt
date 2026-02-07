cmake_minimum_required(VERSION 3.5...3.27)


add_executable(derivative derivative_filters.cpp)
target_link_libraries(derivative lib_imgproc le::opencv)

add_executable(pyramid pyramid.cpp)
target_link_libraries(pyramid lib_imgproc le::opencv)

if(ENABLE_EVALUATION)
    add_executable(thesis_imgproc_threshold_examples threshold.cpp)
    target_link_libraries(thesis_imgproc_threshold_examples lib_edge lib_imgproc lib_utility lib_eval lib_imgproc le::opencv)

    add_executable(thesis_edge_segment_examples edge_seg.cpp)
    target_link_libraries(thesis_edge_segment_examples lib_edge lib_geometry lib_imgproc lib_utility lib_eval lib_imgproc le::opencv)

    add_executable(thesis_zc_examples zc_example.cpp)
    target_link_libraries(thesis_zc_examples lib_edge lib_imgproc lib_utility lib_eval lib_imgproc le::opencv)
endif()

add_executable(thesis_opencv_features opencv_features.cpp)
target_link_libraries(thesis_opencv_features le::opencv)
