cmake_minimum_required(VERSION 3.5...3.27)


add_executable(derivative src/derivative_filters.cpp)
target_link_libraries(derivative lib_imgproc le::opencv)

add_executable(pyramid src/pyramid.cpp)
target_link_libraries(pyramid lib_imgproc le::opencv)

if(ENABLE_EVALUATION)
    add_executable(thesis_imgproc_threshold_examples src/threshold.cpp)
    target_link_libraries(thesis_imgproc_threshold_examples lib_edge lib_imgproc lib_utility lib_eval lib_imgproc le::opencv)

    add_executable(thesis_edge_segment_examples src/edge_seg.cpp)
    target_link_libraries(thesis_edge_segment_examples lib_edge lib_geometry lib_imgproc lib_utility lib_eval lib_imgproc le::opencv)

    add_executable(thesis_zc_examples src/zc_example.cpp)
    target_link_libraries(thesis_zc_examples lib_edge lib_imgproc lib_utility lib_eval lib_imgproc le::opencv)
endif()

add_executable(thesis_opencv_features src/opencv_features.cpp)
target_link_libraries(thesis_opencv_features le::opencv)
