cmake_minimum_required(VERSION 3.0)

add_executable(thesis_imgproc_threshold_examples threshold.cpp)
target_link_libraries(thesis_imgproc_threshold_examples lib_utility lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS})

