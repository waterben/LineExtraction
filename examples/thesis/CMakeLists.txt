cmake_minimum_required(VERSION 3.0)


add_executable(derivative derivative_filters.cpp)
target_link_libraries(derivative ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(pyramid pyramid.cpp)
target_link_libraries(pyramid ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(thesis_imgproc_threshold_examples threshold.cpp)
target_link_libraries(thesis_imgproc_threshold_examples lib_utility lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS})

