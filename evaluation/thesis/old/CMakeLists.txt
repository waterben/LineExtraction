cmake_minimum_required(VERSION 3.0)

# Add Libraries

add_executable(eval_line_fit_profiling line_fit_profiling.cpp)
target_link_libraries(eval_line_fit_profiling lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_nms_results nms_results.cpp)
target_link_libraries(eval_nms_results lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_threshold_results threshold_results.cpp)
target_link_libraries(eval_threshold_results ${Boost_LIBRARIES} ${OpenCV_LIBS} )
