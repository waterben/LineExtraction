cmake_minimum_required(VERSION 2.6)

# Add Libraries

add_executable(eval_color_test color_test.cpp)
target_link_libraries(eval_color_test ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(eval_color_test2 color_test_2.cpp)
target_link_libraries(eval_color_test2 ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(eval_gradient_orientation gradient_orientation.cpp)
target_link_libraries(eval_gradient_orientation lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_even_parameter even_parameter_test.cpp)
target_link_libraries(eval_even_parameter ${OpenCV_LIBS} )

add_executable(eval_filter_results filter_results.cpp)
target_link_libraries(eval_filter_results lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_gradient_noise gradient_noise.cpp)
target_link_libraries(eval_gradient_noise lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_phase_dc phase_dc.cpp)
target_link_libraries(eval_phase_dc lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_image_denoise image_denoise.cpp)
target_link_libraries(eval_image_denoise lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_nms_results nms_results.cpp)
target_link_libraries(eval_nms_results lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_threshold_results threshold_results.cpp)
target_link_libraries(eval_threshold_results ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_segment_results segment_results.cpp)
target_link_libraries(eval_segment_results lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_segment_split_results segment_split_results.cpp)
target_link_libraries(eval_segment_split_results lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_lsd_results lsd_results.cpp)
target_link_libraries(eval_lsd_results lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(eval_lsd_precision lsd_precision.cpp)
target_link_libraries(eval_lsd_precision ${Boost_LIBRARIES} ${OpenCV_LIBS} )

