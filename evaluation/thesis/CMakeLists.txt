cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

add_executable(eval_color_test_sobel color_test_sobel.cpp)
target_link_libraries(eval_color_test_sobel lib_edge lib_imgproc le::opencv)

add_executable(eval_color_test_rcmg color_test_rcmg.cpp)
target_link_libraries(eval_color_test_rcmg lib_edge lib_imgproc le::opencv)

add_executable(eval_even_parameter even_parameter_test.cpp)
target_link_libraries(eval_even_parameter lib_imgproc dlib::dlib le::opencv)

add_executable(eval_filter_results filter_results.cpp)
target_link_libraries(eval_filter_results lib_imgproc le::opencv)

add_executable(eval_gradient_noise gradient_noise.cpp)
target_link_libraries(eval_gradient_noise lib_imgproc le::opencv)

add_executable(eval_gradient_orientation gradient_orientation.cpp)
target_link_libraries(eval_gradient_orientation lib_edge lib_geometry lib_imgproc le::opencv)

add_executable(eval_image_denoise image_denoise.cpp)
target_link_libraries(eval_image_denoise lib_edge lib_imgproc lib_utility lib_imgproc le::opencv)

add_executable(eval_phase_dc phase_dc.cpp)
target_link_libraries(eval_phase_dc lib_imgproc le::opencv )

add_executable(eval_spe_precision spe_precision.cpp)
target_link_libraries(eval_spe_precision lib_edge lib_utility lib_eval lib_imgproc le::opencv)
