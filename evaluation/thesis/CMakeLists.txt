cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

add_executable(eval_color_test_sobel src/color_test_sobel.cpp)
target_link_libraries(eval_color_test_sobel lib_edge lib_imgproc le::opencv)

add_executable(eval_color_test_rcmg src/color_test_rcmg.cpp)
target_link_libraries(eval_color_test_rcmg lib_edge lib_imgproc le::opencv)

add_executable(eval_even_parameter src/even_parameter_test.cpp)
target_link_libraries(eval_even_parameter lib_imgproc dlib::dlib le::opencv)

add_executable(eval_filter_results src/filter_results.cpp)
target_link_libraries(eval_filter_results lib_imgproc le::opencv)

add_executable(eval_gradient_noise src/gradient_noise.cpp)
target_link_libraries(eval_gradient_noise lib_imgproc le::opencv)

add_executable(eval_gradient_orientation src/gradient_orientation.cpp)
target_link_libraries(eval_gradient_orientation lib_edge lib_geometry lib_imgproc le::opencv)

add_executable(eval_image_denoise src/image_denoise.cpp)
target_link_libraries(eval_image_denoise lib_edge lib_imgproc lib_utility lib_imgproc le::opencv)

add_executable(eval_phase_dc src/phase_dc.cpp)
target_link_libraries(eval_phase_dc lib_imgproc le::opencv )

add_executable(eval_spe_precision src/spe_precision.cpp)
target_link_libraries(eval_spe_precision lib_edge lib_utility lib_eval lib_imgproc le::opencv)
