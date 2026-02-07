cmake_minimum_required(VERSION 3.5...3.27)

add_executable(test_imgproc_steer src/steer_test.cpp)
target_link_libraries(test_imgproc_steer lib_imgproc le::opencv)

add_executable(test_imgproc_grad_ops src/grad_test.cpp)
target_link_libraries(test_imgproc_grad_ops lib_edge lib_imgproc le::opencv)

add_executable(test_imgproc_step_line src/step_line_test.cpp)
target_link_libraries(test_imgproc_step_line lib_edge lib_imgproc le::opencv)

add_executable(test_imgproc_fft src/fft_test.cpp)
target_link_libraries(test_imgproc_fft lib_utility le::opencv)

add_executable(test_imgproc_pyramid src/pyramid_test.cpp)
target_link_libraries(test_imgproc_pyramid lib_edge lib_imgproc le::opencv)

add_executable(test_imgproc_noise src/noise_test.cpp)
target_link_libraries(test_imgproc_noise lib_imgproc le::opencv)
