cmake_minimum_required(VERSION 2.6)

add_executable(test_imgproc_steer steer_test.cpp)
target_link_libraries(test_imgproc_steer ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_imgproc_grad_ops grad_test.cpp)
target_link_libraries(test_imgproc_grad_ops lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_imgproc_step_line step_line_test.cpp)
target_link_libraries(test_imgproc_step_line lib_phase_cong ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_imgproc_fft fft_test.cpp)
target_link_libraries(test_imgproc_fft ${OpenCV_LIBS})

add_executable(test_imgproc_pyramid pyramid_test.cpp)
target_link_libraries(test_imgproc_pyramid ${OpenCV_LIBS})

add_executable(test_imgproc_noise noise_test.cpp)
target_link_libraries(test_imgproc_noise ${OpenCV_LIBS})
