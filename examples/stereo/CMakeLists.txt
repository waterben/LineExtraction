cmake_minimum_required(VERSION 2.6)

# Add Libraries

add_executable(test_stereo_matcher stereo_matcher.cpp)
target_link_libraries(test_stereo_matcher lib_lsd lib_line_tracker lib_stereo_matcher ${OpenCV_LIBS})

add_executable(test_stereo_PnL PnL_example.cpp)
target_link_libraries(test_stereo_PnL lib_lsd lib_line_tracker lib_stereo_matcher ${OpenCV_LIBS} lib_calc_PnL)


