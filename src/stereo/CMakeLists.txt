cmake_minimum_required(VERSION 2.6)

# Add Libraries

add_library(lib_stereo_matcher ${LIBRARY_TYPE} stereo_matcher.cpp)

target_link_libraries(lib_stereo_matcher lib_lsd lib_line_tracker ${OpenCV_LIBS} lib_calc_PnL)

