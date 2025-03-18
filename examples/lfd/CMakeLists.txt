cmake_minimum_required(VERSION 3.0)

# Add Libraries

add_executable(test_lfd_dsc dsc_test.cpp)
target_link_libraries(test_lfd_dsc lib_lsd ${OpenCV_LIBS})

add_executable(test_lfd_stereo_match stereo_match_test.cpp)
target_link_libraries(test_lfd_stereo_match lib_lsd ${OpenCV_LIBS})

add_executable(test_lfd_generic generic_test.cpp)
target_link_libraries(test_lfd_generic lib_lsd ${OpenCV_LIBS})

# add_executable(test_lfd_pair pair_test.cpp)
# target_link_libraries(test_lfd_pair lib_lsd ${OpenCV_LIBS})

# add_executable(test_lfd_match match_test.cpp)
# target_link_libraries(test_lfd_match lib_lsd ${OpenCV_LIBS})

# add_executable(test_lfd_motion_match motion_match_test.cpp)
# target_link_libraries(test_lfd_motion_match lib_lsd ${OpenCV_LIBS})


