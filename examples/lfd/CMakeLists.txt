cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

add_executable(test_lfd_dsc dsc_test.cpp)
target_link_libraries(test_lfd_dsc lib_lfd lib_lsd le::opencv)

add_executable(test_lfd_stereo_match stereo_match_test.cpp)
target_link_libraries(test_lfd_stereo_match lib_lfd lib_lsd le::opencv)

add_executable(test_lfd_generic generic_test.cpp)
target_link_libraries(test_lfd_generic lib_lfd lib_lsd le::opencv)

# The following targets require SuperLU for sparse eigenvalue problems
if(TARGET superlu::superlu)
    add_executable(test_lfd_pair pair_test.cpp)
    target_link_libraries(test_lfd_pair lib_lfd lib_lsd le::opencv superlu::superlu)

    add_executable(test_lfd_motion_match motion_match_test.cpp)
    target_link_libraries(test_lfd_motion_match lib_lfd lib_lsd le::opencv superlu::superlu)

    add_executable(test_lfd_video_match video_match_test.cpp)
    target_link_libraries(test_lfd_video_match lib_lfd lib_lsd le::opencv superlu::superlu)

    # The following require opencv_contrib (line_descriptor module)
    if(OpenCV_xfeatures2d_FOUND OR OpenCV_line_descriptor_FOUND)
        add_executable(test_lfd_match match_test.cpp)
        target_link_libraries(test_lfd_match lib_lfd lib_lsd le::opencv superlu::superlu)

        add_executable(test_lfd_video_cv_matching video_cv_matching_test.cpp)
        target_link_libraries(test_lfd_video_cv_matching lib_lfd lib_lsd le::opencv superlu::superlu)

        add_executable(test_lfd_cv3_linematching cv3_linematching_test.cpp)
        target_link_libraries(test_lfd_cv3_linematching lib_lfd lib_lsd le::opencv superlu::superlu)
    endif()
endif()
