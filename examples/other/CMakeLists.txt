cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

add_executable(test_other_houghlines houghlines.cpp)
target_link_libraries(test_other_houghlines lib_geometry ${OpenCV_LIBS})

add_executable(test_other_dlib test_dlib.cpp)
target_link_libraries(test_other_dlib lib_geometry dlib::dlib)

add_executable(test_opencv_features opencv_features.cpp)
target_link_libraries(test_opencv_features lib_geometry ${OpenCV_LIBS})
