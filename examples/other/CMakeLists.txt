cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

add_executable(test_other_houghlines houghlines.cpp)
target_link_libraries(test_other_houghlines ${OpenCV_LIBS})

add_executable(test_other_dlib test_dlib.cpp)
target_link_libraries(test_other_dlib dlib::dlib ${OpenCV_LIBS})

add_executable(test_opencv_features opencv_features.cpp)
target_link_libraries(test_opencv_features dlib::dlib ${OpenCV_LIBS})
