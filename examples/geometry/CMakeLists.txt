cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

add_executable(test_geometry_stereo_pluecker src/stereo_pluecker_test.cpp)
target_link_libraries(test_geometry_stereo_pluecker lib_geometry le::opencv)

add_executable(test_geometry_gt src/gt_test.cpp)
target_link_libraries(test_geometry_gt lib_geometry le::opencv)

add_executable(test_geometry_line src/line_test.cpp)
target_link_libraries(test_geometry_line lib_geometry lib_imgproc le::opencv)

add_executable(test_geometry_poly src/poly_test.cpp)
target_link_libraries(test_geometry_poly lib_imgproc lib_geometry le::opencv)
