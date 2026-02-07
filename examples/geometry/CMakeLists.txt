cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

#add_executable(test_geometry_camera camera_test.cpp)
#target_link_libraries(test_geometry_camera lib_geometry ${Boost_LIBRARIES} le::opencv)

#add_executable(test_geometry_stereo stereo_test.cpp)
#target_link_libraries(test_geometry_stereo lib_geometry ${Boost_LIBRARIES} le::opencv)

add_executable(test_geometry_stereo_pluecker stereo_pluecker_test.cpp)
target_link_libraries(test_geometry_stereo_pluecker lib_geometry le::opencv)

add_executable(test_geometry_gt gt_test.cpp)
target_link_libraries(test_geometry_gt lib_geometry le::opencv)

add_executable(test_geometry_line line_test.cpp)
target_link_libraries(test_geometry_line lib_geometry lib_imgproc le::opencv)

if(OPENGL_FOUND)
    add_executable(test_geometry_obj obj_test.cpp)
    target_link_libraries(test_geometry_obj lib_geometry ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} le::opencv)

endif()

add_executable(test_geometry_poly poly_test.cpp)
target_link_libraries(test_geometry_poly lib_imgproc lib_geometry le::opencv)
