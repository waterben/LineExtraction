cmake_minimum_required(VERSION 3.0)

# Add Libraries

#add_executable(test_geometry_camera camera_test.cpp)
#target_link_libraries(test_geometry_camera lib_geometry ${Boost_LIBRARIES} ${OpenCV_LIBS})

#add_executable(test_geometry_stereo stereo_test.cpp)
#target_link_libraries(test_geometry_stereo lib_geometry ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_geometry_stereo_pluecker stereo_pluecker_test.cpp)
target_link_libraries(test_geometry_stereo_pluecker lib_geometry ${OpenCV_LIBS})

add_executable(test_geometry_gt gt_test.cpp)
target_link_libraries(test_geometry_gt lib_geometry ${OpenCV_LIBS})

add_executable(test_geometry_line line_test.cpp)
target_link_libraries(test_geometry_line lib_geometry ${OpenCV_LIBS})

if(OPENGL_FOUND)
    add_executable(test_geometry_obj obj_test.cpp)
    target_link_libraries(test_geometry_obj lib_geometry ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS})

    add_executable(test_render_gl_tr render_gl_tr_test.cpp)
    target_link_libraries(test_render_gl_tr lib_geometry ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS})
    if(UNIX)
    target_link_libraries(test_render_gl_tr pthread)
    endif(UNIX)
endif()

add_executable(test_geometry_poly poly_test.cpp)
target_link_libraries(test_geometry_poly lib_geometry ${OpenCV_LIBS})
