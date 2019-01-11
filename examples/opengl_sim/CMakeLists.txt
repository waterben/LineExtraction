cmake_minimum_required(VERSION 2.6)

# Add Libraries

add_executable(test_AA_render AccumAntiAlias.cpp)
target_link_libraries(test_AA_render lib_geometry ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${Boost_LIBRARIES} ${OpenCV_LIBS})
if(UNIX)
target_link_libraries(test_AA_render pthread)
endif(UNIX)

add_executable(test_render_gl render_gl_test.cpp)
target_link_libraries(test_render_gl lib_geometry lib_calc_PnL ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} ${CERES_LIBRARIES})
if(UNIX)
target_link_libraries(test_render_gl pthread)
endif(UNIX)

add_executable(test_render_gl_matching_gt render_gl_matching_gt_test.cpp)
target_link_libraries(test_render_gl_matching_gt lib_geometry lib_calc_PnL ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} ${CERES_LIBRARIES})
if(UNIX)
target_link_libraries(test_render_gl_matching_gt pthread)
endif(UNIX)

add_executable(test_render_gl_all_gt render_gl_all_gt_test.cpp)
target_link_libraries(test_render_gl_all_gt lib_geometry lib_calc_PnL ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} ${CERES_LIBRARIES})
if(UNIX)
target_link_libraries(test_render_gl_all_gt pthread)
endif(UNIX)

add_executable(test_render_gl_matching render_gl_matching_test.cpp)
target_link_libraries(test_render_gl_matching lib_geometry lib_calc_PnL ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} ${CERES_LIBRARIES})
if(UNIX)
target_link_libraries(test_render_gl_matching pthread)
endif(UNIX)

add_executable(test_depth_gl depth_gl_test.cpp)
target_link_libraries(test_depth_gl lib_geometry lib_calc_PnL ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS})
if(UNIX)
target_link_libraries(test_depth_gl pthread)
endif(UNIX)

add_executable(test_render_gl_tracker render_gl_tracker.cpp)
target_link_libraries(test_render_gl_tracker lib_geometry lib_calc_PnL ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} ${CERES_LIBRARIES})
if(UNIX)
target_link_libraries(test_render_gl pthread)
endif(UNIX)
