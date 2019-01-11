cmake_minimum_required(VERSION 2.6)

# Add Libraries

IF (CERES_FOUND)
    add_executable(test_ceres_line3 test_line3.cpp)
    target_link_libraries(test_ceres_line3 ${CERES_LIBRARIES} ${OpenCV_LIBS})

    add_executable(test_ceres ceres_test.cpp)
    target_link_libraries(test_ceres ${CERES_LIBRARIES})

    add_executable(test_ceres_cube ceres_cube_test.cpp)
    target_link_libraries(test_ceres_cube ${CERES_LIBRARIES} ${OpenCV_LIBS} lib_calc_PnL)

    #add_executable(test_ceres_calibration calibration.cpp)
    #target_link_libraries(test_ceres_calibration ${OpenCV_LIBS})
ENDIF (CERES_FOUND)


