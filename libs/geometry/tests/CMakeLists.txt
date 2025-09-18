cmake_minimum_required(VERSION 3.0)

file(GLOB TEST_SOURCES "test_*.cpp")

foreach(test_src ${TEST_SOURCES})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_link_libraries(${test_name}
    lib_geometry
    lib_utility
    ${OpenCV_LIBS}
    ${GTEST_LIBRARY}
    ${GTEST_MAIN_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
  )
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

