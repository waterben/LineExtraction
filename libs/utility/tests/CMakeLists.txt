cmake_minimum_required(VERSION 3.0)

file(GLOB TEST_SOURCES "test_*.cpp")

foreach(test_src ${TEST_SOURCES})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  
  # Special handling for tests that require eval library
  if(${test_name} STREQUAL "test_console_eval_app")
    if(ENABLE_EVALUATION)
      target_link_libraries(${test_name} lib_utility lib_eval ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
    else()
      # Skip this test if evaluation is disabled
      continue()
    endif()
  else()
    target_link_libraries(${test_name} lib_utility ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
  endif()
  
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

