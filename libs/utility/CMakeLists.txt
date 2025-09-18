cmake_minimum_required(VERSION 3.0)

file(GLOB utility_hpp "include/utility/*.hpp")
file(GLOB utility_cpp "src/*.cpp")
add_library(lib_utility ${LIBRARY_TYPE} ${utility_cpp} ${utility_hpp})

# Public include path for consumers
target_include_directories(lib_utility PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link OpenCV components used by sources (imwrite, cv::Mat, etc.)
target_link_libraries(lib_utility PUBLIC ${OpenCV_LIBS})
if (TARGET Eigen3::Eigen)
    target_link_libraries(lib_utility PUBLIC Eigen3::Eigen)
elseif(EIGEN3_INCLUDE_DIRS)
    target_include_directories(lib_utility PUBLIC ${EIGEN3_INCLUDE_DIRS})
elseif(EIGEN3_INCLUDE_DIR)
    target_include_directories(lib_utility PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

SET_TARGET_PROPERTIES(lib_utility PROPERTIES LINKER_LANGUAGE CXX)

# Add unit tests for this library when enabled
if(ENABLE_UNIT_TEST)
  add_subdirectory(tests)
endif()
