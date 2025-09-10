cmake_minimum_required(VERSION 3.0)

file(GLOB imgproc_hpp "include/imgproc/*.hpp")
add_library(lib_imgproc ${LIBRARY_TYPE} ${imgproc_hpp})

target_include_directories(lib_imgproc
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lib_imgproc
    PUBLIC lib_utility
    PUBLIC ${OpenCV_LIBS}
)

SET_TARGET_PROPERTIES(lib_imgproc PROPERTIES LINKER_LANGUAGE CXX)
