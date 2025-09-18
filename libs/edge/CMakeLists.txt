cmake_minimum_required(VERSION 3.0)

file(GLOB edge_hpp "include/edge/*.hpp")
add_library(lib_edge ${LIBRARY_TYPE} ${edge_hpp})

target_include_directories(lib_edge
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lib_edge
    PUBLIC lib_imgproc
    PUBLIC lib_geometry
    PUBLIC ${OpenCV_LIBS}
)

SET_TARGET_PROPERTIES(lib_edge PROPERTIES LINKER_LANGUAGE CXX)

# Add tests if testing is enabled
if(ENABLE_UNIT_TEST OR BUILD_TESTING)
    add_subdirectory(tests)
endif()
