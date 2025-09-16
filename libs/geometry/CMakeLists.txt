cmake_minimum_required(VERSION 3.0)

file(GLOB geometry_hpp "include/geometry/*.hpp")
add_library(lib_geometry ${LIBRARY_TYPE} ${geometry_hpp} src/render_gl.cpp src/tr.c)

target_include_directories(lib_geometry
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lib_geometry
    PUBLIC lib_utility
    PUBLIC ${OpenCV_LIBS}
)
if (TARGET Eigen3::Eigen)
    target_link_libraries(lib_geometry PUBLIC Eigen3::Eigen)
endif()

# Link OpenGL/GLUT for render_gl.cpp when available
if (OPENGL_FOUND)
    target_link_libraries(lib_geometry PUBLIC ${OPENGL_LIBRARIES})
endif()
if (GLUT_FOUND)
    target_link_libraries(lib_geometry PUBLIC ${GLUT_LIBRARIES})
endif()

SET_TARGET_PROPERTIES(lib_geometry PROPERTIES LINKER_LANGUAGE CXX)

# Add unit tests for this library when enabled
if(ENABLE_UNIT_TEST)
  add_subdirectory(tests)
endif()
