cmake_minimum_required(VERSION 3.0)

file(GLOB opengl_sim_hpp "include/opengl_sim/*.hpp")
add_library(lib_opengl_sim ${LIBRARY_TYPE} ${opengl_sim_hpp})

target_include_directories(lib_opengl_sim
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lib_opengl_sim
    PUBLIC lib_lsd
    PUBLIC lib_geometry
    PUBLIC lib_edge
    PUBLIC ${OpenCV_LIBS}
)

if (OPENGL_FOUND)
    target_link_libraries(lib_opengl_sim PUBLIC ${OPENGL_LIBRARIES})
endif()
if (GLUT_FOUND)
    target_link_libraries(lib_opengl_sim PUBLIC ${GLUT_LIBRARIES})
endif()

SET_TARGET_PROPERTIES(lib_opengl_sim PROPERTIES LINKER_LANGUAGE CXX)
