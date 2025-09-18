cmake_minimum_required(VERSION 3.0)

file(GLOB lsd_hpp "include/lsd/*.hpp")
add_library(lib_lsd ${LIBRARY_TYPE}
    src/lsd_fgioi_impl.cpp
    src/kht_impl.cpp
    src/lsd_edlz_impl.cpp
    ${lsd_hpp}
)

target_include_directories(lib_lsd
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lib_lsd
    PUBLIC lib_imgproc
    PUBLIC lib_edge
    PUBLIC lib_geometry
    PUBLIC lib_utility
    PUBLIC ${OpenCV_LIBS}
)

SET_TARGET_PROPERTIES(lib_lsd PROPERTIES LINKER_LANGUAGE CXX)

# Add tests subdirectory
if(ENABLE_UNIT_TEST OR BUILD_TESTING)
    add_subdirectory(tests)
endif()
