cmake_minimum_required(VERSION 3.5...3.27)

file(GLOB lfd_hpp "include/lfd/*.hpp")
add_library(lib_lfd ${LIBRARY_TYPE} ${lfd_hpp})

target_include_directories(lib_lfd
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lib_lfd
    PUBLIC lib_lsd
    PUBLIC lib_geometry
    PUBLIC lib_utility
    PUBLIC ${OpenCV_LIBS}
)

SET_TARGET_PROPERTIES(lib_lfd PROPERTIES LINKER_LANGUAGE CXX)

# Add tests subdirectory
if(ENABLE_UNIT_TEST OR BUILD_TESTING)
    add_subdirectory(tests)
endif()
