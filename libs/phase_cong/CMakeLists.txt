cmake_minimum_required(VERSION 3.0)

file(GLOB phase_cong_cpp "src/*.cpp")
add_library(lib_phase_cong ${LIBRARY_TYPE} ${phase_cong_cpp} "include/phase_cong/PhaseCong.hpp")

target_include_directories(lib_phase_cong
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lib_phase_cong
    PUBLIC lib_imgproc
    PUBLIC lib_utility
    PUBLIC ${OpenCV_LIBS}
)

SET_TARGET_PROPERTIES(lib_phase_cong PROPERTIES LINKER_LANGUAGE CXX)
