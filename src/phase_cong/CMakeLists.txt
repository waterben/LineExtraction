cmake_minimum_required(VERSION 3.0)

# Add Libraries
file(GLOB phase_cong_cpp "*.cpp")
add_library(lib_phase_cong ${LIBRARY_TYPE} ${phase_cong_cpp} "../../include/phase_cong/PhaseCong.hpp")

SET_TARGET_PROPERTIES(lib_phase_cong PROPERTIES LINKER_LANGUAGE CXX)
