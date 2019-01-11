cmake_minimum_required(VERSION 2.6)

# Add Libraries
file(GLOB phase_cong_cpp "*.cpp")
add_library(lib_phase_cong ${LIBRARY_TYPE} ${phase_cong_cpp} "../../include/phase_cong/PhaseCong.hpp")

