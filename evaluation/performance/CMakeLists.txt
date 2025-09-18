cmake_minimum_required(VERSION 3.5...3.27)

file(GLOB performance_hpp "*.hpp")
file(GLOB performance_cpp "*.cpp")

add_executable(eval_performance ${performance_cpp} ${performance_hpp})
target_link_libraries(eval_performance lib_utility lib_phase_cong lib_lsd ${OpenCV_LIBS})
