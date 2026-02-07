cmake_minimum_required(VERSION 3.5...3.27)

file(GLOB performance_hpp "src/*.hpp")
file(GLOB performance_cpp "src/*.cpp")

add_executable(eval_performance ${performance_cpp} ${performance_hpp})
target_link_libraries(eval_performance lib_eval lib_utility lib_imgproc lib_lsd le::opencv)
