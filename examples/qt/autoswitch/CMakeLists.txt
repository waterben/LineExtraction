cmake_minimum_required(VERSION 3.5...3.27)

if (POLICY CMP0020)
	cmake_policy(SET CMP0020 OLD)
	cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0020)

# Add Libraries
add_executable(test_autoswitch autoswitch.cpp)
qt5_use_modules(test_autoswitch Core Gui Widgets OpenGL)
target_link_libraries(test_autoswitch lib_qplot3d ${OPENGL_LIBRARIES})

