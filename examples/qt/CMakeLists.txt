cmake_minimum_required(VERSION 3.5...3.27)

if(OPENGL_FOUND)
	if (POLICY CMP0020)
	cmake_policy(SET CMP0020 OLD)
	cmake_policy(SET CMP0043 OLD)
	endif (POLICY CMP0020)
endif()
