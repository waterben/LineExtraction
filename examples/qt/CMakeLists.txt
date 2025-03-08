cmake_minimum_required(VERSION 3.0)

if(OPENGL_FOUND)
	if (POLICY CMP0020)
	cmake_policy(SET CMP0020 OLD)
	cmake_policy(SET CMP0043 OLD)
	endif (POLICY CMP0020)

	# Add Libraries
	add_executable(test_qplot3d test_qplot3d)
	qt5_use_modules(test_qplot3d Core Gui Widgets OpenGL)
	target_link_libraries(test_qplot3d lib_qplot3d ${OPENGL_LIBRARIES} ${OpenCV_LIBS})
    
    #add_subdirectory(autoswitch)
    #add_subdirectory(axes)
    #add_subdirectory(enrichments)
    #add_subdirectory(mesh2)
endif()

