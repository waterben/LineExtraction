cmake_minimum_required(VERSION 3.5...3.27)

if (POLICY CMP0020)
	cmake_policy(SET CMP0020 OLD)
	cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0020)

# Add Libraries
add_executable(test_mesh designerworkaround.cpp lightingdlg.cpp
main.cpp mesh.cpp mesh2mainwindow.cpp lightingdlgbase.ui mesh2mainwindowbase.ui)
qt5_use_modules(test_mesh Core Gui Widgets OpenGL)
target_link_libraries(test_mesh lib_qplot3d ${OPENGL_LIBRARIES})

