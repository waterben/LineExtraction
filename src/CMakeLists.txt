# Copyright (c) 2014 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add Build Targets
add_subdirectory(lsd)
add_subdirectory(lfd)
add_subdirectory(stereo)
add_subdirectory(utility)
add_subdirectory(edge)
add_subdirectory(imgproc)
add_subdirectory(geometry)
add_subdirectory(calc_PnL)
add_subdirectory(phase_cong)
add_subdirectory(slam)

if(OPENGL_FOUND)
    add_subdirectory(opengl_sim)
endif()

IF (USE_QT5 AND Qt5Widgets_FOUND)
add_subdirectory(qplot)  
add_subdirectory(qplot3d)  
ENDIF (USE_QT5 AND Qt5Widgets_FOUND)

