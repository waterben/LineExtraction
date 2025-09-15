cmake_minimum_required(VERSION 3.0)

# Component libraries
add_subdirectory(utility)
add_subdirectory(lsd)
add_subdirectory(lfd)
add_subdirectory(edge)
add_subdirectory(imgproc)
add_subdirectory(geometry)
add_subdirectory(phase_cong)

IF (ENABLE_QT AND Qt5Widgets_FOUND)
    # moved to third-party/qplot and third-party/qplot3d
ENDIF (ENABLE_QT AND Qt5Widgets_FOUND)
