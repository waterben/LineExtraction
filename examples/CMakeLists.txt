cmake_minimum_required(VERSION 2.6)

add_subdirectory(lsd)
add_subdirectory(lfd)
add_subdirectory(other)
add_subdirectory(imgproc)
add_subdirectory(edge)
add_subdirectory(geometry)

if (ENABLE_QT AND Qt5Widgets_FOUND)
    add_subdirectory(qt)
endif()
