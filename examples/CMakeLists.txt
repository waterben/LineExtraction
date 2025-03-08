cmake_minimum_required(VERSION 3.0)

add_subdirectory(lsd)
add_subdirectory(lfd)
add_subdirectory(other)
add_subdirectory(imgproc)
add_subdirectory(edge)
add_subdirectory(geometry)
add_subdirectory(thesis)

if (ENABLE_QT AND Qt5Widgets_FOUND)
    add_subdirectory(qt)
endif()
