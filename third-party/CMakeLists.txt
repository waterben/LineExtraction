cmake_minimum_required(VERSION 3.5...3.27)

# Disable warnings for third-party libraries
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# Third-party libraries
if(ENABLE_QT AND Qt5Widgets_FOUND)
    add_subdirectory(qplot)
    add_subdirectory(qplot3d)
endif()
