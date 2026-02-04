cmake_minimum_required(VERSION 3.5...3.27)

# Disable warnings for third-party libraries
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# Add contrib library (external/third-party code with different licenses)
add_subdirectory(contrib)

# Third-party libraries
if(ENABLE_QT AND Qt5Widgets_FOUND)
    add_subdirectory(qplot)
    add_subdirectory(qplot3d)
endif()
