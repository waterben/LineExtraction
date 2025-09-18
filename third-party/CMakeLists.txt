cmake_minimum_required(VERSION 3.5...3.27)

# Third-party libraries
if(ENABLE_QT AND Qt5Widgets_FOUND)
    add_subdirectory(qplot)
    add_subdirectory(qplot3d)
endif()