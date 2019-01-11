# Copyright (c) 2014 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add Build Targets


IF (USE_QT5 AND Qt5Widgets_FOUND)
add_subdirectory(line_analyzer)
ENDIF (USE_QT5 AND Qt5Widgets_FOUND)

