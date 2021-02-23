# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add Build Targets


IF (ENABLE_QT AND Qt5Widgets_FOUND)
add_subdirectory(line_analyzer)
ENDIF (ENABLE_QT AND Qt5Widgets_FOUND)

