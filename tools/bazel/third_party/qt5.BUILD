"""Qt5 GUI framework for C++

This module provides Qt5 libraries from the system installation.

Prerequisites:
  sudo apt-get install qtbase5-dev qtbase5-dev-tools
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

# Common include paths for all Qt5 modules
QT5_INCLUDES = [
    "include/x86_64-linux-gnu/qt5",
    "include/x86_64-linux-gnu/qt5/QtCore",
    "include/x86_64-linux-gnu/qt5/QtGui",
    "include/x86_64-linux-gnu/qt5/QtWidgets",
    "include/x86_64-linux-gnu/qt5/QtOpenGL",
    "include/x86_64-linux-gnu/qt5/QtPrintSupport",
]

# Qt5 Core
cc_library(
    name = "qt5_core",
    includes = QT5_INCLUDES,
    linkopts = [
        "-lQt5Core",
    ],
)

# Qt5 Gui
cc_library(
    name = "qt5_gui",
    includes = QT5_INCLUDES,
    linkopts = [
        "-lQt5Gui",
    ],
    deps = [":qt5_core"],
)

# Qt5 Widgets
cc_library(
    name = "qt5_widgets",
    includes = QT5_INCLUDES,
    linkopts = [
        "-lQt5Widgets",
    ],
    deps = [
        ":qt5_core",
        ":qt5_gui",
    ],
)

# Qt5 OpenGL
cc_library(
    name = "qt5_opengl",
    includes = QT5_INCLUDES,
    linkopts = [
        "-lQt5OpenGL",
        "-lGL",
    ],
    deps = [
        ":qt5_core",
        ":qt5_gui",
        ":qt5_widgets",
    ],
)

# Qt5 PrintSupport
cc_library(
    name = "qt5_printsupport",
    includes = QT5_INCLUDES,
    linkopts = [
        "-lQt5PrintSupport",
    ],
    deps = [
        ":qt5_core",
        ":qt5_gui",
        ":qt5_widgets",
    ],
)
