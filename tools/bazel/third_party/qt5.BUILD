"""Qt5 GUI framework for C++

This module provides Qt5 libraries from the system installation.

Prerequisites:
  sudo apt-get install qtbase5-dev qtbase5-dev-tools libqt5opengl5-dev
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

# System include paths for Qt5 (used with -isystem to suppress warnings)
QT5_COPTS = [
    "-isystem/usr/include/x86_64-linux-gnu/qt5",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtCore",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtGui",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtWidgets",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtOpenGL",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtPrintSupport",
    "-fPIC",
]

# Qt5 Core
cc_library(
    name = "qt5_core",
    copts = QT5_COPTS,
    linkopts = ["-lQt5Core"],
)

# Qt5 Gui
cc_library(
    name = "qt5_gui",
    copts = QT5_COPTS,
    linkopts = ["-lQt5Gui"],
    deps = [":qt5_core"],
)

# Qt5 Widgets
cc_library(
    name = "qt5_widgets",
    copts = QT5_COPTS,
    linkopts = ["-lQt5Widgets"],
    deps = [
        ":qt5_core",
        ":qt5_gui",
    ],
)

# Qt5 OpenGL
cc_library(
    name = "qt5_opengl",
    copts = QT5_COPTS,
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
    copts = QT5_COPTS,
    linkopts = ["-lQt5PrintSupport"],
    deps = [
        ":qt5_core",
        ":qt5_gui",
        ":qt5_widgets",
    ],
)
