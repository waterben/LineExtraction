#
#  QCustomPlot Plot Examples
#

QT       += core gui widgets printsupport

TARGET = plot-examples
TEMPLATE = app

SOURCES += main.cpp\
           mainwindow.cpp \
         ../../qcustomplot.cpp

HEADERS  += mainwindow.h \
         ../../qcustomplot.h

FORMS    += mainwindow.ui
