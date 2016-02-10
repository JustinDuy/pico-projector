#-------------------------------------------------
#
# Project created by QtCreator 2016-01-27T13:41:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pico-projector
TEMPLATE = app


SOURCES += main.cpp\
    myqwidget.cpp \
    utility.cpp

HEADERS  += \
    myqwidget.h \
    utility.h

INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /usr/local/include/opencv

LIBS += `pkg-config opencv --libs`

FORMS    +=
