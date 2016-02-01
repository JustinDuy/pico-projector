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
    myqwidget.cpp

HEADERS  += \
    myqwidget.h

INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib \
-lopencv_core \
-lopencv_imgproc \
-lopencv_highgui \
-lopencv_ml \
-lopencv_video \
-lopencv_features2d \
-lopencv_calib3d \
-lopencv_objdetect \
-lopencv_flann

FORMS    +=
