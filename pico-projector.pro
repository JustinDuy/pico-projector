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
    utility.cpp \
    graycodedecoder.cpp
HEADERS  += \
    utility.h \
    graycodedecoder.h

#CV_LIB_NAMES = core imgproc highgui calib3d
#for(lib, CV_LIB_NAMES) {
#    CV_LIBS += -lopencv_$$lib
#}


INCLUDEPATH += /home/duy/opencv-3.1.0/release/include
#LIBS += -L/home/duy/opencv-3/release/lib $$CV_LIBS
LIBS += -L/home/duy/opencv-3.1.0/release/lib `pkg-config opencv --libs`
FORMS    +=
