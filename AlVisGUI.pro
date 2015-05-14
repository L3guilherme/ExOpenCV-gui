#-------------------------------------------------
#
# Project created by QtCreator 2014-09-02T16:50:02
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AlVisGUI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    openglp.cpp

HEADERS  += mainwindow.h \
    openglp.h

FORMS    += mainwindow.ui

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_calib3d

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_contrib

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_core

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_features2d

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_highgui

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_imgproc

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_objdetect

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_ocl

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

QMAKE_CXXFLAGS += -std=c++11

unix:!macx: LIBS += -L$$PWD/../../../../../opt/AMDAPP/lib/x86_64/ -lglut

INCLUDEPATH += $$PWD/../../../../../opt/AMDAPP/include
DEPENDPATH += $$PWD/../../../../../opt/AMDAPP/include

INCLUDEPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/ -lGL

INCLUDEPATH += $$PWD/../../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/ -lGLU

INCLUDEPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/AMDAPP/lib/x86_64/ -lGLEW

INCLUDEPATH += $$PWD/../../../../../../opt/AMDAPP/include
DEPENDPATH += $$PWD/../../../../../../opt/AMDAPP/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/AMDAPP/lib/x86_64/ -lamdocl64

INCLUDEPATH += $$PWD/../../../../../../opt/AMDAPP/include
DEPENDPATH += $$PWD/../../../../../../opt/AMDAPP/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/AMDAPP/lib/x86_64/ -lOpenCL

INCLUDEPATH += $$PWD/../../../../../../opt/AMDAPP/lib/x86_64
DEPENDPATH += $$PWD/../../../../../../opt/AMDAPP/lib/x86_64

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_video

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_ml

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_nonfree

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_flann

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/ -lm3api

INCLUDEPATH += $$PWD/../../../../../usr/include/m3api
DEPENDPATH += $$PWD/../../../../../usr/include/m3api

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/ -lzbar

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

