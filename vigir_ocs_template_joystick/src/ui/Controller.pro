#-------------------------------------------------
#
# Project created by QtCreator 2014-05-20T11:10:43
#
#-------------------------------------------------

QT       += core gui\
        opengl

TARGET = Controller
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    GlWidget.cpp

HEADERS  += mainwindow.h \
    GlWidget.h

FORMS    += mainwindow.ui

RESOURCES += \
    up.qrc \
    down.qrc \
    left.qrc \
    right.qrc

OTHER_FILES += \
    fragmentShader.frag \
    vertexShader.vert
