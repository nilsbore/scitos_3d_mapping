#-------------------------------------------------
#
# Project created by QtCreator 2015-02-22T13:13:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TARGET = QTLabeller
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

unix:!macx: LIBS += -L$$PWD/../../../devel/lib/ -lboost_system

INCLUDEPATH += $$PWD/../../strands_3d_mapping/metaroom_xml_parser/include
DEPENDPATH += $$PWD/../../strands_3d_mapping/metaroom_xml_parser/include

DISTFILES += \
    display_labels.txt \
    labels.txt
