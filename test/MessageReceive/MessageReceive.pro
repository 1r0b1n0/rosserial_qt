QT += core
QT -= gui
QT += network

CONFIG += c++11

TARGET = MessageReceive
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += ../../src/ros_lib
INCLUDEPATH += ../ros_lib

SOURCES += main.cpp \
    ../../src/ros_lib/duration.cpp \
    ../../src/ros_lib/time.cpp \
    ../../src/ros_lib/RosQtSocket.cpp \
    node.cpp
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    ../../src/ros_lib/ros/duration.h \
    ../../src/ros_lib/ros/msg.h \
    ../../src/ros_lib/ros/node_handle.h \
    ../../src/ros_lib/ros/publisher.h \
    ../../src/ros_lib/ros/ros.h \
    ../../src/ros_lib/ros/service_client.h \
    ../../src/ros_lib/ros/service_server.h \
    ../../src/ros_lib/ros/subscriber.h \
    ../../src/ros_lib/ros/time.h \
    ../../src/ros_lib/ros/RosQtSocket.h \
    node.h
