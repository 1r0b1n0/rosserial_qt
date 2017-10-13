QT += core
QT -= gui
QT += network

CONFIG += c++11

TARGET = Basic
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

DEFINES += QT_DEPRECATED_WARNINGS

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
