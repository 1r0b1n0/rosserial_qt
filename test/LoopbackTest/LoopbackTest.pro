QT += core
QT -= gui
QT += network

CONFIG += c++11

TARGET = LoopbackTest
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += ../../ros_lib

SOURCES += main.cpp \
    ../../ros_lib/duration.cpp \
    ../../ros_lib/time.cpp \
    ../../ros_lib/RosQtSocket.cpp \
    ../../ros_lib/node_handle.cpp \
    node.cpp

DEFINES += QT_DEPRECATED_WARNINGS

HEADERS += \
    ../../ros_lib/ros/duration.h \
    ../../ros_lib/ros/msg.h \
    ../../ros_lib/ros/node_handle.h \
    ../../ros_lib/ros/publisher.h \
    ../../ros_lib/ros/ros.h \
    ../../ros_lib/ros/service_client.h \
    ../../ros_lib/ros/service_server.h \
    ../../ros_lib/ros/subscriber.h \
    ../../ros_lib/ros/time.h \
    ../../ros_lib/ros/RosQtSocket.h \
    node.h
