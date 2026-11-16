QT += core gui network
CONFIG += console c++17
CONFIG -= app_bundle

TARGET = radarsimulator

SOURCES += \
    radarsimulator.cpp \
    pointcloudprotocol.cpp

HEADERS += \
    pointcloudprotocol.h
