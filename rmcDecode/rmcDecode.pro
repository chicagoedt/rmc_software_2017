TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    rmcData.cpp \
    rmcEnDecoder.cpp

HEADERS += \
    rmcData.h \
    rmcEnDecoder.h
