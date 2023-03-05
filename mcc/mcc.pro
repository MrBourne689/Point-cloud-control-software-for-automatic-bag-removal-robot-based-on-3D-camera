QT       += core gui
QT       += serialbus serialport widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = mcc
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    xiancheng.cpp

HEADERS += \
    mainwindow.h \
    xiancheng.h

FORMS += \
    mainwindow.ui

include(D:/Projects/test/pcl.pri)

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


win32: LIBS += -L$$PWD/lib/win32/ -lPhoXi_API_msvc14_Release_1.4.1

INCLUDEPATH += $$PWD/lib/win32
DEPENDPATH += $$PWD/lib/win32

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/lib/win32/PhoXi_API_msvc14_Release_1.4.1.lib
else:win32-g++: PRE_TARGETDEPS += $$PWD/lib/win32/libPhoXi_API_msvc14_Release_1.4.1.a



DISTFILES +=

RESOURCES += \
    photo.qrc



