#-------------------------------------------------
#
# Project created by QtCreator 2011-11-08T19:07:34
#
#-------------------------------------------------

QT       += core gui network widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = LightCrafterGUI
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    PacketHandler.cpp \
    MBMCReadmeTxtParser.cpp \
    bmpparser.cpp

HEADERS  += mainwindow.h \
    PacketHandler.h \
    Helper.h \
    Version.h \
    MBMCReadmeTxtParser.h \
    bmpparser.h

FORMS    += mainwindow.ui

#-------------------------------------------------
# Icons
macx{
ICON = LightCrafterGUI.icns
}

win32{
RC_FILE = LightCrafterGUI.rc
}

linux-g++{
#QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/lib
}

OTHER_FILES += \
    LightCrafterGUI.rc \
    LightCrafterGUI.ico
