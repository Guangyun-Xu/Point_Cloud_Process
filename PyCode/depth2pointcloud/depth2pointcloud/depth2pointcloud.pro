QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp \

INCLUDEPATH+=/usr/local/include/pcl-1.9\
/usr/include/eigen3\
/usr/local/include/vtk-7.1\
/usr/include/flann\
/usr/local/include/opencv

LIBS+=/usr/local/lib/libpcl_*.so\
-lboost_system\
-lrt\
/usr/local/lib/libvtk*.so\
/usr/local/lib/libopencv_*
