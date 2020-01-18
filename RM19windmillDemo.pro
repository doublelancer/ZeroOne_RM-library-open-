TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp
#Libraries
unix: CONFIG += link_pkgconfig
LINKFLAGS += -fopenmp -pthread -fPIC $(COMMON_FLAGS) $(WARNINGS) -std=c++11
#OpenCV
unix: PKGCONFIG += opencv
DISTFILES += \
    LICENSE \
    README.md \
    SVM4_9.xml \
    SVM3.xml \
    SVM2.xml \
    SVM.xml \
    red.avi \
    red1.avi \
    robo.mp4 \
    RedCar.avi \
    robo.png \
    robo.jpg
