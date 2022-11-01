QT -= gui

CONFIG += c++17 console
CONFIG -= app_bundle

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        detector.cpp \
        main.cpp

INCLUDEPATH += D:\opencv\build\install\include

LIBS += D:\opencv\build\bin\libopencv_core454.dll
LIBS += D:\opencv\build\bin\libopencv_highgui454.dll
LIBS += D:\opencv\build\bin\libopencv_imgcodecs454.dll
LIBS += D:\opencv\build\bin\libopencv_imgproc454.dll
LIBS += D:\opencv\build\bin\libopencv_features2d454.dll
LIBS += D:\opencv\build\bin\libopencv_calib3d454.dll
LIBS += D:\opencv\build\bin\libopencv_video454.dll
LIBS += D:\opencv\build\bin\libopencv_videoio454.dll

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    detector.h
