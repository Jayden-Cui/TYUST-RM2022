TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        Info_slover/angle_slover.cpp \
        armor_detect/armor_detect.cpp \
        cam_driver/open_camera.cpp \
        main/main.cpp \
        main/visual_proc.cpp \
    predict/kala.cpp \
        predict/predict.cpp \
        usb_serial/serial_usb.cpp \
    usb_serial/CRC_Check.cpp

HEADERS += \
    Info_slover/angle_solver.h \
    armor_detect/armor_detect.h \
    cam_driver/DVPCamera.h \
    cam_driver/open_camera.h \
    main/visual_proc.h \
    predict/kala.h \
    predict/predict.h \
    shoot_top/shoot_top.h \
    usb_serial/serial_usb.h \
    usb_serial/CRC_Check.h

LIBS += /usr/local/lib/libopencv_*.so \
        /usr/local/lib/libhzd.so \
        /usr/local/lib/libdvp.so \
       /usr/local/lib/usb3_v_all.dscam.so
LIBS +=-lpthread

SUBDIRS += \
    Auto-shoot.pro
