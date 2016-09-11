TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += c++11


LIBS += -lgomp -lpthread
#QMAKE_CXXFLAGS += -fopenmp
#QMAKE_CXXFLAGS+= -fopenmp
QMAKE_LFLAGS +=  -fopenmp

SOURCES += main.cpp \
    APIThread.cpp \
    conboardsdktask.cpp \
    DJIHardDriverManifold.cpp \
    src/DJI_API.cpp \
    src/DJI_App.cpp \
    src/DJI_Camera.cpp \
    src/DJI_Codec.cpp \
    src/DJI_Flight.cpp \
    src/DJI_Follow.cpp \
    src/DJI_HardDriver.cpp \
    src/DJI_HotPoint.cpp \
    src/DJI_Link.cpp \
    src/DJI_Memory.cpp \
    src/DJI_Mission.cpp \
    src/DJI_VirtualRC.cpp \
    src/DJI_WayPoint.cpp \
    src/cmdCamera.cpp \
    src/cmdCoreAPI.cpp \
    src/cmdFlight.cpp \
    src/cmdFollow.cpp \
    src/cmdHotPoint.cpp \
    src/cmdIO.cpp \
    src/cmdSettings.cpp \
    src/cmdVirtualRC.cpp \
    src/cmdWayPoint.cpp \
    src/DJI_Interpreter.cpp \
    src/DJI_Script.cpp \
    maincontrol.cpp \
    Services/AprilTagDetector.cpp \
    Services/Function.cpp \
    src/Edge.cc \
    src/FloatImage.cc \
    src/Gaussian.cc \
    src/GLine2D.cc \
    src/GLineSegment2D.cc \
    src/GrayModel.cc \
    src/Homography33.cc \
    src/MathUtil.cc \
    src/Quad.cc \
    src/Segment.cc \
    src/TagDetection.cc \
    src/TagDetector.cc \
    src/TagFamily.cc \
    src/UnionFindSimple.cc \
    src/DJI_utility.cpp \
    AStar.cpp \
    BlockAllocator.cpp \
    Singleton.cpp

HEADERS += \
    APIThread.h \
    conboardsdktask.h \
    DJIHardDriverManifold.h \
    inc/DJI_API.h \
    inc/DJI_App.h \
    inc/DJI_Camera.h \
    inc/DJI_Codec.h \
    inc/DJI_Config.h \
    inc/DJI_Flight.h \
    inc/DJI_Follow.h \
    inc/DJI_HardDriver.h \
    inc/DJI_HotPoint.h \
    inc/DJI_Link.h \
    inc/DJI_Memory.h \
    inc/DJI_Mission.h \
    inc/DJI_Type.h \
    inc/DJI_Version.h \
    inc/DJI_VirtualRC.h \
    inc/DJI_WayPoint.h \
    inc/cmdCamera.h \
    inc/cmdCoreAPI.h \
    inc/cmdFlight.h \
    inc/cmdFollow.h \
    inc/cmdHotPoint.h \
    inc/cmdIO.h \
    inc/cmdSettings.h \
    inc/cmdVirtualRC.h \
    inc/cmdWayPoint.h \
    inc/DJI_Interpreter.h \
    inc/DJI_Script.h \
    maincontrol.h \
    inc/djicam.h \
    inc/Edge.h \
    inc/FloatImage.h \
    inc/Gaussian.h \
    inc/GLine2D.h \
    inc/GLineSegment2D.h \
    inc/GrayModel.h \
    inc/Gridder.h \
    inc/Homography33.h \
    inc/MathUtil.h \
    inc/pch.h \
    inc/Quad.h \
    inc/Segment.h \
    inc/Tag16h5_other.h \
    inc/Tag16h5.h \
    inc/Tag25h7.h \
    inc/Tag25h9.h \
    inc/Tag36h9.h \
    inc/Tag36h11_other.h \
    inc/Tag36h11.h \
    inc/TagDetection.h \
    inc/TagDetector.h \
    inc/TagFamily.h \
    inc/UnionFindSimple.h \
    inc/XYWeight.h \
    Services/AprilTagDetector.h \
    Services/Function.h \
    inc/NewTag/AllHelpers.h \
    inc/NewTag/AllOpenCVHelpers.h \
    inc/NewTag/AllStdHelpers.h \
    inc/NewTag/CameraHelper.h \
    inc/NewTag/ClusterHelper.h \
    inc/NewTag/config.hpp \
    inc/NewTag/CvMatHelper.h \
    inc/NewTag/DetectorHelper.h \
    inc/NewTag/DirHelper.h \
    inc/NewTag/FilterHelper.h \
    inc/NewTag/flycap2opencv.hpp \
    inc/NewTag/ImageHelper.h \
    inc/NewTag/IOHelper.h \
    inc/NewTag/LogHelper.h \
    inc/NewTag/OpenCVHeaders.h \
    inc/NewTag/PerformanceHelper.h \
    inc/NewTag/RotationHelper.h \
    inc/NewTag/SearchHelper.h \
    inc/NewTag/singleton.hpp \
    inc/NewTag/StringHelper.h \
    inc/NewTag/UtilHelper.h \
    inc/NewTag/VisHelper.h \
    inc/NewTag/cv2cgConfig.h \
    inc/NewTag/apriltag.hpp \
    inc/NewTag/Tag16h5.hpp \
    inc/NewTag/Tag25h7.hpp \
    inc/NewTag/Tag25h9.hpp \
    inc/NewTag/Tag36h9.hpp \
    inc/NewTag/Tag36h11.hpp \
    inc/NewTag/TagDetection.hpp \
    inc/NewTag/TagDetector.hpp \
    inc/NewTag/TagFamily.hpp \
    inc/NewTag/TagFamilyFactory.hpp \
    inc/NewTag/TagTypes.hpp \
    inc/NewTag/TagUtils.hpp \
    inc/DJI_utility.h \
    AStar.h \
    BlockAllocator.h \
    Singleton.h

#INCLUDEPATH += /home/ubuntu/demo/Qt/NewEagles/inc/djicam.h
LIBS += -L/home/ubuntu/demo/Qt/NewEagles -ldcam

INCLUDEPATH += /usr/include/eigen3/Eigen/

INCLUDEPATH += /usr/local/include/opencv \
/usr/local/include/opencv2 \
/usr/include \
./inc/NewTag \
/home/ubuntu/opencv-2.4.9/modules/core/include/opencv2

LIBS += /usr/local/lib/libopencv_core.so.2.4
LIBS += /usr/local/lib/libopencv_highgui.so.2.4
LIBS += /usr/local/lib/libopencv_imgproc.so.2.4
LIBS += /usr/local/lib/libopencv_objdetect.so.2.4
LIBS += /usr/local/lib/libopencv_photo.so.2.4
LIBS += /usr/local/lib/libopencv_video.so.2.4
LIBS += /usr/local/lib/libopencv_features2d.so.2.4
LIBS += /usr/local/lib/libopencv_objdetect.so.2.4
LIBS += /usr/local/lib/libopencv_calib3d.so.2.4
LIBS += /usr/local/lib/libopencv_flann.so.2.4

LIBS += /usr/local/lib/libopencv_ml.so.2.4
LIBS += /usr/local/lib/libopencv_gpu.so.2.4
LIBS += /usr/local/lib/libopencv_legacy.so.2.4
LIBS += /usr/local/lib/libopencv_contrib.so.2.4
LIBS += /usr/local/lib/libopencv_superres.so.2.4


LIBS     += -lusb-1.0 -lpthread
#LIBS += -lDJI_guidance
#LIBS += -L/home/ubuntu/3d/Guidance-SDK-master/so/XU3 -lDJI_guidance
LIBS += /home/ubuntu/Qt/NewEagles/libDJI_guidance.so



OTHER_FILES += \
    libdcam.so \
    inc/NewTag/CMakeLists.txt \
    libDJI_guidance.so


INCLUDEPATH += /home/ubuntu/Qt/NewEagles/inc



