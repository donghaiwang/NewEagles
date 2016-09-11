#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"

#ifndef MAINCONTROL_H
#define MAINCONTROL_H

/* Guidance */
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/opencv.hpp"
#include "inc/DJI_guidance.h"
#include "inc/DJI_utility.h"
//#include "inc/imagetransfer.h"
//#include "inc/usb_transfer.h"

#include <stdio.h>
#include "string.h"
#include "unistd.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "Services/AprilTagDetector.h"


//using namespace cv;
using namespace std;
//#pragma pack 1
struct TagInfoPack
{
public:
    int X;  //4
    int Y;  //4
    uint8_t Type;//1
    uint8_t ID;  //1
};

struct RoutePack
{
public:
    int X;
    int Y;
    uint8_t Type;
};

class X3DataBuffer{
public:
    unsigned char* X3Buffer;//[FRAME_SIZE+8] = {0};//图像数据接口
    unsigned int X3nFrame = 0;
    int mode = 0;
};


class MainControl
{
public:
    MainControl(ConboardSDKScript* api, HardDriverManifold* driver, CoreAPI* Vapi);

    void Initialize();    
    void Release();

    void Active();
    static void activationCallback(CoreAPI *This, Header *header, UserData userData);
    void SetControl();
    static void setControlCallback(CoreAPI *This, Header *header, UserData userData);

    void Run();
    void TagDetectandAutoLandingTest();
    void TakeOffandLandingTest();
    void TagTrackingTest();
    void GetCameraBuffer();

    void AvoidObstaclesTest();
    void Turn(double angle);
//    void TurnLeft();
//    void TurnRight();
    void MoveTo(double curX, double curY, double moveToX, double moveToY);
    void AStarSearch();
    static int guidance_callback(int data_type, int data_len, char *content);

    void CreateGetBufferThread();
    void CreateSendReadThread();
    void CreateX3ImageReadThread();
    void CreateCalculateThread();//by zhou 0624

    static void *GetCall(void *param);
    static void *SendReadCall(void *param);
    static void *X3ImageRead(void *param);//by zhou 0622
    static void *calLandingPoint(void *param);

    void MoveInHorizontal(double distanceX, double distanceY, double time_in);

public:
    FlightData mFlightData;
    ConboardSDKScript* mAPI;
    HardDriverManifold* mDriver;
    CoreAPI* mVapi;

    X3DataBuffer mX3DataBuffer;
    AprilTagDetector mTagDetector;
    bool ActiveSuc;
    bool ControlSuc;

    double mCurX;
    double mCurY;
//    float movedXY[200000][2];//by zhou 0624
//    timeval StartTimeDW,CurrentTimeDW;
//    double time_intervalDW,Which_X,Which_Y;

    static void DoSomething(char *content, int data_type);

    pthread_t x3ReadThread,x3ProcessThread,readSendThread,calculateThread;
};

#endif // MAINCONTROL_H
