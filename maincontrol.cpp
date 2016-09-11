#include "maincontrol.h"
#include "Services/Function.h"

extern "C"{
#include "inc/djicam.h"
}

//AStarSearch
#include <chrono>
#include <iostream>
#include "AStar.h"


ConboardSDKScript* globalApi;

/* parameter */
#define TAKEOFF			(uint8_t) 4
#define LAND			(uint8_t) 6
#define WIDTH			320
#define HEIGHT			240
#define IMAGE_SIZE		(HEIGHT * WIDTH)
#define VBUS			e_vbus1
#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); printf( "error code:%d,%s %d\n", err_code, __FILE__, __LINE__ );}}

/* guidance */
#define VBUS			e_vbus1
int			err_code;
Mat     	g_greyscale_image_left=Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
Mat 		g_greyscale_image_right=Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
DJI_lock    guidance_lock;
DJI_lock    calculerate_lock;//by zhou0627
DJI_lock    X3image_lock;
DJI_lock    X3Buffer_lock;
DJI_lock    send_lock;
DJI_event   guidance_event;
DJI_event   X3Image_event;

// cells 20*20
#define CELL_ROWS 20
#define CELL_COLS 20
//struct cell		//
//{
//    double of_x; // x optical flow x
//    double of_y; // y optical flow y
//    uint count; //  number of keypoints
//    bool disc; //  discontinuity
//};
//cell of_cells[CELL_ROWS][CELL_COLS];

#define DISC_MARGIN (double) 0.5 //  a cell considered discontinous if the proportion of neighboring cells with different optical flow values exceeds this margin
#define MAX_NEIGH_DIST 1 //  max distance allowed between two cells to be considered neighbors
#define MAX_DISC (double) 0.7 //  max proportion of discontinuous cells allowed for drone to continue maneuvering

// Shi-Tomasi
#define MIN_CORNERS (uint) 15
#define CORNER_THRESHOLD (uint) 95
#define MAX_CORNERS (uint) 100
#define QUALITY_LEVEL (double) 0.01
#define MIN_DISTANCE (double) 1

// Lucas-Kanade
int frame_num = 0;

Mat prevleftimg, prevrightimg;
vector<Point2f> prevleftpts, prevrightpts, prevlefttracked, prevrighttracked;
#define OF_MARGIN (double) 0.5 // 2 optical flow values considered different if their difference exceeds this margin
bool l_kpt_regen = true;
bool r_kpt_regen = true;

// control
#define CMD_FLAG 0x4A // control horizontal/vertical velocity in body frame and yaw rate in ground frame
#define FWD (double) 0.5 // constant horizontal velocity
#define TURN (double) 10 // constant yaw rate
#define ALT (double) 0.01 // constant vertical velocity
double l_fwd = FWD; // left image forward control
double l_turn = 0; // left image turn control
double l_alt = 0; // left image altitude control
double r_fwd = FWD; // right image forward control
double r_turn = 0; // right image turn control
double r_alt = 0; // right image altitude control
double turn_prev = 0; // previous yaw for weighted camera observation

// control strategy
#define NONE 0 // no control
#define CTRL 1 // baseline control
#define PAUSE_CTRL 2 // stop moving at each obstacle and turn until obstacle is outside FOV
#define RATIO_CTRL 3 // multiply avoidance maneuver by ratio of discontinuities
int ctrl_strat = CTRL;
#define NO_ALT_CTRL

//float movedXY[40000][2];
double Location_X, Location_Y;
timeval StartTimeDW,CurrentTimeDW;
double time_intervalDW;
double Which_X,Which_Y;


// A* Search
#define mapsWidth 50
#define mapsHeight 50
#define RAD_TO_A 57.29577951308232
//int mapsWidth = 50;
//int mapsHeight = 50;
char maps[mapsWidth][mapsHeight];
//{
//    { 0, 1, 0, 0, 0, 1, 0, 0, 0, 0 },
//    { 0, 0, 0, 1, 0, 1, 0, 1, 0, 1 },
//    { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1 },
//    { 0, 0, 0, 1, 0, 0, 0, 1, 0, 1 },
//    { 0, 1, 0, 1, 1, 1, 1, 1, 0, 1 },
//    { 0, 1, 0, 0, 0, 0, 0, 0, 0, 1 },
//    { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
//    { 0, 0, 0, 0, 1, 0, 0, 0, 1, 0 },
//    { 1, 1, 0, 0, 1, 0, 1, 0, 0, 0 },
//    { 0, 0, 0, 0, 0, 0, 1, 0, 1, 0 },
//};
double distancePerCell = 1.0;       // the width of every Cell (the real world)
double camera_matrix[3][3]={247.357576, 0.000000, 153.295063, 0.000000, 247.390025, 116.893925, 0.000000, 0.000000, 1.000000};//camera_matrix[0][0] is f.
double K[3][3]={247.35757622530616, 0.0, 153.29506251287017, 0.0, 247.39002468328675, 116.8939252668646, 0.0, 0.0, 1.0};//camera calibration matrix
//double mx=K[0][0]/camera_matrix[0][0];
double mx = 1.012508986324;
double baseline = 0.149837;
double camera_cu = 159.475;
double camera_cv = 120.945;
double focal = 241.909;
//in image coordinate,origin is at the top left corner.
//double CoorInCam[240][320][3];
double CoorInWorld[240][320][3];
double CoorInCam[240][320][3];
double searchHeight = 0;

double origin_lati = 0;
double origin_longi = 0;

Vector2d searchAreaLeftDown;
Vector2d searchAreaRightUp;
Vector2d searchAreaLeftDownXY;
Vector2d searchAreaRightUpXY;

double thetaWithGrid;

FILE *fp;
FILE *fpFly;

struct  cell
{
    double x;
    double z;
    int cnt=0;
    char indicator = 0;//0,1

}grid[200][200];
double Nresolution = 1;             // 0.3
int quantity=ceil(mapsWidth/Nresolution);  // 167x167


double timeNow()
{
    struct timeval t;
    gettimeofday(&t,NULL);
    return ((double)t.tv_sec + ((double)t.tv_usec)/1000000);
}



static uint8_t* Double2Bytes(double v)
{
    int n = sizeof(v);
   uint8_t* result = new uint8_t[n];
   memcpy(result,&v,n);
   return result;
}

static void TagInfoPack2Bytes(TagInfoPack* v,uint8_t* result)
{
   memcpy(result,v,10);
//   return result;
}




MainControl::MainControl(ConboardSDKScript* api,HardDriverManifold* driver, CoreAPI* Vapi)
{
    mAPI = api;
    globalApi = api;
    mDriver = driver;
    mVapi = Vapi;

    mX3DataBuffer.X3Buffer = new unsigned char[FRAME_SIZE+8];
    ActiveSuc = false;
}

void MainControl::Initialize()
{
    while(!ActiveSuc)
    {
        Active();
        sleep(1);
    }

    API_LOG(mDriver, STATUS_LOG, "...succeed Active\n");
    while(!ControlSuc)
    {
         SetControl();
         sleep(1);
    }
    API_LOG(mDriver, STATUS_LOG, "...succeed Control\n");

    /* Guidance */
    reset_config();
    cout<<"reset_config is OK!"<<endl;
    init_transfer();
    select_depth_image( VBUS );
//    select_obstacle_distance();
    set_sdk_event_handler(MainControl::guidance_callback); // set guidance callback

//    CreateGetBufferThread();

    // init search area two point (altitude, lontitude)
    searchAreaLeftDown.x = 22.5429;
    searchAreaLeftDown.y = 113.9529;
    searchAreaRightUp.x = 22.5430;
    searchAreaRightUp.y = 113.95230;
    searchAreaLeftDownXY = Function::GPS_2_XYTrans(searchAreaLeftDown.x, searchAreaLeftDown.y, origin_lati, origin_longi);
    searchAreaRightUpXY = Function::GPS_2_XYTrans(searchAreaRightUp.x, searchAreaRightUp.y, origin_lati, origin_longi);
    searchAreaLeftDownXY.x = 0; searchAreaLeftDownXY.y = 0;       // temp use
    searchAreaRightUpXY.x = 70.71067811; searchAreaRightUpXY.y = 0; // temp use
    double alpha = asin((searchAreaRightUpXY.x - searchAreaLeftDownXY.x)
                        / sqrt( (searchAreaLeftDownXY.x-searchAreaRightUpXY.x)*(searchAreaLeftDownXY.x-searchAreaRightUpXY.x)
                                + (searchAreaRightUpXY.y-searchAreaLeftDownXY.y)*(searchAreaRightUpXY.y-searchAreaLeftDownXY.y)));
    thetaWithGrid = alpha - 45*PI/180;


    fp = fopen("/home/ubuntu/CallBackLog.data", "w+");
    fpFly = fopen("/home/ubuntu/FlyLog.data", "w+");
}

void MainControl::Run()
{
    cout<<"enter run\n";
//    CreateCalculateThread();//Location in room by zhou 0624
//    CreateX3ImageReadThread();//recognize picture by zhou 0622
    origin_lati = globalApi->getFlight()->getPosition().latitude;
    origin_longi = globalApi->getFlight()->getPosition().longitude;

    AvoidObstaclesTest();
}




void MainControl::TakeOffandLandingTest()
{
    while(mAPI->getApi()->getFlightStatus() == 1)
    {
         mAPI->getFlight()->task(Flight::TASK_TAKEOFF);
         sleep(1);
    }
    API_LOG(mDriver, STATUS_LOG, "...succeed take off\n");

    sleep(1);

    while(mAPI->getFlight()->getPosition().height <50)
    {
        FlightData data;
        data.flag = 147;
        data.x = 0;
        data.y = 0;
        data.z = 51;
        data.yaw = 0;
        mAPI->getFlight()->setFlight(&data);
        sleep(1);
    }
    API_LOG(mDriver, STATUS_LOG, "...succeed Flight\n");

    while(mAPI->getApi()->getFlightStatus() != 4)
    {
        mAPI->getFlight()->task(Flight::TASK_LANDING);
        sleep(1);
    }
    API_LOG(mDriver, STATUS_LOG, "...succeed Landing\n");
}


void MainControl::GetCameraBuffer()
{
    int ret;
    ret = manifold_cam_read(mX3DataBuffer.X3Buffer, &mX3DataBuffer.X3nFrame, CAM_BLOCK);
}

void MainControl::activationCallback(CoreAPI *This, Header *header, UserData userData)
{
    MainControl *sdk = (MainControl *)userData;
    volatile unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        if (ack_data == ACK_ACTIVE_NEW_DEVICE)
        {
        }
        else
        {
            if (ack_data == ACK_ACTIVE_SUCCESS)
            {
            }
            else
            {
            }
        }
    }
    else
    {
    }
    sdk->ActiveSuc = true;
    This->activateCallback(This, header);
}

void MainControl::Active()
{
    ActivateData data;
 //   data.version = mAPI->getSDKVersion();
    data.reserved = 2;
    data.ID = 1026652;
    string tmp = "b231439889d3f0e417fcbb887a32cd78770bb12014d1a80e98c745ea25303ddd";
   // char tmpchar[] = "c5a9482b8261a6327835f6eaac50f6aca09e0fe507322386ea4676ce32d119d5\0";
    data.encKey =(char*) tmp.data(); //! @warning memory leak fixme
    mAPI->getApi()->activate(&data, MainControl::activationCallback, this);
}

void MainControl::SetControl()
{
     mAPI->getApi()->setControl(true, MainControl::setControlCallback, this);
}

void MainControl::setControlCallback(CoreAPI *This, Header *header, UserData userData)
{
    MainControl *sdk = (MainControl *)userData;
    unsigned short ack_data = AC_COMMON_NO_RESPONSE;
    unsigned char data = 0x1;

    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               2);
    }
    else
    {
     //   API_LOG(sdk->driver, ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
       //         header->sessionID, header->sequenceNumber);
    }
 //sdk->ControlSuc = true;
    switch (ack_data)
    {
        case ACK_SETCONTROL_NEED_MODE_F:

            break;
        case ACK_SETCONTROL_RELEASE_SUCCESS:


            break;
        case ACK_SETCONTROL_OBTAIN_SUCCESS:
             sdk->ControlSuc = true;
            break;
        case ACK_SETCONTROL_OBTAIN_RUNNING:

        This->send(2, 1, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500, 2,
                   MainControl::setControlCallback, userData);
            break;
        case ACK_SETCONTROL_RELEASE_RUNNING:

            break;
    }
    //! @note For debug, all functional print is moving to this function,
    //! default API callback is not necessary.
   // CoreAPI::setControlCallback(This, header);
}

void MainControl::CreateGetBufferThread()
{

    int ret = -1;
    ret = pthread_create(&x3ProcessThread, NULL, GetCall, (void*)this);
}

void *MainControl::GetCall(void *param)
{
    int ret = -1;
    while(ret == -1)
    {
        ret =  manifold_cam_init(6);
    }
    API_LOG(((MainControl*)param)->mDriver, STATUS_LOG, "...succeed Get Camera\n");
    usleep(100);

    while(true)
    {
        ((MainControl*)param)->GetCameraBuffer();
        usleep(1000);
    }
}

void MainControl::CreateSendReadThread()
{
    int ret = -1;
    ret = pthread_create(&readSendThread, NULL, SendReadCall, (void*)this);
}

void *MainControl::SendReadCall(void *param)
{
    while(true)
    {
        //        ((MainControl*)param)->mAPI->getCamera()->setGimbalAngle(&(((MainControl*)param)->mAngleData));
        //        ((MainControl*)param)->mAPI->getFlight()->setFlight(&(((MainControl*)param)->mFlightData));
        ((MainControl*)param)->mAPI->getApi()->sendPoll();
        ((MainControl*)param)->mAPI->getApi()->readPoll();
        usleep(2000);
    }
}





void MainControl::CreateCalculateThread()
{
    int ret = -1;
    ret = pthread_create(&calculateThread, NULL, calLandingPoint, (void*)this);
}

void *MainControl::calLandingPoint(void *param)
{
    MainControl* self = (MainControl*)param;
    ConboardSDKScript* mAPI = self->mAPI;
    double movedX = 0.0, movedY = 0.0;
//    float Route_X=0.0, Route_Y=0.0;
    self->mCurX=0.0;
    self->mCurY=0.0;
    double theta = 3.1415926*13.0/180;
//    int cntIntegralStep = 0;
    while (1)
    {
        guidance_lock.enter();
        double xVel = mAPI->getFlight()->getVelocity().x;
        double yVel = mAPI->getFlight()->getVelocity().y;
        guidance_lock.leave();

        usleep(10000);

        movedX += xVel*0.01;
        movedY += yVel*0.01;

        self->mCurX = movedX*cos(theta) - movedY*sin(theta);
        self->mCurY = movedX*sin(theta) + movedY*cos(theta);

        calculerate_lock.enter();
        Location_X = self->mCurX;
        Location_Y = self->mCurY;
        calculerate_lock.leave();

//        movedXY[cntIntegralStep][0] = mMC->mCurX;
//        movedXY[cntIntegralStep][1] = mMC->mCurY;
//        cntIntegralStep++;
//        cout << cntIntegralStep << ": movedX: " << mMC->mCurX << ", movedY: " << mMC->mCurY << endl;
        cout<<"Location_X: "<<Location_X<<" Location_Y: "<<Location_Y<<endl;
    }
}

void MainControl::Release()
{
         while(!manifold_cam_exit()) /*make sure all threads exit ok*/
         {
           sleep(1);
         }
}

void MainControl::MoveInHorizontal(double distanceX, double distanceY, double time_in)
{
    timeval StartTimeUs,CurrentTimeUs;
    gettimeofday(&StartTimeUs,NULL);
    double time_interval;
//    double time_x=timex,Distance_x=distanceX, time_y = timey, Distance_y = distanceY;//international unit s,m
    bool my_flag = true;
    /*zwg 2016 using S_left And Time_left as a indicator of velocity*/
    double XS_left=distanceX;
    double YS_left=distanceY;

    double Time_left=time_in;

    double vx_Real_last=0;
    double vy_Real_last=0;
    double vx_Real_Cur=0;
    double vy_Real_Cur=0;
    double vx_set=0;
    double vy_set=0;

    double lastTimeInterval=0;
    double lastStartTime=0;

    while(my_flag)
    {
        gettimeofday(&CurrentTimeUs,NULL);   //currenttime=mAPI->getApi()->getTime().time---is not accurcy!
        time_interval = 1000.0*(CurrentTimeUs.tv_sec-StartTimeUs.tv_sec)+0.001*(CurrentTimeUs.tv_usec - StartTimeUs.tv_usec);
        lastTimeInterval =(time_interval-lastStartTime)*0.001;

        //get the end velocity of every step
        vx_Real_Cur=mAPI->getFlight()->getVelocity().x;
        vy_Real_Cur=mAPI->getFlight()->getVelocity().y;
        //get the progress of every step ,we assume AUV have fixed AccelerationSpeed during a short time
        Time_left-=lastTimeInterval;
        XS_left-=((vx_Real_Cur-vx_Real_last)*lastTimeInterval/2);
        YS_left-=((vy_Real_Cur-vy_Real_last)*lastTimeInterval/2);

         mFlightData.flag = 81;
         if(time_interval < time_in*1000 + 50)
         {

            if(Time_left<0.01)
            {
                vx_set= (1.57079633*XS_left*sin(3.141592653*0.001*time_interval/time_in))/0.01;
                vy_set=(1.57079633*YS_left*sin(3.141592653*0.001*time_interval/time_in))/0.01;
            }
            else
            {
                vx_set= (3.1415926*XS_left*sin(3.141592653*0.001*time_interval/time_in))/(time_in*(1+cos(3.1415926*time_interval/time_in)));
                vy_set= (3.1415926*YS_left*sin(3.141592653*0.001*time_interval/time_in))/(time_in*(1+cos(3.1415926*time_interval/time_in)));
            }
            mFlightData.x = vx_set;
            mFlightData.y = vy_set;
            mFlightData.z = mAPI->getFlight()->getPosition().height;
            mFlightData.yaw = 0;
            //get the start velocity of every step
             vx_Real_last=mAPI->getFlight()->getVelocity().x;
             vy_Real_last=mAPI->getFlight()->getVelocity().y;
         }
         else
         {
             mFlightData.x = 0;
             mFlightData.y = 0;
             mFlightData.z = mAPI->getFlight()->getPosition().height;
             mFlightData.yaw = 0;
             my_flag = false;
         }
        lastStartTime=time_interval;
        mAPI->getFlight()->setFlight(&mFlightData);
        usleep(10000);
    }
}


/* Maneuvering and avoiding obstacles */
void MainControl::AvoidObstaclesTest()
{
//    cout<<"init_transfer is OK!"<<endl;
//    select_greyscale_image( VBUS, true );
//    cout<<"select_greyscale_image_true is OK!"<<endl;
//    select_greyscale_image( VBUS, false );
//    cout<<"select_greyscale_image_false is OK!"<<endl;
//    stereo_cali cali[CAMERA_PAIR_NUM];                                                                      // cu           cv      focal       baseline
//    get_stereo_cali(cali);                                                                                  // 0            0       4.70198e-38	0
//    std::cout<<"cu\tcv\tfocal\tbaseline\n";                                                                 // 0            0       1.06449e+35	1.69672
//    for (int i=0; i<CAMERA_PAIR_NUM; i++)                                                                   // -0.0125075	1.99862	0.1         0
//    {                                                                                                       // 5.60519e-45	0       0           2
//        std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;    // 2.18795e-38	0       0           1.75865e-38
//    }
    /*
     * cu	cv      focal	baseline
    159.475	120.945	241.909	0.149837
    164.654	126.61	244.187	0.150067
    158.515	127.094	241.192	0.150194
    161.667	116.625	255.697	0.150184
    165.905	123.029	234.004	0.150134
    */

    while(mAPI->getApi()->getFlightStatus() == 1)
    {
         mAPI->getFlight()->task(Flight::TASK_TAKEOFF);
         sleep(1);
         cout<<"try takeoff"<<endl;
    }
    API_LOG(mDriver, STATUS_LOG, "...succeed take off\n");

    while(mAPI->getFlight()->getPosition().height < 1.5)
    {
        FlightData data;
        data.flag = 147;
        data.x = 0;
        data.y = 0;
        data.z = 1.6;
        data.yaw = 0;
        mAPI->getFlight()->setFlight(&data);
    }
    searchHeight = globalApi->getFlight()->getPosition().height;
    API_LOG(mDriver, STATUS_LOG, "...succeed Flight\n");
    cout<<"flight height is: " << searchHeight <<endl;

//    Turn(90);
//    Turn(-90);
//    Turn(-90);

    err_code = start_transfer(); // start guidance data collection
    RETURN_IF_ERR( err_code );

    sleep(2);   //Let Guidance stay in working for a period of time to construct obstacle maps
    double searchStart = Function::tic();
    AStarSearch();
    double searchEnd = Function::tic();
    cout << "A* search use " << searchEnd-searchStart << "s!!!" << endl;    // 475.691s (0,0)->(49,49)

    stop_transfer();
    release_transfer();

    while(mAPI->getApi()->getFlightStatus() != 4)   // Landing
    {
        mAPI->getFlight()->task(Flight::TASK_LANDING);
        sleep(1);
        cout<<"try landing!"<<endl;
    }
     API_LOG(mDriver, STATUS_LOG, "...succeed Landing\n");

    /*
     int ret = -1;
     while(ret == -1)
     {
         ret =  manifold_cam_exit();
     }
     cout<< "...succeed release Camera\n";
     API_LOG(mDriver, STATUS_LOG, "...succeed release Camera\n");
     */
}


void MainControl::Turn(double angle) {
    QuaternionData q = globalApi->getFlight()->getQuaternion();
    double initYaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
    initYaw *= RAD_TO_A;
//    if (abs(0 - (initYaw+thetaWithGrid*RAD_TO_A)) < 30) {
//        initYaw = 0;
//    } else if (abs(90 - (initYaw+thetaWithGrid*RAD_TO_A)) < 30) {
//        initYaw = 90;
//    } else if (abs(180 - (initYaw+thetaWithGrid*RAD_TO_A)) < 30) {
//        initYaw = 180;
//    } else if (abs(-180 - (initYaw+thetaWithGrid*RAD_TO_A)) < 30) {
//        initYaw = -180;
//    } else if (abs(-90 - (initYaw+thetaWithGrid*RAD_TO_A)) < 30) {
//        initYaw = -90;
//    }
//    initYaw -= thetaWithGrid*RAD_TO_A;

//    printf("Before Turn: initYaw: %lf\n", initYaw);

    double curYaw = initYaw;
    while (abs(curYaw - initYaw - angle) >= 0.1) {
        FlightData mFlightData;
        mFlightData.flag = 0x93;
        mFlightData.x = 0;
        mFlightData.y = 0;
        mFlightData.z = searchHeight;
        mFlightData.yaw = initYaw + angle;
        mAPI->getFlight()->setFlight(&mFlightData);

        QuaternionData q = globalApi->getFlight()->getQuaternion();
        curYaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
        curYaw *= RAD_TO_A;
    }
//    printf("After Turn: curYaw: %lf\n", curYaw);
}


void MainControl::MoveTo(double curX, double curY, double moveToX, double moveToY) {
    QuaternionData q = globalApi->getFlight()->getQuaternion();
    double curYaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
//    moveToX += 0.5; moveToY += 0.5; // move from grid line to grid center
//    double alpha = PI/2 - atan(moveToX / moveToY) - thetaWithGrid;

//    moveToX = sqrt(moveToX*moveToX + moveToY*moveToY) * cos(curYaw);
//    moveToY = sqrt(moveToX*moveToX + moveToY*moveToY) * sin(curYaw);


    double latitudeCur = globalApi->getFlight()->getPosition().latitude;
    double longitudeCur = globalApi->getFlight()->getPosition().longitude;
    Vector2d xy = Function::GPS_2_XYTrans(latitudeCur, longitudeCur, origin_lati, origin_longi);
    double curXInWorldX = xy.x;
    double curYInWorldY = xy.y;

//    cout << "CurrrentPosition: (" << priX << "," << priY << "),";
//    cout << "mvoeTo: (" << moveToX << "," << moveToY << ")" << endl;

//    double targetDistanceX = moveToX - priX;
//    double targetDistanceY = moveToY - priY;
    double targetDistanceX = sqrt((moveToX-curX)*(moveToX-curX)+(moveToY-curY)*(moveToY-curY)) * cos(curYaw);
    double targetDistanceY = sqrt((moveToX-curX)*(moveToX-curX)+(moveToY-curY)*(moveToY-curY)) * sin(curYaw);
    moveToX = curXInWorldX + targetDistanceX;
    moveToY = curYInWorldY + targetDistanceY;
    int reachDestination = 0;
    double reachTimeStart = 0;
    double reachTimeEnd = 0;


    while (reachTimeEnd-reachTimeStart < 1) {   // targetDistanceX > 0.05 || targetDistanceY > 0.05 ||

        latitudeCur = globalApi->getFlight()->getPosition().latitude;
        longitudeCur = globalApi->getFlight()->getPosition().longitude;
        xy = Function::GPS_2_XYTrans(latitudeCur, longitudeCur, origin_lati, origin_longi);
        double curX = xy.x;
        double curY = xy.y;

        targetDistanceX = moveToX  - curX;
        targetDistanceY = moveToY  - curY;

        if ((targetDistanceX < 0.05 && targetDistanceY < 0.05) && reachDestination == 0) {
            reachTimeStart = Function::tic();
            reachDestination++;
        }
        if (reachDestination > 0) {
            reachTimeEnd = Function::tic();
        }

        FlightData mFlightData;
        mFlightData.flag = 0x90;        // 0x91 -> 0x90
        mFlightData.x = targetDistanceX - 0.45*globalApi->getFlight()->getVelocity().x;
        mFlightData.y = targetDistanceY - 0.45*globalApi->getFlight()->getVelocity().y;
        mFlightData.z = searchHeight;
//        printf("In MoveTo, curYaw: %lf\n", curYaw*RAD_TO_A);
        mFlightData.yaw = curYaw*RAD_TO_A;
        mAPI->getFlight()->setFlight(&mFlightData);
    }
}


//http://blog.csdn.net/laogong5i0/article/details/8225429   A*
//http://www.cnblogs.com/Alip/p/5085515.html                draw grid
void MainControl::AStarSearch() {
    Turn(0-thetaWithGrid*RAD_TO_A);

    AStar::Vec2 pos[12] = {AStar::Vec2(0, 0), AStar::Vec2(49, 0),
                         AStar::Vec2(49, 9), AStar::Vec2(0, 9),
                         AStar::Vec2(0, 19), AStar::Vec2(49, 19),
                         AStar::Vec2(49, 29), AStar::Vec2(0, 29),
                         AStar::Vec2(0, 39), AStar::Vec2(49, 39),
                         AStar::Vec2(49, 49), AStar::Vec2(0, 49),};

    AStar::Param param; // Search parameter
    param.width = mapsWidth;
    param.height = mapsHeight;
    param.corner = false;   // consider 4 corner
    param.start = pos[0];
    param.end = pos[1];
    param.can_reach = [&](const AStar::Vec2 &pos)->bool
    {
        return maps[pos.x][pos.y] == 0;
    };

    AStar as;       // start search
    auto path = as.find(param);

    std::cout << "find path! " << "path length: " <<  path.size() << std::endl;
    for (int i = 0; i < path.size(); i++) {
        fprintf(fpFly, "(%u, %u), ", path[i].x, path[i].y);
    }
    fprintf(fpFly, "\n");

    double moveToX = 0, moveToY = 0;
    int preX = 0, preY = 0, curX = 0, curY = 0, laterX = 0, laterY = 0; // judge turn angle
    if (!path.empty()) {
        uint16_t targetX = path[0].x;
        uint16_t targetY = path[0].y;
        moveToX = targetX * distancePerCell;
        moveToY = targetY * distancePerCell;
        if (moveToY == 0) {
            if (moveToX > 0) {

            } else {
                Turn(90);
                Turn(90);
            }
        } else if (moveToX == 0) {
            if (moveToY > 0) {
                Turn(90);
            } else if (moveToY < 0) {
                Turn(-90);
            }
        }
        QuaternionData q = globalApi->getFlight()->getQuaternion();
        double curYaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
        fprintf(fpFly, "Before MoveTo, curYaw: %lf\n", curYaw*RAD_TO_A);
        MoveTo(curX, curX, moveToX, moveToY);
        q = globalApi->getFlight()->getQuaternion();
        curYaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
        fprintf(fpFly, "After MoveTo, curYaw: %lf\n", curYaw*RAD_TO_A);
        curX = moveToX; curY = moveToY;
    } else {
        cout << "can't find path!" << endl;
    }

    fprintf(fpFly, "*************************************************************************************************************");
    int targetIndex = 1;    // target destination(param.end) in pos array.  pos[targetIndex]
    fprintf(fpFly, "\n\n\n");
    while (moveToX != 0 || moveToY != mapsHeight-1) {      // do not reach (0, 49), continue.
        for (int i = 0; i < quantity; i++) {
            for (int j = 0; j < quantity; j++) {
                maps[i][j] = grid[i][j].indicator;
                printf("%d ", maps[i][j]);
            }
            printf("\n");
        }
        AStar::Param param; // Search parameter
        param.width = mapsWidth;
        param.height = mapsHeight;
        param.corner = false;   // consider 4 corner
        param.start = AStar::Vec2(moveToX, moveToY);
        param.end = pos[targetIndex];
        param.can_reach = [&](const AStar::Vec2 &pos)->bool
        {
            return maps[pos.x][pos.y] == 0;
        };

        AStar as;
        auto path = as.find(param);
        std::cout << "find path! " << "path length: " <<  path.size() << std::endl;
        for (int i = 0; i < path.size(); i++) {
            fprintf(fpFly, "(%u, %u), ", path[i].x, path[i].y);
        }
        fprintf(fpFly, "\n");
        if (!path.empty()) {
            uint16_t targetX = path[0].x;
            uint16_t targetY = path[0].y;

            // go straight, if the path in the later steps. Move 1 step, we use 1243.31 s(20 min) to cover the search area
            int steps = 0;
            if (curX == targetX) {  // fly along x axis
                for (int i = 1; i < path.size(); i++) {
                    if (curX == path[i].x && steps <= 0) {
                        steps++;
                    } else {
                        break;
                    }
                }
            } else {                // fly along y axis
                for (int i = 1; i < path.size(); i++) {
                    if (curY == path[i].y && steps <= 0) {
                        steps++;
                    } else {
                        break;
                    }
                }
            }
            targetX = path[steps].x; targetY = path[steps].y;


            moveToX = targetX * distancePerCell;
            moveToY = targetY * distancePerCell;

            // TODO: update preX, preY;
            laterX = targetX; laterY = targetY;
//            pfprintf(fpFly, "\n");rintf("(%d, %d), (%d, %d), (%d, %d)\n", preX, preY, curX, curY, laterX, laterY);
            if (preX == curX) {  // judge turn angle
                if (laterX == curX) {

                } else if (laterX > curX) {
                    if (curY > preY) {
                        Turn(-90);
                    } else {
                        Turn(90);
                    }
                } else  {
                    if (curY > preY) {
                        Turn(90);
                    } else {
                        Turn(-90);
                    }
                }
            } else if (preY == curY) {
                if (laterY == curY)  {

                } else if (laterY > curY) {
                    if (curX > preX) {
                        Turn(90);
                    } else {
                        Turn(-90);
                    }
                } else {
                    if (curX > preX) {
                        Turn(-90);
                    } else {
                        Turn(90);
                    }
                }
            }

            QuaternionData q = globalApi->getFlight()->getQuaternion();
            double curYaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
            fprintf(fpFly, "Before MoveTo, curYaw: %lf\n", curYaw*RAD_TO_A);
//            sleep(2);
            MoveTo(curX, curY, moveToX, moveToY);
            q = globalApi->getFlight()->getQuaternion();
            curYaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
            fprintf(fpFly, "After MoveTo, curYaw: %lf\n", curYaw*RAD_TO_A);

            preX = curX; preY = curY;
            curX = targetX; curY = targetY;

            if (moveToX == pos[targetIndex].x && moveToY == pos[targetIndex].y) {     // change to next destination
                targetIndex++;
            }
        } else {
            cout << "can't find path!" << endl;
            break;  // ?
        }

        fprintf(fpFly, "*************************************************************************************************************");
        fprintf(fpFly, "\n\n\n");

    }

}

int MainControl::guidance_callback(int data_type, int data_len, char *content) {
    guidance_lock.enter();

    if (e_image == data_type && NULL != content)
    {
        image_data* data = (image_data* )content;
        Mat		g_depth;

        if ( data->m_depth_image[e_vbus1] ){
            g_depth = Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
            memcpy( g_depth.data, data->m_depth_image[e_vbus1], IMAGE_SIZE * 2 );
            double dep[g_depth.rows][g_depth.cols];
            double depFiltered[g_depth.rows][g_depth.cols];
//            FILE *fp = fopen("/home/ubuntu/image.data", "w");
//            printf("rows: %d, cols: ", g_depth.rows, g_depth.cols);
            for (int i = 0; i < quantity; i++) {
                for (int j = 0; j < quantity; j++) {
                    grid[i][j].cnt = 0;
                }
            }

            fprintf(fp, "*************************************************************************************************************");
            fprintf(fp, "*************************************************************************************************************");
            for (int r = 0; r < g_depth.rows; r++)
            {
//                printf("\n");
                for (int c = 0; c < g_depth.cols; c++)
                {
                    ushort data = g_depth.at<ushort>(r,c);
                    ushort intPartAnd = 65408;
                    ushort inte = (data & intPartAnd) / 128;       // 11111111 10000000 (2^16-2^7 = 65408)
                    ushort decPartAnd = 127;                       // 00000000 01111111
                    ushort Tmp = data & decPartAnd;
                    double dec = (double)Tmp / 128.0;
                    double res = inte + dec;
                    dep[r][c] = res;
                    if (dep[r][c] >= 1 && dep[r][c] <= 10) {      // reserve the distance between 0.2m and 20m
                        depFiltered[r][c] = res;
                    } else {
                        depFiltered[r][c] = 0;
                    }

                    if (depFiltered[r][c] > 0.00000001) {
                        CoorInCam[r][c][0] = depFiltered[r][c] * (c-camera_cu) / focal;
                        CoorInCam[r][c][1] = depFiltered[r][c] * (camera_cv-r) / focal;
                        CoorInCam[r][c][2] = depFiltered[r][c];
//                        if (r == 120) {
//                            printf("%lf    %lf    %lf\n", CoorInCam[r][c][0], CoorInCam[r][c][1], CoorInCam[r][c][2]);
//                        }

                        if (r == 120) {        // CoorInCam[r][c][1] > -0.15 && CoorInCam[r][c][1] < 0.15 select a layer of depth
                            fprintf(fp, "---------------------------------------------------------------------------------");
                            fprintf(fp, "(row, column):(%d, %d), ", r, c);
                            fprintf(fp, "depFiltered[r][c]: %.4f,    ", depFiltered[r][c]);
                            fprintf(fp, "CoorInCam: %lf    %lf    %lf,     \n", CoorInCam[r][c][0], CoorInCam[r][c][1], CoorInCam[r][c][2]);
                            double latitudeCur = globalApi->getFlight()->getPosition().latitude;
                            double longitudeCur = globalApi->getFlight()->getPosition().longitude;
                            Vector2d xy = Function::GPS_2_XYTrans(latitudeCur, longitudeCur, origin_lati, origin_longi);
                            double curX = xy.x;
                            double curY = xy.y;
                            double curZ = searchHeight;
    //                        globalApi->getFollow()->getData().yaw;
//                            double curYaw = globalApi->getFlight()->YAW_ANGLE;
                            QuaternionData q = globalApi->getFlight()->getQuaternion();
                            double curYaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
                            fprintf(fp, "curYaw: %lf        ", curYaw);

                            double curXInWorldX= (CoorInCam[r][c][0])*sin(curYaw)+(CoorInCam[r][c][2]+0.09)*cos(curYaw) + curX;
                            double curYInWorldY = (CoorInCam[r][c][0])*cos(curYaw) -(CoorInCam[r][c][2]+0.09)*sin(curYaw) + curY;
                            double curZInWorldZ = CoorInCam[r][c][1] + 0.1 + curZ;
                            fprintf(fp, "curInWorld: (%lf, %lf, %lf)    \n", curXInWorldX, curYInWorldY, curZInWorldZ);

                            double beta = asin((curXInWorldX-searchAreaLeftDownXY.x)
                                               / sqrt( (curXInWorldX-searchAreaLeftDownXY.x)*(curXInWorldX-searchAreaLeftDownXY.x)
                                                       + (curYInWorldY-searchAreaLeftDownXY.y)*(curYInWorldY-searchAreaLeftDownXY.y) ));
                            if (curYInWorldY < searchAreaLeftDownXY.y) {
                                beta = PI - beta;
//                                printf("beta: %lf\n", beta);
                            }

//                            double theta = beta-alpha+(45*PI/180);
                            double theta = beta-thetaWithGrid;
//                            printf("beta: %lf\n", beta);

                            if (theta >= 0 && theta <= 90*PI/180) { // remove the point outside the grid
                                fprintf(fp, "theta: %lf\n", theta);
                                int n = floor(sqrt( (curXInWorldX-searchAreaLeftDownXY.x)*(curXInWorldX-searchAreaLeftDownXY.x)
                                                    + (curYInWorldY-searchAreaRightUpXY.y)*(curYInWorldY-searchAreaRightUpXY.y) )
                                                    * cos(theta) / 1.0);
                                int m = floor(sqrt( (curXInWorldX-searchAreaLeftDownXY.x)*(curXInWorldX-searchAreaLeftDownXY.x)
                                                    + (curYInWorldY-searchAreaRightUpXY.y)*(curYInWorldY-searchAreaRightUpXY.y) )
                                                    * sin(theta) / 1.0);
                                fprintf(fp, "curInWorld: (%lf, %lf)     ", curXInWorldX, curYInWorldY);
    //                            int m = floor(curXInWorldX / 1.0);
    //                            int n = floor(curYInWorldY / 1.0);
                                fprintf(fp, "m = %d,    n = %d\n", m, n);
                                if (m >= 0 && m < quantity && n >= 0 && n < quantity) {
                                    grid[m][n].cnt = grid[m][n].cnt + 1;
                                }

                            }

                            fprintf(fp, "---------------------------------------------------------------------------------\n\n\n");

                        }
                    }

                }
//                if (r == 120) {
//                    for (int c = 0; c < g_depth.cols; c++) {
//                        fprintf(fp, "%.4lf    ", depFiltered[r][c]);
//                        printf("%.4lf    ", depFiltered[r][c]);
//                    }
//                }
//                fprintf(fp, "\n");
            }
//            fprintf(fp, "\n");

            fprintf(fp, "\n");
            for (int m = 0; m < quantity; m++) {
                for (int n = 0; n < quantity; n++) {
                    if (grid[m][n].cnt >= 5) {  //label one cell as obstacle when it includes more than 5 obstacle points.
                        grid[m][n].indicator = 1;
                        fprintf(fp, "(%d, %d), cnt = %d   \n", m, n, grid[m][n].cnt);
                    }

                }
                fprintf(fp, "\n");
            }



            fprintf(fp, "*************************************************************************************************************");
            fprintf(fp, "*************************************************************************************************************");

        } else g_depth.release();
    }


    guidance_lock.leave();
    guidance_event.set_DJI_event();
    return 0;
}
