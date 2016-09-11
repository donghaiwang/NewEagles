#ifndef CONBOARDSDK_H
#define CONBOARDSDK_H

#include <inc/DJI_HardDriver.h>
#include <inc/DJI_Camera.h>
#include <inc/DJI_Flight.h>
#include <inc/DJI_HotPoint.h>
#include <inc/DJI_Follow.h>
#include <inc/DJI_WayPoint.h>
#include <inc/DJI_VirtualRC.h>
#include <inc/DJI_API.h>
#include <pthread.h>
#include <string>

using namespace DJI::onboardSDK;

class APIThread
{
  public:
    APIThread();
    APIThread(CoreAPI *API, int Type);

    bool createThread();

  private:
    CoreAPI *api;
    int type;

    static void *send_call(void *param);
    static void *read_call(void *param);
};

#endif // CONBOARDSDK_H
