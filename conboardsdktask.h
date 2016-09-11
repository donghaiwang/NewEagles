#ifndef CONBOARDSDKTASK_H
#define CONBOARDSDKTASK_H

#include <inc/DJI_Script.h>

#include <inc/cmdIO.h>
#include <inc/cmdSettings.h>
#include <inc/cmdCoreAPI.h>
#include <inc/cmdFlight.h>
#include <inc/cmdFollow.h>
#include <inc/cmdHotPoint.h>
#include <inc/cmdVirtualRC.h>
#include <inc/cmdWayPoint.h>
#include <inc/cmdCamera.h>

using namespace DJI::onboardSDK;

class ConboardSDKScript : public Script
{
  public:
    ConboardSDKScript(CoreAPI* api);

    TaskSetItem match(const char* name);
    void addTask(const char* Name, UserData Data = 0, time_t Timeout = 0);
};

class ScriptThread
{
  public:
    ScriptThread(ConboardSDKScript* Script = 0);

    void run();

  private:
    ConboardSDKScript* script;
};

#endif // CONBOARDSDKTASK_H
