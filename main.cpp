#include <iostream>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "maincontrol.h"

using namespace std;


int main(int argc, char *argv[])
{

    //! @note replace these two lines below to change to an other hard-driver level.
    HardDriverManifold driver("/dev/ttyTHS1", 9600);
//    driver.setBaudrate(230400);
    driver.setBaudrate(115200);
    driver.setDevice("/dev/ttyTHS1");
    driver.init();


    CoreAPI api(&driver);
    ConboardSDKScript sdkScript(&api);


    //! @note replace these four lines below to change to an other hard-driver level.
    APIThread send(&api, 1);
    APIThread read(&api, 2);
    send.createThread();
    read.createThread();

    MainControl MC(&sdkScript,&driver,&api);
    MC.Initialize();
    MC.Run();

    MC.Release();
    //!@note ....
//   ScriptThread st(&sdkScript);
//    st.run();



    return 0;
}

