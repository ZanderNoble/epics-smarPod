/*
* Copyright (c) 2020 SmarAct GmbH
*
* Programming example for the SmarPod C/C++ API.
*
* THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
* WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
* BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
* FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
* THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
* REMAINS WITH YOU.
* IN  NO  EVENT  SHALL  THE  SMARACT  GMBH  BE  LIABLE  FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
* OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
*/


#include <stdlib.h>
#include <stdio.h>
#include <SmarPod.h>


const int cCalibrate = 0;
const int cEnableAccelerationControl = 0;
const double cAcceleration = 0.001;



int LogError(Smarpod_Status status)
{
    if(status != SMARPOD_OK)
    {
        const char *info;
        if(Smarpod_GetStatusInfo(status,&info))
            printf("unknown SmarPod status\n");
        else
            printf("error: %s\n",info);
    }
    return status;
}


void ExitOnError(Smarpod_Status status)
{
    if(LogError(status))
        exit(1);
}


int main()
{
    const double kXyMax = 0.0102;
    const Smarpod_Pose pZero = { 0.0,0.0,0.0, 0.0,0.0,0.0 };
    const Smarpod_Pose pZ0Left = { -kXyMax,0.0,0.0, 0.0,0.0,0.0 };
    const Smarpod_Pose pZ0Right = { kXyMax,0.0,0.0, 0.0,0.0,0.0 };

    /* replace with the code for YOUR SmarPod model: */
    unsigned int model = 10115;    

    /* change the locator of your positioner controller: if it is connected
       over USB, "usb:ix:0" should work. If if it connected over ethernet,
       use the network locator format and replace the ip address by the one
       of your controller. (for MCS 1 controllers append the port number
       :5000, for MCS 2 controllers, when specifying the locator with an IPv4 
       address, no port should be appended. alternatively, for MCS 2 the
       serial number can be used in the locator)
    */
    //const char *locator = "usb:ix:0";
    const char* locator = "network:198.168.2.200";
    //const char* locator = "network:sn:MCS2-00020183";
#if 0
    const char *locator = "usb:id:123456789";               /* MCS 1 */
    const char* locator = "network:ip:198.168.1.200:5000";  /* MCS 1 */
    const char *locator = "usb:sn:MCS2-00001234";           /* MCS 2 */
    const char* locator = "network:ip:198.168.1.200";       /* MCS 2 */
    const char* locator = "network:sn:MCS2-000001234";      /* MCS 2 */
#endif

    unsigned int major,minor,update;
    int referenced = SMARPOD_FALSE;
    Smarpod_Pose pose;
    unsigned int mstatus;
    unsigned int id;

    ExitOnError( Smarpod_GetDLLVersion(&major,&minor,&update) );
    printf("using SmarPod library version %u.%u.%u\n",major,minor,update);

    /* initialize the SmarPod and connect to the controller */
    ExitOnError( Smarpod_Open(&id,model,locator,"") );

    /* set the sensor power-move to (always) enabled */
    ExitOnError( Smarpod_SetSensorMode(id,SMARPOD_SENSORS_ENABLED));

    /* calibrate the sensors. this has to be done only once
       if the sensors are already calibrated, this step can be skipped */
    if(cCalibrate) 
        ExitOnError( Smarpod_Calibrate(id) );

    /* check if the actuators know their absolute positions */
    ExitOnError( Smarpod_IsReferenced(id,&referenced) );
    /* ... if not, find the reference-marks */
    if(!referenced)
        ExitOnError( Smarpod_FindReferenceMarks(id) );      

    /* optionally set a maximum acceleration (see SmarPod Programmer's Guide) */
    if(cEnableAccelerationControl)                          
        LogError( Smarpod_SetAcceleration(id,SMARPOD_TRUE,cAcceleration) );

    /* set the movement speed to 2mm/sec */
    LogError( Smarpod_SetSpeed(id,SMARPOD_TRUE,0.002) );    

    /* move and wait until movement has finished */
    LogError( Smarpod_Move(id,&pZ0Left,SMARPOD_HOLDTIME_INFINITE,SMARPOD_TRUE) ); 

    LogError( Smarpod_SetSpeed(id,SMARPOD_TRUE,0.006) );
    LogError( Smarpod_Move(id,&pZ0Right,SMARPOD_HOLDTIME_INFINITE,SMARPOD_TRUE) );
    LogError( Smarpod_SetSpeed(id,SMARPOD_TRUE,0.01) );
    LogError( Smarpod_Move(id,&pZero,SMARPOD_HOLDTIME_INFINITE,SMARPOD_TRUE) );

    LogError( Smarpod_GetPose(id,&pose) );
    printf("pose = (%lf,%lf,%lf,%lf,%lf,%lf)\n",
        pose.positionX,pose.positionY,
        pose.positionZ,pose.rotationX,
        pose.rotationY,pose.rotationZ);

    LogError( Smarpod_GetMoveStatus(id,&mstatus) );
    printf("move-status = %u\n",mstatus);

    /* release SmarPod */
    LogError( Smarpod_Close(id) );

    return 0;
}

