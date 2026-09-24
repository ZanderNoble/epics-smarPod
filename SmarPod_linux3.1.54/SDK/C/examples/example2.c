/*
* Copyright (c) 2020 SmarAct GmbH
*
* Programming example for the SmarPod C/C++ API.
* This example demostrates how to use the Smarpod_ConfigureSystem command.
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


const unsigned int cSmarpodModel = 10001;


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
    /*
    change the locator of your positioner controller: if it is connected
    over USB, "usb:ix:0" should work. If if it connected over ethernet,
    use the network locator format and replace the ip address by the one
    of your controller. (for MCS 1 controllers append the port number
    :5000, for MCS 2 controllers, when specifying the locator with an IPv4 
    address, no port should be appended. alternatively, for MCS 2 the
    serial number can be used in the locator)
    */
    const char *locator = "usb:ix:0";

#if 0
    const char *locator = "usb:id:123456789";               /* MCS 1 */
    const char* locator = "network:ip:198.168.1.200:5000";  /* MCS 1 */
    const char *locator = "usb:sn:MCS2-00001234";           /* MCS 2 */
    const char* locator = "network:ip:198.168.1.200";       /* MCS 2 */
    const char* locator = "network:sn:MCS2-000001234";      /* MCS 2 */
#endif


    unsigned int id;
    /* initialize the SmarPod at the first USB MCS */
    Smarpod_Status st = Smarpod_Open(&id,cSmarpodModel,locator,"");           
    if(st != SMARPOD_OK && st != SMARPOD_SYSTEM_CONFIGURATION_ERROR)
    {
        /* the result is neither OK nor an invalid system config => quit */
        ExitOnError(st);
    }

    /* configure the MCS or MCS2 controller for the SmarPod model code
       specified in the call to Open.
       do once this after: 
        - updating the controller firmware, 
        - replacing the controller,
        - replacing the SmarPod,
        - getting the controller or SmarPod back from a repair.
      the configuration is written to non-volatile controller memory
      and has to be done only once after the changes listed above.
    */
    ExitOnError( Smarpod_ConfigureSystem(id) );

    /* after configuring the controller the sensors must be re-calibrated */
    ExitOnError( Smarpod_Calibrate(id) );        

    LogError( Smarpod_Close(id) );
    return 0;
}

