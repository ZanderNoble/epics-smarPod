# Copyright (c) 2020 SmarAct GmbH

# Programming example for the SmarPod Python API.
# This example demonstrates how to use the ConfigureSystem command.

# THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
# WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
# BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
# FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
# THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
# REMAINS WITH YOU.
# IN  NO  EVENT  SHALL  THE  SMARACT  GMBH  BE  LIABLE  FOR ANY DIRECT,
# INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
# OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
#
# WHEN RUNNING, THIS PROGRAMMING EXAMPLE WILL MOVE THE POSITIONER. 
# ENSURE THAT THIS CAN'T CAUSE COLLISIONS. THE POSITIONER MUST HAVE
# SUFFICIENT SPACE TO MOVE OVER ITS FULL RANGE.


import smaract.smarpod as smarpod


def check_lib_compatibility():
    """
    checks that the major version numbers of the Python API and the
    loaded shared library are the same to avoid errors due to 
    incompatibilites.
    raises a RuntimeError if the major version numbers are different.
    """
    vapi = smarpod.api_version
    vlib = smarpod.GetDLLVersion()
    if vapi[0] != vlib[0]:
        raise RuntimeError("incompatible python api and library version")    


h = None    # SmarPod handle
try:
    check_lib_compatibility()

    # change model to the model code of your SmarPod model code before
    # starting the program
    model = 10001

    # set to false to skip calibration after configuring the controller
    # (not recommended)
    do_calibrating = True
    
    # change the locator of your positioner controller: if it is connected
    # over USB, "usb:ix:0" should work. If if it connected over ethernet,
    # use the network locator format and replace the IPv4 address by the one
    # of your controller. (for MCS 1 controllers append the port number
    # :5000, for MCS 2 controllers, when specifying the locator with an IPv4 
    # address, no port should be appended. alternatively, for MCS 2 the
    # serial number can be used in the locator)
    locator = "usb:ix:0"
    #locator = "usb:id:123456789"               # MCS 1
    #locator = "network:198.168.1.200:5000"     # MCS 1
    #locator = "network:ip:198.168.1.200"       # MCS 2
    #locator = "network:sn:MCS2-000001234"      # MCS 2

    # configure the MCS or MCS2 controller for the specified SmarPod model code.
    # do this once after: 
    #   - updating the controller firmware, 
    #   - replacing the controller,
    #   - replacing the SmarPod,
    #   - getting the controller or SmarPod back from a repair.
    # the configuration is written to non-volatile controller memory
    # and has to be done only once after the changes listed above.
    smarpod.ConfigureController(model,locator)
    print("controller %s configured for SmarPod model %s" % (locator,model))

    # after re-configuring the controller, a calibration of the positioner
    # sensors is necessary
    if do_calibrating:
        # open device for calibrating after configuring the controller
        h = smarpod.Open(model,locator)
        print("device opened successfully")

        print("calibrating...")
        smarpod.Calibrate(h)

    print("the controller has been configured")

except smarpod.Error as err:
    print("SMARPOD ERROR: %s -> %s (%s) " % (err.func, smarpod.ErrorCode(err.code), err.code))
    
finally:
    if h != None:
        print("closing device")
        smarpod.Close(h)
        h = None
