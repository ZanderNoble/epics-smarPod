/** ********************************************************************
*
* @mainpage SmarPod API
*
* Copyright (c) 2010-2025  SmarAct GmbH
*
* File name: SmarPod.h
*
* This is the software interface to the SmarAct SmarPod system.
* Please refer to the Programmer's Guide for a detailed documentation.
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
********************************************************************* */

#ifndef SMARPOD_H
#define SMARPOD_H


#if defined(_WIN32)
#  include <Windows.h>
#  define SMARPOD_PLATFORM_WINDOWS
#elif defined(__linux__)
#  define SMARPOD_PLATFORM_LINUX
#else
#  define SMARPOD_PLATFORM_OTHER
#endif


#if defined(SMARPOD_PLATFORM_WINDOWS)
#  if !defined(_SMARPOD_DIRECTLINK)
#    ifdef SMARPOD_EXPORTS
#      define SMARPOD_API __declspec(dllexport)
#    else
#      define SMARPOD_API __declspec(dllimport)
#    endif
#  else
#    define SMARPOD_API
#  endif
#  define SMARPOD_CC __cdecl
#else
#  define SMARPOD_API __attribute__ ((visibility ("default")))
#  define SMARPOD_CC
#endif



#ifdef __cplusplus
extern "C" {
#endif


/************************************************************************
*************************************************************************
**                          API version number                         **
*************************************************************************
************************************************************************/

/** @constants ApiVersion */
#define SMARPOD_API_VERSION_MAJOR                   1
#define SMARPOD_API_VERSION_MINOR                   11
#define SMARPOD_API_VERSION_UPDATE                  7
/** @endconstants ApiVersion */


/************************************************************************
*************************************************************************
**                            status codes                             **
*************************************************************************
************************************************************************/

/** @constants ErrorCode */

#define SMARPOD_OK                                  0
#define SMARPOD_OTHER_ERROR                         1
#define SMARPOD_SYSTEM_NOT_INITIALIZED_ERROR        2
#define SMARPOD_NO_SYSTEMS_FOUND_ERROR              3
#define SMARPOD_INVALID_PARAMETER_ERROR             4
#define SMARPOD_COMMUNICATION_ERROR                 5
#define SMARPOD_UNKNOWN_PROPERTY_ERROR              6
#define SMARPOD_RESOURCE_TOO_OLD_ERROR              7
#define SMARPOD_FEATURE_UNAVAILABLE_ERROR           8
#define SMARPOD_INVALID_SYSTEM_LOCATOR_ERROR        9
#define SMARPOD_QUERYBUFFER_SIZE_ERROR              10
#define SMARPOD_COMMUNICATION_TIMEOUT_ERROR         11
#define SMARPOD_DRIVER_ERROR                        12

#define SMARPOD_STATUS_CODE_UNKNOWN_ERROR           500
#define SMARPOD_INVALID_ID_ERROR                    501
#define SMARPOD_INITIALIZED_ERROR                   502
#define SMARPOD_HARDWARE_MODEL_UNKNOWN_ERROR        503
#define SMARPOD_WRONG_COMM_MODE_ERROR               504
#define SMARPOD_NOT_INITIALIZED_ERROR               505
#define SMARPOD_INVALID_SYSTEM_ID_ERROR             506
#define SMARPOD_NOT_ENOUGH_CHANNELS_ERROR           507
#define SMARPOD_INVALID_CHANNEL_ERROR               508
#define SMARPOD_CHANNEL_USED_ERROR                  509
#define SMARPOD_SENSORS_DISABLED_ERROR              510
#define SMARPOD_WRONG_SENSOR_TYPE_ERROR             511
#define SMARPOD_SYSTEM_CONFIGURATION_ERROR          512
#define SMARPOD_SENSOR_NOT_FOUND_ERROR              513
#define SMARPOD_STOPPED_ERROR                       514
#define SMARPOD_BUSY_ERROR                          515

#define SMARPOD_NOT_REFERENCED_ERROR                550
#define SMARPOD_POSE_UNREACHABLE_ERROR              551
#define SMARPOD_COMMAND_OVERRIDDEN_ERROR            552
#define SMARPOD_ENDSTOP_REACHED_ERROR               553
#define SMARPOD_NOT_STOPPED_ERROR                   554
#define SMARPOD_COULD_NOT_REFERENCE_ERROR           555
#define SMARPOD_COULD_NOT_CALIBRATE_ERROR           556

/** @endconstants ErrorCode */


/************************************************************************
*************************************************************************
**                             constants                               **
*************************************************************************
************************************************************************/

/** @constants Global */

#define SMARPOD_DEFAULT                             0

#define SMARPOD_FALSE                               0
#define SMARPOD_TRUE                                1

/* infinite actuator position holdtime */
#define SMARPOD_HOLDTIME_INFINITE                   60000

/** @endconstants Global */


/** @constants Property */

/* property symbols.
  to read or write the respective property use the functions
  Smarpod_Get_<SUFFIX> and Smarpod_Set_<SUFFIX> where

  <SUFFIX>:         DATA TYPE OF THE PROPERTY:
    ui              unsigned int
    i               int
    d               double
*/
#define SMARPOD_FREF_METHOD                         1000  /* unsigned int */
#define SMARPOD_FREF_ZDIRECTION                     1002  /* unsigned int */
#define SMARPOD_FREF_XDIRECTION                     1003  /* unsigned int */
#define SMARPOD_FREF_YDIRECTION                     1004  /* unsigned int */
#define SMARPOD_PIVOT_MODE                          1010  /* unsigned int */
#define SMARPOD_FREF_AND_CAL_FREQUENCY              1020  /* unsigned int */
#define SMARPOD_POSITIONERS_MIN_SPEED               1100  /* double */

/** @endconstants Property */


/** @constants Axis */
#define SMARPOD_X                                   0x0001
#define SMARPOD_Y                                   0x0002
#define SMARPOD_Z                                   0x0004
/** @endconstants Axis */

/** @constants Direction */
#define SMARPOD_POSITIVE                            0x0100
#define SMARPOD_NEGATIVE                            0x0200
#define SMARPOD_REVERSE                             0x1000
/** @endconstants Direction */



/** @constants PivotMode */
#define SMARPOD_PIVOT_RELATIVE                      0
#define SMARPOD_PIVOT_FIXED                         1
/** @endconstants PivotMode */


/** @constants FrefMethod */
#define SMARPOD_METHOD_SEQUENTIAL                   1
#define SMARPOD_METHOD_ZSAFE                        2
#define SMARPOD_METHOD_XYSAFE                       3
/** @endconstants FrefMethod */


/** @constants SensorPowerMode */
#define SMARPOD_SENSORS_DISABLED                    0
#define SMARPOD_SENSORS_ENABLED                     1
#define SMARPOD_SENSORS_POWERSAVE                   2
/** @endconstants SensorPowerMode */

/** @constants MoveStatus */
#define SMARPOD_STOPPED                             0
#define SMARPOD_HOLDING                             1
#define SMARPOD_MOVING                              2
#define SMARPOD_CALIBRATING                         3
#define SMARPOD_REFERENCING                         4
#define SMARPOD_STANDBY                             5
/** @endconstants MoveStatus */



/* DEPRECATED CONSTANTS, INCLUDED FOR BACKWARD COMPATIBILITY */
#define SMARPOD_MOVING_ERROR                        SMARPOD_BUSY_ERROR



/************************************************************************
*************************************************************************
**                            data types                               **
*************************************************************************
************************************************************************/

/** @brief Function return status.
*/
typedef unsigned int Smarpod_Status;


/** @brief SmarPod pose.
   
   Describes a pose of a SmarPod:
   positionX, positionY, positionZ in meters,
   rotationX, rotationY, rotationZ in degrees
*/
typedef struct
{
    double positionX;
    double positionY;
    double positionZ;
    double rotationX;
    double rotationY;
    double rotationZ;
}Smarpod_Pose;




/************************************************************************
*************************************************************************
**                             functions                               **
*************************************************************************
************************************************************************/

/** **************************************************************************
   @defgroup Functions
   @{
 ************************************************************************** */



/** @brief Returns the library version.

   @param[out] major Major version.
   @param[out] minor Minor version.
   @param[out] update Update version.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetDLLVersion(unsigned int *major,unsigned int *minor,unsigned int *update);


/** @brief Returns a string representation for a status code.

   @returns INVALID_PARAMETER if @p status is not a known status code.
   @param[in] status The status code.
   @param[out] statusInfo The returned status string.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetStatusInfo(Smarpod_Status status,const char **statusInfo);


/** @brief Returns list of supported model codes.

   @param[out] modelList The buffer the results are written into. the found items are
            separated by a newline character.
   @param[in,out] ioListSize Pass the size of the provided buffer in here,
            on return, it contains the number of characters returned
            in modelList, or the required size, if the supplied buffer
            was too small.

   @size modelList ~ioListSize
   @default ioListSize 1024
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetModels(unsigned int *modelList,unsigned int *ioListSize);


/** @brief Returns the name for a model code.

   @param[in] model The model code.
   @param[out] name The returned model name.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetModelName(unsigned int model,const char **name);


/** @brief Opens a SmarPod and returns a handle to the device.

   @param[out] smarpodId The device handle.
   @param[in] model The SmarPod model code.
   @param[in] locator The controller locator.
   @param[in] options Options (reserved).

   @default options ""
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Open(unsigned int *smarpodId,unsigned int model,const char *locator,const char *options);


/** @brief Closes the SmarPod session.

   @param[in] smarpodId The device handle.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Close(unsigned int smarpodId);


/** @brief Searches for controllers.

   @param options Reserved for now.
   @param[out] systems The buffer for writing the result into. the found items are
            separated by a newline character.
   @param[in,out] ioBufferSize Pass the size of the provided buffer in here,
            on return, it contains the number of characters returned
            in systems, or the required size, if the supplied buffer
            was too small.

   @size systems ~ioBufferSize
   @default ioBufferSize 256
   @default options ""
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_FindSystems(const char *options,char *systems,unsigned int *ioBufferSize);


/** @brief Returns the locator of the controller.

   @param smarpodId SmarPod device handle.
   @param[out] locator The buffer for writing the result into.
   @param[in,out] ioBufferSize Pass the size of the provided buffer in here,
            on return, it contains the number of characters returned
            in locator, or the required size, if the supplied buffer
            was too small.

   @size locator ~ioBufferSize
   @default ioBufferSize 1024
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetSystemLocator(unsigned int smarpodId,char *locator,unsigned int *ioBufferSize);


/** @brief Configures the MCS controller for the selected SmarPod model.

   @param[in] smarpodId The device handle.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_ConfigureSystem(unsigned int smarpodId);


/** @brief Configures the MCS controller for the selected SmarPod model.

   @param[in] model The SmarPod model code.
   @param[in] locator The controller locator.
   @param[in] options Options (reserved).

   @default options ""
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_ConfigureController(unsigned int model,const char* locator,const char* options);



/** @brief Sets an unsigned integer property.

   @param[in] smarpodId The device handle.
   @param[in] property The property key.
   @param[in] value The property value.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Set_ui(unsigned int smarpodId, unsigned int property, unsigned int value);

/** @brief Sets a signed integer property.

   @param[in] smarpodId The device handle.
   @param[in] property The property key.
   @param[in] value The property value.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Set_i(unsigned int smarpodId, unsigned int property, int value);


/** @brief Sets a double precision floating point property.

   @param[in] smarpodId The device handle.
   @param[in] property The property key.
   @param[in] value The property value.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Set_d(unsigned int smarpodId, unsigned int property, double value);


/** @brief Returns the value of an unsigned integer property.

   @param[in] smarpodId The device handle.
   @param[in] property The property key.
   @param[out] value The property value.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Get_ui(unsigned int smarpodId, unsigned int property, unsigned int *value);


/** @brief Returns the value of a signed integer property.

   @param[in] smarpodId The device handle.
   @param[in] property The property key.
   @param[out] value The property value.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Get_i(unsigned int smarpodId, unsigned int property, int *value);


/** @brief Returns the value of double precision floating point property.

   @param[in] smarpodId The device handle.
   @param[in] property The property key.
   @param[out] value The property value.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Get_d(unsigned int smarpodId, unsigned int property, double *value);


/** @brief Sets the sensor power mode.

   @param[in] smarpodId The device handle.
   @param[in] mode The sensor power mode.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_SetSensorMode(unsigned int smarpodId, unsigned int mode);


/** @brief Returns the sensor power mode.

   @param[in] smarpodId The device handle.
   @param[out] mode The sensor power mode.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetSensorMode(unsigned int smarpodId, unsigned int *mode);


/** @brief Sets maximum closed-loop frequency.

   @param[in] smarpodId The device handle.
   @param[in] frequency The frequency in Hz.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_SetMaxFrequency(unsigned int smarpodId,unsigned int frequency);


/** @brief Returns the maximum closed-loop frequency.

   @param[in] smarpodId The device handle.
   @param[out] frequency The frequency in Hz.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetMaxFrequency(unsigned int smarpodId,unsigned int *frequency);


/** @brief Sets the movement speed.

   @param[in] smarpodId The device handle.
   @param[in] speedControl If TRUE (1), speed control is enabled, if FALSE (0), it is disabled.
   @param[in] speed The speed in m/s.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_SetSpeed(unsigned int smarpodId,int speedControl, double speed);


/** @brief Returns the movement speed.

   @param[in] smarpodId The device handle.
   @param[out] speedControl If TRUE (1), speed control is enabled, if FALSE (0), it is disabled.
   @param[out] speed The speed in m/s.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetSpeed(unsigned int smarpodId,int *speedControl, double *speed);


/** @brief Sets the movement acceleration.

   @param[in] smarpodId The device handle.
   @param[in] accelControl If TRUE (1), acceleration control is enabled, if FALSE (0), it is disabled.
   @param[in] acceleration The acceleration in m/s^2.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_SetAcceleration(unsigned int smarpodId,int accelControl, double acceleration);


/** @brief Returns the movement acceleration.

   @param[in] smarpodId The device handle.
   @param[out] accelControl If TRUE (1), acceleration control is enabled, if FALSE (0), it is disabled.
   @param[out] acceleration The acceleration in m/s^2.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetAcceleration(unsigned int smarpodId,int *accelControl, double *acceleration);


/** @brief Search for positioner reference marks.
   If successful, the SmarPod will be referenced afterwards.

   @param[in] smarpodId The device handle.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_FindReferenceMarks(unsigned int smarpodId);


/** @brief Calibrate the SmarPod positioners.

   @param[in] smarpodId The device handle.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Calibrate(unsigned int smarpodId);


/** @brief Returns referenced status.

   @param[in] smarpodId The device handle.
   @param[out] referenced The referenced state: FALSE (0) (not referenced) or TRUE (1) (referenced).
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_IsReferenced(unsigned int smarpodId, int *referenced);


/** @brief Set the pivot point.

   @param[in] smarpodId The device handle.
   @param[in] pivot An array of size three with the pivot point coodinates X,Y,Z.

   @size pivot 3 
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_SetPivot(unsigned int smarpodId, const double *pivot);


/** @brief Returns the pivot point.

   @param[in] smarpodId The device handle.
   @param[out] pivot An array of size three to write the pivot point coodinates X,Y,Z to.

   @size pivot 3 
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetPivot(unsigned int smarpodId, double *pivot);


/** @brief Check if the specified pose is reachable.

   @param[in] smarpodId The device handle.
   @param[in] pose The pose to test.
   @param[out] reachable TRUE (1) if pose is reachable, else FALSE (0).
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_IsPoseReachable(unsigned int smarpodId, const Smarpod_Pose *pose, int *reachable);


/** @brief Returns the current pose.

   @param[in] smarpodId The device handle.
   @param[out] pose The current pose.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetPose(unsigned int smarpodId,Smarpod_Pose *pose);


/** @brief Returns the current move status.

   @param[in] smarpodId The device handle.
   @param[out] status The current move status.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetMoveStatus(unsigned int smarpodId,unsigned int *status);


/** @brief Move to a pose.

   @param[in] smarpodId The device handle.
   @param[in] pose The target pose.
   @param[in] holdTime The time to hold the target pose after move in milliseconds.
   @param[in] waitForCompletion If TRUE (1), wait for termination of movement, otherwise return immediately.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Move(unsigned int smarpodId, const Smarpod_Pose *pose, unsigned int holdTime, int waitForCompletion);


/** @brief Stop any motion.

   @param[in] smarpodId The device handle.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Stop(unsigned int smarpodId);


/** @brief Stop any motion and actively hold current pose.

   @param[in] smarpodId The device handle.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_StopAndHold(unsigned int smarpodId, unsigned int holdTime);


/** @brief Put all positioners into standby mode.

   @param[in] smarpodId The device handle.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Standby(unsigned int smarpodId);


/** @brief Set the coordinate system (only with zero axes orientation).

   @param[in] smarpodId The device handle.
   @param[in] csys The new coordinate system.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_SetCoordinateSystem(unsigned int smarpodId, const Smarpod_Pose *csys);


/** @brief Returns the coordinate system.

   @param[in] smarpodId The device handle.
   @param[out] csys The new coordinate system.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetCoordinateSystem(unsigned int smarpodId, Smarpod_Pose *csys);


/** @brief Set the current pose as coordinate system zero pose (only with zero axes orientation).

   @param[in] smarpodId The device handle.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_SetCurrentPoseAsZero(unsigned int smarpodId);


/** @brief Set the axes orientation (only with zero coordinate system).

   @param[in] smarpodId The device handle.
   @param[in] rx, ry, rz The new axes orientation.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_SetAxesOrientation(unsigned int smarpodId,
    double rx, double ry, double rz);


/** @brief Returns the axes orientation.

   @param[in] smarpodId The device handle.
   @param[out] rx, ry, rz The new axes orientation.
*/
SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_GetAxesOrientation(unsigned int smarpodId,
    double *rx, double *ry, double *rz);

/** @} */



/**************************************************
  DEPRECATED FUNCTIONS AND CONSTANTS
 **************************************************/

#define SMARPOD_MAX_ID                              7

/* for backward compatibility. use SMARPOD_FREF_ZDIRECTION instead. */
#define SMARPOD_FREF_DIRECTION                      SMARPOD_FREF_ZDIRECTION

SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_InitSystems(const unsigned int *initList,unsigned int initListSize);

SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_Initialize(unsigned int smarpodId, unsigned int hardwareModel, unsigned int systemId);

SMARPOD_API
Smarpod_Status SMARPOD_CC Smarpod_ReleaseAll(void);



#ifdef __cplusplus
}
#endif

#endif
