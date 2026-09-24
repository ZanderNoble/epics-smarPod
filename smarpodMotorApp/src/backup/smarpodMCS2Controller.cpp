#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <epicsPrint.h>
#include <iocsh.h>
#include <epicsThread.h>
#include "asynDriver.h"
#include "asynMotorController.h"
#include <epicsExport.h>
#include "smarpodMCS2Controller.h"
#include <cmath> 

const char* SmarpodMotorController::driverName = "SmarPodMotorController";
SmarpodMotorController::SmarpodMotorController(const char* portName, const char* locator,
                                             int numAxes, double movingPollPeriod, double idlePollPeriod,
                                             unsigned int model,int unusedMask)
    : asynMotorController(portName, numAxes, 0, 0, 0, 
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0),
      smarpodID_(0), model_(model), isConnected_(false) {
    
    int axis;
    int axisMask = 0;
    static const char *functionName = "SmarpodMotorController";
    //基本操作
    createParam(SmarPodMotorMoveStatusString, asynParamInt32, &this->moveStatus_);
    createParam(SmarPodMotorCalibratingString, asynParamInt32, &this->motorCalibrate_);
    createParam(SmarPodMotorReferencingString, asynParamInt32, &this->motorReference_);
    createParam(SmarPodMoveToZeroString, asynParamInt32, &this->moveToZero_);
    createParam(SmarPodMotorStatusCalibrateString, asynParamInt32, &this->motorCalibrateStatus_);
    createParam(SmarPodMotorStatusReferenceString, asynParamInt32, &this->motorReferenceStatus_);
    // 创建PV参数
    createParam("PIVOT_X_SET", asynParamFloat64, &PivotX_Set_);
    createParam("PIVOT_Y_SET", asynParamFloat64, &PivotY_Set_);
    createParam("PIVOT_Z_SET", asynParamFloat64, &PivotZ_Set_);
    createParam("PIVOT_X_GET", asynParamFloat64, &PivotX_Get_);
    createParam("PIVOT_Y_GET", asynParamFloat64, &PivotY_Get_);
    createParam("PIVOT_Z_GET", asynParamFloat64, &PivotZ_Get_);
    // 坐标系统PV
    createParam("CS_X_SET", asynParamFloat64, &CS_X_Set_);
    createParam("CS_Y_SET", asynParamFloat64, &CS_Y_Set_);
    createParam("CS_Z_SET", asynParamFloat64, &CS_Z_Set_);
    createParam("CS_RX_SET", asynParamFloat64, &CS_Rx_Set_);
    createParam("CS_RY_SET", asynParamFloat64, &CS_Ry_Set_);
    createParam("CS_RZ_SET", asynParamFloat64, &CS_Rz_Set_);
    createParam("CS_X_GET", asynParamFloat64, &CS_X_Get_);
    createParam("CS_Y_GET", asynParamFloat64, &CS_Y_Get_);
    createParam("CS_Z_GET", asynParamFloat64, &CS_Z_Get_);
    createParam("CS_RX_GET", asynParamFloat64, &CS_Rx_Get_);
    createParam("CS_RY_GET", asynParamFloat64, &CS_Ry_Get_);
    createParam("CS_RZ_GET", asynParamFloat64, &CS_Rz_Get_);
    createParam("CS_SET_EXEC", asynParamInt32, &CS_Set_Exec_);
    createParam("CS_ZERO_EXEC", asynParamInt32, &CS_Zero_Exec_);
    // 轴方向PV
    createParam("AO_RX_SET", asynParamFloat64, &AO_Rx_Set_);
    createParam("AO_RY_SET", asynParamFloat64, &AO_Ry_Set_);
    createParam("AO_RZ_SET", asynParamFloat64, &AO_Rz_Set_);
    createParam("AO_RX_GET", asynParamFloat64, &AO_Rx_Get_);
    createParam("AO_RY_GET", asynParamFloat64, &AO_Ry_Get_);
    createParam("AO_RZ_GET", asynParamFloat64, &AO_Rz_Get_);
    createParam("AO_SET_EXEC", asynParamInt32, &AO_Set_Exec_);
    // 初始化默认值
    memset(&currentCS_, 0, sizeof(Smarpod_Pose));
    memset(currentAO_, 0, sizeof(currentAO_));
    // 设置默认参数值
    setDoubleParam(CS_X_Set_, 0.0);
    setDoubleParam(CS_Y_Set_, 0.0);
    setDoubleParam(CS_Z_Set_, 0.015);
    setDoubleParam(CS_Rx_Set_, 0.0);
    setDoubleParam(CS_Ry_Set_, 0.0);
    setDoubleParam(CS_Rz_Set_, 0.0);
    
    setDoubleParam(AO_Rx_Set_, 0.0);
    setDoubleParam(AO_Ry_Set_, 0.0);
    setDoubleParam(AO_Rz_Set_, 0.0);
    // 初始化轴数组
    pAxes_ = (SmarpodMotorAxis**) calloc(numAxes, sizeof(SmarpodMotorAxis*));

    // 初始化互斥锁
    mutex_ = epicsMutexCreate();
    //初始化校正状态
    setIntegerParam(motorCalibrateStatus_, 0);
    setIntegerParam(motorReferenceStatus_, 0);
    // 保存定位器信息
    strncpy(locator_, locator, sizeof(locator_)-1);
    locator_[sizeof(locator_)-1] = '\0';

    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
              "%s:%s: 创建控制器，定位器: %s, 型号: %u\n", 
              driverName, functionName, locator_, model_);
    // 连接设备
    connect();
    // 创建轴对象
    for(axis = 0; axis < numAxes; axis++) {
        axisMask = (unusedMask & (1 << axis)) >> axis;
        printf("the axi-num=%d axisMask is %d\n",axis,axisMask);
        if(!axisMask){
            pAxes_[axis]=new SmarpodMotorAxis(this, axis);
          }
    }
    callParamCallbacks();
    // 启动轮询
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

// IOC shell注册函数
extern "C" int smarpodMotorCreate(const char* portName, const char* locator, 
                                 int numAxes, int movingPollPeriod, int idlePollPeriod, 
                                 int model=SMARPOD_MODEL_10115,int unusedMask=0) {
    new SmarpodMotorController(portName, locator, numAxes, 
                              movingPollPeriod/1000., idlePollPeriod/1000., 
                              (unsigned int)model,unusedMask);
    return(asynSuccess);
}
SmarpodMotorAxis::SmarpodMotorAxis(SmarpodMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC), axisNo_(axisNo) {
    // 构造函数的初始化代码
}
asynStatus SmarpodMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    if (function == CS_Set_Exec_ && value == 1) {
        // 执行坐标系统设置
        setCoordinateSystem(&currentCS_);
    } else if (function == CS_Zero_Exec_ && value == 1) {
        // 执行当前姿态设零
        setCurrentPoseAsZero();
    } else if (function == AO_Set_Exec_ && value == 1) {
        // 执行轴方向设置
        setAxesOrientation(currentAO_[0], currentAO_[1], currentAO_[2]);
    }
    else if (function==motorCalibrate_){
        calibrateSensor();
    }
    else if (function==motorReference_){
        findReferenceMarks();
    }
    else if (function==moveToZero_){
        setCurrentPoseAsZero();
    }
    else{
        asynMotorController::writeInt32(pasynUser, value);
    }
    return asynSuccess;
}
asynStatus SmarpodMotorController::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    double baseVelocity, velocity, acceleration;
    int autoPower = 0;
    double autoPowerOnDelay = 0.0;
    asynStatus status = asynError;
    static const char *functionName = "writeFloat64";
    asynMotorAxis *pAxis=asynMotorController::getAxis(pasynUser);;
    int axis;
    if (!pAxis) return asynError;
    SmarpodMotorAxis* pSmarpodAxis = static_cast<SmarpodMotorAxis*>(pAxis);
    axis = pSmarpodAxis->axisNo_;
    printf("axis=%d",axis);
    //SmarpodMotorAxis *pCurrentAxis；
   /* if (function == motorRecOffset_ || function == motorRecResolution_ || function == motorRecEncoderResolution_) 
    {
        // 直接返回成功，避免write error
        return asynSuccess;
    }*/
    if (function == PivotX_Set_) {
        currentPivot_[0] = value;
    } 
    else if (function == PivotY_Set_) {
        currentPivot_[1] = value;
    } 
    else if (function == PivotZ_Set_) {
        currentPivot_[2] = value;
        // Z轴设置后，执行实际的旋转中心设置
        printf("通过PV设置旋转中心: (%.6f, %.6f, %.6f) m\n", 
               currentPivot_[0], currentPivot_[1], currentPivot_[2]);
        setPivot(currentPivot_);
        // 更新GET PV的值
        setDoubleParam(PivotX_Get_, currentPivot_[0]);
        setDoubleParam(PivotY_Get_, currentPivot_[1]);
        setDoubleParam(PivotZ_Get_, currentPivot_[2]);

    }
    // 坐标系统设置
    else if (function == CS_X_Set_) {
        currentCS_.positionX = value;
    } else if (function == CS_Y_Set_) {
        currentCS_.positionY = value;
    } else if (function == CS_Z_Set_) {
        currentCS_.positionZ = value;
    } else if (function == CS_Rx_Set_) {
        currentCS_.rotationX = value;
    } else if (function == CS_Ry_Set_) {
        currentCS_.rotationY = value;
    } else if (function == CS_Rz_Set_) {
        currentCS_.rotationZ = value;
    } 
    // 轴方向设置
    else if (function == AO_Rx_Set_) {
        currentAO_[0] = value;
    } else if (function == AO_Ry_Set_) {
        currentAO_[1] = value;
    } else if (function == AO_Rz_Set_) {
        currentAO_[2] = value;
    }
    else{
        asynMotorController::writeFloat64(pasynUser, value);
    }
    callParamCallbacks();
    return status;
}
int SmarpodMotorController::getAxisIndex(asynUser *pasynUser) {
    // 方法2：直接从 userPvt 获取并验证
    int axisNo = (int)(intptr_t)pasynUser->userPvt;
    
    // 验证轴号是否有效
    if (axisNo >= 0 && axisNo < numAxes_) {
        return axisNo;
    }
    printf("警告：无效轴号 %d (有效范围 0-%d)\n", axisNo, numAxes_-1);
    // 返回无效轴号标识
    return -1;
}
asynStatus SmarpodMotorController::readFloat64(asynUser *pasynUser, epicsFloat64 *value) {
    int function = pasynUser->reason;
    
    if (function == PivotX_Get_) {
        *value = currentPivot_[0];
    } 
    else if (function == PivotY_Get_) {
        *value = currentPivot_[1];
    } 
    else if (function == PivotZ_Get_) {
        *value = currentPivot_[2];
    }
    else
    {
        asynMotorController::readFloat64(pasynUser, value);
    }
    return asynSuccess;
}

asynStatus SmarpodMotorController::PrintStatus(const char* operation, Smarpod_Status status) {
    if (status == SMARPOD_OK) {
        printf("[成功] %s\n", operation);
    } 
    else if (status == SMARPOD_SYSTEM_CONFIGURATION_ERROR) {
        printf("[警告] %s - 系统配置不匹配（功能正常）\n", operation);
    } 
    else {
        const char *info;
        Smarpod_GetStatusInfo(status, &info);
        printf("[错误] %s: %s\n", operation, info);
    }
    return (status == SMARPOD_OK) ? asynSuccess : asynError; 
}
// 离线配置控制器（Smarpod_ConfigureController）
asynStatus SmarpodMotorController::configureController() {
    epicsMutexLock(mutex_);
    printf("开始配置控制器（型号: %u, 定位器: %s）...\n", model_, locator_);
    Smarpod_Status st = Smarpod_ConfigureController(model_, locator_, "");
    if (st == SMARPOD_OK) {
        printf("✅ 控制器配置成功！\n");
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        const char* errInfo;
        if (Smarpod_GetStatusInfo(st, &errInfo)) {
            printf("❌ 控制器配置失败 - 未知错误代码: %d\n", st);
        } else {
            printf("❌ 控制器配置失败: %s (代码: %d)\n", errInfo, st);
        }
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}

// 在线配置系统（Smarpod_ConfigureSystem）
asynStatus SmarpodMotorController::configureSystem() {
    epicsMutexLock(mutex_);
    
    if (!isConnected_) {
        printf("❌ 设备未连接，无法配置系统！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    printf("开始配置系统（设备ID: %u）...\n", smarpodID_);
    Smarpod_Status st = Smarpod_ConfigureSystem(smarpodID_);
    if (st == SMARPOD_OK) {
        printf("✅ 系统配置成功！\n");
        epicsMutexUnlock(mutex_);
        // 调用独立的校准和参考点查找函数
        calibrateSensor();
        if (!isReferenced()) {
            findReferenceMarks();
        }
        return asynSuccess;
    } else {
        const char* errInfo;
        if (Smarpod_GetStatusInfo(st, &errInfo)) {
            printf("❌ 系统配置失败 - 未知错误代码: %d\n", st);
        } else {
            printf("❌ 系统配置失败: %s (代码: %d)\n", errInfo, st);
        }
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}

asynStatus SmarpodMotorController::connect() {
    epicsMutexLock(mutex_);
    checkDeviceStatus();
    
    // 获取传感器模式
    unsigned int sensorMode;
    Smarpod_GetSensorMode(smarpodID_, &sensorMode);
    unsigned int verifyMode;
    getSensorMode(&verifyMode);
    //configureSystem();
    // 检查参考状态
    printf("参考状态: %s\n", isReferenced() ? "已参考" : "未参考");
    if (!isReferenced()) {
        //findReferenceMarks();
    }
    unsigned int verifyFrequency;
    getMaxFrequency(&verifyFrequency);
    // 获取当前姿态
    Smarpod_Pose pose;
    if(Smarpod_GetPose(smarpodID_, &pose) == SMARPOD_OK) {
        printf("当前姿态:\n");
        printf("  位置: X=%.6f mm, Y=%.6f mm, Z=%.6f mm\n", 
               pose.positionX*1000, pose.positionY*1000, pose.positionZ*1000);
        printf("  旋转: Rx=%.3f°, Ry=%.3f°, Rz=%.3f°\n", 
               pose.rotationX, pose.rotationY, pose.rotationZ);
    }
    epicsMutexUnlock(mutex_);
    return isConnected_ ? asynSuccess : asynError;
}
//检查库及设备的基础配置信息
asynStatus SmarpodMotorController::checkDeviceStatus() {
    printf("\n=== Smarpod设备状态信息 ===\n");
    // 获取设备信息
    char systems_buffer[4096];
    unsigned int buffer_size = sizeof(systems_buffer);
    Smarpod_FindSystems("", systems_buffer, &buffer_size);
    printf("📡 扫描到设备: %s\n", systems_buffer);
    // 版本信息
    unsigned int major, minor, update;
    Smarpod_GetDLLVersion(&major, &minor, &update);
    printf("🔧 Smarpod库版本: %u.%u.%u\n", major, minor, update);
    //configureController();
    // 打开设备连接
    Smarpod_Status st = Smarpod_Open(&smarpodID_, model_, locator_, "");
    if(st == SMARPOD_OK) {
        printf("成功连接到SmarPod设备，ID: %u\n", smarpodID_);
        isConnected_ = true;
        // 检查设备状态
    } else if(st == SMARPOD_SYSTEM_CONFIGURATION_ERROR) {
        printf("警告: 系统配置不匹配，需要执行配置\n");
        isConnected_ = false;
    } else {
        logSmarpodError(st, "连接失败");
        isConnected_ = false;
    }
    //设备号对应的名字
    const char *name;
    Smarpod_Status result;
    result = Smarpod_GetModelName(model_,&name);
    if(result == SMARPOD_OK) {
    printf("🔧 Name for SmarPod model code %d is: %s\n",model_,name);
    }
    //Smarpod_GetSystemLocator
    char outLocator[4096];
    unsigned int bufferSize = sizeof(outLocator);
    result = Smarpod_GetSystemLocator(smarpodID_, outLocator, &bufferSize);
    if(result == SMARPOD_OK){
    // outLocator holds the locator string.
    // bufferSize holds the number of bytes written to outLocator.
    printf("🔧 Name for SmarPod locator code %s is: %u.\n",locator_,bufferSize);
    }
    printf("===========================\n\n");
    return asynSuccess;
}
//断开设备连接
asynStatus SmarpodMotorController::disconnect() {
    epicsMutexLock(mutex_);
    if(smarpodID_ != 0) {
        Smarpod_Status st = Smarpod_Close(smarpodID_);
        if(st == SMARPOD_OK) {
            printf("成功断开SmarPod设备连接\n");
        } else {
            logSmarpodError(st, "断开连接失败");
        }
        isConnected_ = false;
    }
    
    epicsMutexUnlock(mutex_);
    return asynSuccess;
}
asynStatus SmarpodMotorAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    epicsMutexLock(pC_->mutex_);
    asynStatus status = asynSuccess;
    static const char *functionName = "move";
    Smarpod_Pose currentPose;
    Smarpod_Pose targetPose;  
    Smarpod_Status result;  
    pC_->getDoubleParam(axisNo_, pC_->motorVelocity_, &maxVelocity);
    pC_->getDoubleParam(axisNo_, pC_->motorAccel_, &acceleration); 
    printf("maxVelocity=%f，acceleration=%f\n",maxVelocity,acceleration);
    Smarpod_SetAcceleration(pC_->smarpodID_,SMARPOD_TRUE,acceleration);
    Smarpod_SetSpeed(pC_->smarpodID_,SMARPOD_TRUE,maxVelocity);
    printf("\n===============\naxis=%d will be move!\n", axisNo_);  // 修复：使用正确的轴号变量
    printf("position=%f\n",position);
    if (Smarpod_GetPose(pC_->smarpodID_, &currentPose) == SMARPOD_OK) {
    printf("\ncurrentPose: X=%.6f m, Y=%.6f m, Z=%.6f m，Rotation: Rx=%.2f°, Ry=%.2f°, Rz=%.2f°\n\n",currentPose.positionX, currentPose.positionY, currentPose.positionZ,currentPose.rotationX, currentPose.rotationY, currentPose.rotationZ);
        unsigned int moveStatus;
        if (Smarpod_GetMoveStatus(pC_->smarpodID_, &moveStatus) == SMARPOD_OK) {
            int isMoving = (moveStatus == SMARPOD_MOVING) ? 1 : 0;
            //setIntegerParam(pC_->motorStatusDone_, !isMoving);
            //setIntegerParam(pC_->motorStatusMoving_, isMoving);  
            if (isMoving == 0) {
                targetPose = currentPose;
                printf("\ntargetPose 1: X=%.6f m, Y=%.6f m, Z=%.6f m，Rotation: Rx=%.2f°, Ry=%.2f°, Rz=%.2f°\n\n",targetPose.positionX, targetPose.positionY, targetPose.positionZ,targetPose.rotationX, targetPose.rotationY, targetPose.rotationZ);
                if (relative == 0) {
                    // 绝对运动
                    switch(axisNo_) {
                        case 0: targetPose.positionX = position; break;
                        case 1: targetPose.positionY = position; break;  
                        case 2: targetPose.positionZ = position; break;
                        case 3: targetPose.rotationX = position; break;
                        case 4: targetPose.rotationY = position; break;
                        case 5: targetPose.rotationZ = position; break;
                    }
                } else {
                    // 相对运动
                    switch(axisNo_) {
                        case 0: targetPose.positionX += position; break;
                        case 1: targetPose.positionY += position; break;
                        case 2: targetPose.positionZ += position; break;
                        case 3: targetPose.rotationX += position; break;
                        case 4: targetPose.rotationY += position; break;
                        case 5: targetPose.rotationZ += position; break;
                    }
                }
                printf("\ntargetPose 2: X=%.6f m, Y=%.6f m, Z=%.6f m，Rotation: Rx=%.2f°, Ry=%.2f°, Rz=%.2f°\n\n",targetPose.positionX, targetPose.positionY, targetPose.positionZ,targetPose.rotationX, targetPose.rotationY, targetPose.rotationZ);
                int reachable;
                result = Smarpod_IsPoseReachable(pC_->smarpodID_, &targetPose, &reachable);
                if (result != SMARPOD_OK || !reachable) {
                    printf("The result of the test on reachability is error\n");
                    status = asynError;
                } else {
                    result = Smarpod_Move(pC_->smarpodID_, &targetPose, SMARPOD_HOLDTIME_INFINITE, 1);
                    if (result != SMARPOD_OK) {
                        printf("Smarpod_Move failed with error code: %d\n", result);
                        status = asynError;
                    } else {
                        printf("Axis %d move command sent successfully\n", axisNo_);
                        //setIntegerParam(pC_->motorStatusDone_, 0);
                        callParamCallbacks();
                    }
                }
            } else {
                printf("Axis %d is already moving, cannot execute new move command\n", axisNo_);
                status = asynError;
            }
        } else {
            printf("Failed to get move status\n");
            status = asynError;
        }
    } else {
        printf("Failed to get current pose\n");
        status = asynError;
    }
    epicsMutexUnlock(pC_->mutex_);
    return status;
}

//执行calibrate程序
asynStatus SmarpodMotorController::calibrateSensor() {
    int calingStatus;
    epicsMutexLock(mutex_);
        printf("calingStatus  0  ==%d\n",calingStatus);
    setIntegerParam(motorCalibrateStatus_, 1);
    callParamCallbacks();
    getIntegerParam(motorCalibrateStatus_, &calingStatus);
    printf("calingStatus  1  ==%d\n",calingStatus);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法进行传感器校准！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    printf("开始传感器校准（设备ID: %u）...\n", smarpodID_);
    Smarpod_Status st = Smarpod_Calibrate(smarpodID_);
    if (st == SMARPOD_OK) {
        printf("✅ 传感器校准成功！\n");
        setIntegerParam(motorCalibrateStatus_, 2);
        getIntegerParam(motorCalibrateStatus_, &calingStatus);
        printf("calingStatus  2  ==%d\n",calingStatus);
        callParamCallbacks();
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        setIntegerParam(motorCalibrateStatus_, 0);
        callParamCallbacks();
        const char* errInfo;
        if (Smarpod_GetStatusInfo(st, &errInfo)) {
            printf("❌ 传感器校准失败 - 未知错误代码: %d\n", st);
        } else {
            printf("❌ 传感器校准失败: %s (代码: %d)\n", errInfo, st);
        }
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
//执行reference程序
asynStatus SmarpodMotorController::findReferenceMarks() {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法查找参考点！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    printf("开始查找参考点（设备ID: %u）...\n", smarpodID_);
    Smarpod_Status st = Smarpod_FindReferenceMarks(smarpodID_);
    if (st == SMARPOD_OK) {
        printf("✅ 参考点查找成功！\n");
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        const char* errInfo;
        if (Smarpod_GetStatusInfo(st, &errInfo)) {
            printf("❌ 参考点查找失败 - 未知错误代码: %d\n", st);
        } else {
            printf("❌ 参考点查找失败: %s (代码: %d)\n", errInfo, st);
        }
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
//检查参考点状态
bool SmarpodMotorController::isReferenced() {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        epicsMutexUnlock(mutex_);
        return false;
    }
    int referenced = SMARPOD_FALSE;
    Smarpod_Status st = Smarpod_IsReferenced(smarpodID_, &referenced);
    epicsMutexUnlock(mutex_);
    if (st == SMARPOD_OK && referenced == SMARPOD_TRUE) {
        return true;
    } else {
        return false;
    }
}
//设置传感器工作模式
asynStatus SmarpodMotorController::setSensorMode(unsigned int mode) {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法设置传感器模式！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    bool isValidMode = (mode == SMARPOD_SENSORS_DISABLED || mode == SMARPOD_SENSORS_ENABLED || mode == SMARPOD_SENSORS_POWERSAVE);
    if (!isValidMode) {
        printf("❌ 无效的传感器模式: %u\n", mode);
        printf("   有效值：SMARPOD_SENSORS_DISABLED(%u), SMARPOD_SENSORS_ENABLED(%u), SMARPOD_SENSORS_POWERSAVE(%u)\n",
               SMARPOD_SENSORS_DISABLED, SMARPOD_SENSORS_ENABLED, SMARPOD_SENSORS_POWERSAVE);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    const char* modeStr = "未知";
    switch(mode) {
        case SMARPOD_SENSORS_DISABLED:  modeStr = "禁用";    break;
        case SMARPOD_SENSORS_ENABLED:   modeStr = "启用";    break;
        case SMARPOD_SENSORS_POWERSAVE: modeStr = "省电";    break;
    }
    printf("设置传感器模式为: %s...\n", modeStr);
    Smarpod_Status st = Smarpod_SetSensorMode(smarpodID_, mode);
    if (st == SMARPOD_OK) {
        printf("✅ 传感器模式设置请求已发送\n");
        printf("正在验证设置结果...\n");
        unsigned int verifyMode;
        if (getSensorMode(&verifyMode) == asynSuccess) {
            if (verifyMode == mode) {
                printf("✅ 传感器模式验证成功：已设置为 %s (%u)\n", modeStr, mode);
            } else {
                printf("⚠️ 传感器模式验证不一致：期望 %s(%u)，实际 %u\n", modeStr, mode, verifyMode);
            }
        }
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        const char* errInfo = nullptr;
        Smarpod_GetStatusInfo(st, &errInfo);
        printf("❌ 设置传感器模式失败: %s (错误码: %d)\n", 
               errInfo ? errInfo : "未知错误", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
//获取传感器工作模式
asynStatus SmarpodMotorController::getSensorMode(unsigned int* mode) {
    epicsMutexLock(mutex_);
    // 1. 参数有效性检查
    if (!isConnected_) {
        printf("❌ 设备未连接，无法获取传感器模式！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    Smarpod_Status st = Smarpod_GetSensorMode(smarpodID_, mode);
    if (st == SMARPOD_OK) {
        const char* modeStr = "未知";
        switch(*mode) {  // 注意这里是*mode，不是mode
            case SMARPOD_SENSORS_DISABLED:  modeStr = "Disable";    break;
            case SMARPOD_SENSORS_ENABLED:   modeStr = "Enable";    break;
            case SMARPOD_SENSORS_POWERSAVE: modeStr = "PowerSave";    break;
        }
        printf("✅ 当前传感器模式: %s (%u)\n", modeStr, *mode);
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        const char* errInfo = nullptr;
        Smarpod_GetStatusInfo(st, &errInfo);
        printf("❌ 获取传感器模式失败: %s (错误码: %d)\n", 
               errInfo ? errInfo : "未知错误", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
//设置最大频率
asynStatus SmarpodMotorController::setMaxFrequency(unsigned int frequency) {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法设置最大频率！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    // 验证频率范围（1到18500）
    if (frequency < 1 || frequency > 18500) {
        printf("❌ 无效的最大频率值: %u\n", frequency);
        printf("   有效值范围：1 到 18500 Hz\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    printf("设置最大频率为: %u Hz...\n", frequency);
    Smarpod_Status st = Smarpod_SetMaxFrequency(smarpodID_, frequency);
    if (st == SMARPOD_OK) {
        printf("✅ 最大频率设置请求已发送\n");
        printf("正在验证设置结果...\n");
        unsigned int verifyFrequency;
        if (getMaxFrequency(&verifyFrequency) == asynSuccess) {
            if (verifyFrequency == frequency) {
                printf("✅ 最大频率验证成功：已设置为 %u Hz\n", frequency);
            } else {
                printf("⚠️ 最大频率验证不一致：期望 %u Hz，实际 %u Hz\n", frequency, verifyFrequency);
            }
        }
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        const char* errInfo = nullptr;
        Smarpod_GetStatusInfo(st, &errInfo);
        printf("❌ 设置最大频率失败: %s (错误码: %d)\n", 
               errInfo ? errInfo : "未知错误", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
//获取最大频率
asynStatus SmarpodMotorController::getMaxFrequency(unsigned int* frequency) {
    epicsMutexLock(mutex_);
    // 参数有效性检查
    if (!isConnected_) {
        printf("❌ 设备未连接，无法获取最大频率！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    if (frequency == nullptr) {
        printf("❌ 输出参数不能为空！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    Smarpod_Status st = Smarpod_GetMaxFrequency(smarpodID_, frequency);
    if (st == SMARPOD_OK) {
        printf("✅ 当前最大频率: %u Hz\n", *frequency);
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        const char* errInfo = nullptr;
        Smarpod_GetStatusInfo(st, &errInfo);
        printf("❌ 获取最大频率失败: %s (错误码: %d)\n", 
               errInfo ? errInfo : "未知错误", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
//设置虚拟旋转中心点
asynStatus SmarpodMotorController::setPivot(const double pivot[3]) {
    epicsMutexLock(mutex_);
    if (pivot == nullptr) {
        printf("❌ 旋转中心点参数不能为空！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    printf("设置虚拟旋转中心点为: (%.6f, %.6f, %.6f) m...\n", 
           pivot[0], pivot[1], pivot[2]);
    Smarpod_Status st = Smarpod_SetPivot(smarpodID_, pivot);
    if (st == SMARPOD_OK) {
        printf("✅ 虚拟旋转中心点设置请求已发送\n");
        printf("正在验证设置结果...\n");
        double verifyPivot[3];
        if (getPivot(verifyPivot) == asynSuccess) {
            // 简单的浮点比较（可根据需求调整精度）
            bool isEqual = (fabs(verifyPivot[0] - pivot[0]) < 1e-6 &&
                           fabs(verifyPivot[1] - pivot[1]) < 1e-6 &&
                           fabs(verifyPivot[2] - pivot[2]) < 1e-6);
            if (isEqual) {
                printf("✅ 虚拟旋转中心点验证成功：(%.6f, %.6f, %.6f) m\n", 
                       verifyPivot[0], verifyPivot[1], verifyPivot[2]);
            } else {
                printf("⚠️ 虚拟旋转中心点验证不一致：\n");
                printf("   期望: (%.6f, %.6f, %.6f) m\n", pivot[0], pivot[1], pivot[2]);
                printf("   实际: (%.6f, %.6f, %.6f) m\n", verifyPivot[0], verifyPivot[1], verifyPivot[2]);
            }
        }
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        const char* errInfo = nullptr;
        Smarpod_GetStatusInfo(st, &errInfo);
        printf("❌ 设置虚拟旋转中心点失败: %s (错误码: %d)\n", 
               errInfo ? errInfo : "未知错误", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
//获取虚拟旋转中心点
asynStatus SmarpodMotorController::getPivot(double pivot[3]) {
    epicsMutexLock(mutex_);
    // 参数有效性检查
    if (!isConnected_) {
        printf("❌ 设备未连接，无法获取虚拟旋转中心点！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    if (pivot == nullptr) {
        printf("❌ 输出参数不能为空！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    Smarpod_Status st = Smarpod_GetPivot(smarpodID_, pivot);
    if (st == SMARPOD_OK) {
        printf("✅ 当前虚拟旋转中心点: (%.6f, %.6f, %.6f) m\n", 
               pivot[0], pivot[1], pivot[2]);
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        const char* errInfo = nullptr;
        Smarpod_GetStatusInfo(st, &errInfo);
        printf("❌ 获取虚拟旋转中心点失败: %s (错误码: %d)\n", 
               errInfo ? errInfo : "未知错误", st); 
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
// 设置坐标系统
asynStatus SmarpodMotorController::setCoordinateSystem(const Smarpod_Pose* csys) {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法设置坐标系统！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    if (!csys) {
        printf("❌ 坐标系统参数不能为空！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    // 检查旋转角度范围 [-45°, 45°]
    if (fabs(csys->rotationX) > 45.0 || fabs(csys->rotationY) > 45.0 || fabs(csys->rotationZ) > 45.0) {
        printf("❌ 旋转角度超出范围 [-45°, 45°]！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    printf("设置坐标系统: (%.6f, %.6f, %.6f) m, (%.2f, %.2f, %.2f)°\n",
           csys->positionX, csys->positionY, csys->positionZ,
           csys->rotationX, csys->rotationY, csys->rotationZ);
    Smarpod_Status st = Smarpod_SetCoordinateSystem(smarpodID_, csys);
    if (st == SMARPOD_OK) {
        memcpy(&currentCS_, csys, sizeof(Smarpod_Pose));
        // 更新GET PV
        updateCoordinateSystemPV();
        printf("✅ 坐标系统设置成功\n");
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        printf("❌ 坐标系统设置失败，错误码: %d\n", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}

// 获取坐标系统
asynStatus SmarpodMotorController::getCoordinateSystem(Smarpod_Pose* csys) {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法获取坐标系统！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    if (!csys) {
        printf("❌ 输出参数不能为空！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    Smarpod_Status st = Smarpod_GetCoordinateSystem(smarpodID_, csys);
    if (st == SMARPOD_OK) {
        memcpy(&currentCS_, csys, sizeof(Smarpod_Pose));
        // 更新GET PV
        updateCoordinateSystemPV();
        printf("✅ 获取坐标系统: (%.6f, %.6f, %.6f) m, (%.2f, %.2f, %.2f)°\n",
               csys->positionX, csys->positionY, csys->positionZ,
               csys->rotationX, csys->rotationY, csys->rotationZ);
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        printf("❌ 获取坐标系统失败，错误码: %d\n", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}

// 当前姿态设为零
asynStatus SmarpodMotorController::setCurrentPoseAsZero() {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法设置当前姿态为零！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    printf("设置当前姿态为坐标原点...\n");
    Smarpod_Status st = Smarpod_SetCurrentPoseAsZero(smarpodID_);
    if (st == SMARPOD_OK) {
        // 重置本地存储
        memset(&currentCS_, 0, sizeof(Smarpod_Pose));
        // 更新PV
        updateCoordinateSystemPV();
        printf("✅ 当前姿态已设为坐标原点\n");
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        printf("❌ 设置当前姿态为零失败，错误码: %d\n", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}

// 设置轴方向
asynStatus SmarpodMotorController::setAxesOrientation(double rx, double ry, double rz) {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法设置轴方向！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    // 检查角度范围
    if (fabs(rx) > 180.0 || fabs(ry) > 60.0 || fabs(rz) > 180.0) {
        printf("❌ 轴方向角度超出范围！\n");
        printf("   Rx: [-180°, 180°], Ry: [-60°, 60°], Rz: [-180°, 180°]\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    printf("设置轴方向: (%.2f, %.2f, %.2f)°\n", rx, ry, rz);
    Smarpod_Status st = Smarpod_SetAxesOrientation(smarpodID_, rx, ry, rz);
    if (st == SMARPOD_OK) {
        currentAO_[0] = rx;
        currentAO_[1] = ry;
        currentAO_[2] = rz;
        // 更新GET PV
        updateAxesOrientationPV();
        printf("✅ 轴方向设置成功\n");
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        printf("❌ 轴方向设置失败，错误码: %d\n", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}

// 获取轴方向
asynStatus SmarpodMotorController::getAxesOrientation(double* rx, double* ry, double* rz) {
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法获取轴方向！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    if (!rx || !ry || !rz) {
        printf("❌ 输出参数不能为空！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    Smarpod_Status st = Smarpod_GetAxesOrientation(smarpodID_, rx, ry, rz);
    if (st == SMARPOD_OK) {
        currentAO_[0] = *rx;
        currentAO_[1] = *ry;
        currentAO_[2] = *rz;
        // 更新GET PV
        updateAxesOrientationPV();
        printf("✅ 获取轴方向: (%.2f, %.2f, %.2f)°\n", *rx, *ry, *rz);
        epicsMutexUnlock(mutex_);
        return asynSuccess;
    } else {
        printf("❌ 获取轴方向失败，错误码: %d\n", st);
        epicsMutexUnlock(mutex_);
        return asynError;
    }
}
// 更新坐标系统PV值
void SmarpodMotorController::updateCoordinateSystemPV() {
    setDoubleParam(CS_X_Get_, currentCS_.positionX);
    setDoubleParam(CS_Y_Get_, currentCS_.positionY);
    setDoubleParam(CS_Z_Get_, currentCS_.positionZ);
    setDoubleParam(CS_Rx_Get_, currentCS_.rotationX);
    setDoubleParam(CS_Ry_Get_, currentCS_.rotationY);
    setDoubleParam(CS_Rz_Get_, currentCS_.rotationZ);
    callParamCallbacks();
}

// 更新轴方向PV值
void SmarpodMotorController::updateAxesOrientationPV() {
    setDoubleParam(AO_Rx_Get_, currentAO_[0]);
    setDoubleParam(AO_Ry_Get_, currentAO_[1]);
    setDoubleParam(AO_Rz_Get_, currentAO_[2]);
    callParamCallbacks();
}

asynStatus SmarpodMotorController::logSmarpodError(Smarpod_Status status, const char* context) {
    if(status != SMARPOD_OK) {
        const char *info;
        if(Smarpod_GetStatusInfo(status, &info)) {
            printf("错误: %s - 未知错误代码: %d\n", context, status);
        } else {
            printf("错误: %s - %s (代码: %d)\n", context, info, status);
        }
    }
    return (status == SMARPOD_OK) ? asynSuccess : asynError;
}

asynMotorAxis* SmarpodMotorController::getAxis(int axisNo) {
    if (axisNo >= 0 && axisNo < numAxes_ && pAxes_[axisNo] != NULL) {
        return pAxes_[axisNo];
    }
    return nullptr;
}
void SmarpodMotorAxis::updateAllAxes() {
    if (!pC_->isConnected_) return;
    
    // 1. 更新当前轴位置（使用控制器的参数索引）
    axesCurrentPosition();
    
    // 2. 获取运动状态
    unsigned int moveStatus;
    bool isMoving = false;
    
    if (Smarpod_GetMoveStatus(pC_->smarpodID_, &moveStatus) == SMARPOD_OK) {
        isMoving = (moveStatus == SMARPOD_MOVING || 
                   moveStatus == SMARPOD_CALIBRATING || 
                   moveStatus == SMARPOD_REFERENCING);
        
        // 使用控制器的参数索引（通过pC_访问）
        pC_->setIntegerParam(pC_->motorStatusDone_, !isMoving);
        pC_->setIntegerParam(pC_->motorStatusMoving_, isMoving);
        pC_->setIntegerParam(pC_->moveStatus_, moveStatus);
        
        // 根据状态细分
        switch(moveStatus) {
            case SMARPOD_STOPPED: 
                pC_->setIntegerParam(pC_->moveStatus_, 0);
                break;
                
            case SMARPOD_HOLDING: 
                pC_->setIntegerParam(pC_->moveStatus_, 1);
                break;
                
            case SMARPOD_MOVING: 
                pC_->setIntegerParam(pC_->moveStatus_, 2);
                break;
                
            case SMARPOD_CALIBRATING: 
                pC_->setIntegerParam(pC_->moveStatus_, 3);
                break;
                
            case SMARPOD_REFERENCING: 
                pC_->setIntegerParam(pC_->moveStatus_, 4);
                break;
                
            case SMARPOD_STANDBY: 
                pC_->setIntegerParam(pC_->moveStatus_, 5);
                break;
                
            default:
                pC_->setIntegerParam(pC_->moveStatus_, 99);
                break;
        }
    }
    
    // 调用控制器的回调函数
    pC_->callParamCallbacks();
}

// 修正位置更新方法（使用控制器的参数）
void SmarpodMotorAxis::axesCurrentPosition() {
    Smarpod_Pose currentPose;
    if (Smarpod_GetPose(pC_->smarpodID_, &currentPose) != SMARPOD_OK) {
        printf("获取失败！");
        return;
    }
    
    double position = 0.0;
    switch(axisNo_) {
        case 0: position = currentPose.positionX * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 1: position = currentPose.positionY * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 2: position = currentPose.positionZ * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 3: position = currentPose.rotationX * SMARPOD_ROTATION_PULSES_PER_STEP; break;
        case 4: position = currentPose.rotationY * SMARPOD_ROTATION_PULSES_PER_STEP; break;
        case 5: position = currentPose.rotationZ * SMARPOD_ROTATION_PULSES_PER_STEP; break;
    }
    
    // 使用控制器的参数索引更新当前轴位置
    pC_->setDoubleParam(axisNo_, pC_->motorPosition_, position);
    pC_->setDoubleParam(axisNo_, pC_->motorEncoderPosition_, position);
}

asynStatus SmarpodMotorController::poll()
{
    
    static int count = 0;
    count++;
    //updateAllAxes(); 
    return asynSuccess;
}
asynStatus SmarpodMotorAxis::poll(bool *moving) {
    // 更新当前轴的位置
    updateAllAxes();
    
    // 检查运动状态
    unsigned int moveStatus;
    *moving = (Smarpod_GetMoveStatus(pC_->smarpodID_, &moveStatus) == SMARPOD_OK) && 
              (moveStatus == SMARPOD_MOVING);
    
    return asynSuccess;
}
// 注册函数：在此处注册你的设备支持（如 motor 驱动、记录类型等）,注册IOC命令
static const iocshArg smarpodCreateArg0 = {"Port name", iocshArgString};
static const iocshArg smarpodCreateArg1 = {"Locator", iocshArgString};
static const iocshArg smarpodCreateArg2 = {"Number of axes", iocshArgInt};
static const iocshArg smarpodCreateArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg smarpodCreateArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg smarpodCreateArg5 = {"Model", iocshArgInt};

static const iocshArg * const smarpodCreateArgs[] = {&smarpodCreateArg0, &smarpodCreateArg1, 
                                                    &smarpodCreateArg2, &smarpodCreateArg3, 
                                                    &smarpodCreateArg4, &smarpodCreateArg5};

static const iocshFuncDef smarpodCreateDef = {"smarpodMotorCreate", 6, smarpodCreateArgs};

static void smarpodCreateCallFunc(const iocshArgBuf *args) {
    smarpodMotorCreate(args[0].sval, args[1].sval, args[2].ival, 
                      args[3].ival, args[4].ival, args[5].ival);
}

static void smarpodMotorRegister(void) {
    iocshRegister(&smarpodCreateDef, smarpodCreateCallFunc);
}

extern "C" {
epicsExportRegistrar(smarpodMotorRegister);
// 具体内容根据你的设备支持逻辑实现
}

