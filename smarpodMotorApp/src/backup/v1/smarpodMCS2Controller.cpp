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
    // 新增设备信息参数创建
    createParam(SmarpodScanDevicesString,       asynParamOctet,  &this->scanDevices_);
    createParam(SmarpodConnectStatusString,     asynParamInt32,  &this->connectStatus_);
    createParam(SmarpodLibraryVersionString,    asynParamOctet,  &this->libraryVersion_);
    createParam(SmarpodDeviceIDString,          asynParamInt32,  &this->deviceID_);
    createParam(SmarpodLocatorString,           asynParamOctet,  &this->locatorParam_);
    createParam(SmarpodModelNameString,         asynParamOctet,  &this->modelName_);
    createParam(SmarpodSystemLocatorString,     asynParamOctet,  &this->systemLocator_);
    createParam(SmarpodModelCodeString,         asynParamInt32,  &this->modelCode_);
    createParam(SmarpodMAXFrequencyString,      asynParamInt32,  &this->maxFrequency_);
    //基本操作
    createParam(SmarPodMotorMoveStatusString, asynParamInt32, &this->moveStatus_);
    createParam(SmarPodMotorCalibratingString, asynParamInt32, &this->motorCalibrate_);
    createParam(SmarPodMotorReferencingString, asynParamInt32, &this->motorReference_);
    createParam(SmarPodMotorReferenceMethodString, asynParamInt32, &this->referenceMethod_);
    createParam(SmarPodMotorReferenceZSaftDirectionString, asynParamInt32, &this->referenceZDirection_);
    createParam(SmarPodMotorReferenceXSaftDirectionString, asynParamInt32, &this->referenceXDirection_);
    createParam(SmarPodMotorReferenceYSaftDirectionString, asynParamInt32, &this->referenceYDirection_);
    createParam(SmarPodMotorReferenceFrequencyString, asynParamInt32, &this->referenceFrequency_);
    createParam(SmarPodMoveToZeroString, asynParamInt32, &this->moveToZero_);
    createParam(SmarPodSetToZeroString, asynParamInt32, &this->setToZero_);
    createParam(SmarPodStopAndHoldingString, asynParamInt32, &this->stopHolding_);
    createParam(SmarPodStopHoldingTimeString, asynParamInt32, &this->stopHoldingTime_);
    createParam(SmarPodMotorStatusCalibrateString, asynParamInt32, &this->motorCalibrateStatus_);
    createParam(SmarPodMotorStatusReferenceString, asynParamInt32, &this->motorReferenceStatus_);
    createParam(SmarPodSensorModeString, asynParamInt32, &this->sensorMode_);
    createParam(SmarPodPivotXString, asynParamFloat64, &this->pivotX_);
    createParam(SmarPodPivotYString, asynParamFloat64, &this->pivotY_);
    createParam(SmarPodPivotZString, asynParamFloat64, &this->pivotZ_);
    createParam(SmarPodSetPivotString, asynParamInt32, &this->setPivot_);
    createParam(SmarpodStandbyString, asynParamInt32, &this->standby_);
    createParam(SmarpodStandbyStatusString, asynParamInt32, &this->standbyStatus_);
    createParam(SmarpodReachableStatusString, asynParamInt32, &this->reachableStatus_);
    createParam(SmarpodCSYS_XString, asynParamFloat64, &this->csysX_);
    createParam(SmarpodCSYS_YString, asynParamFloat64, &this->csysY_);
    createParam(SmarpodCSYS_ZString, asynParamFloat64, &this->csysZ_);
    createParam(SmarpodCSYS_RXString, asynParamFloat64, &this->csysRX_);
    createParam(SmarpodCSYS_RYString, asynParamFloat64, &this->csysRY_);
    createParam(SmarpodCSYS_RZString, asynParamFloat64, &this->csysRZ_);
    createParam(SmarpodSetCSYSString, asynParamInt32, &this->setCSYS_);
    createParam(SmarpodGetCSYSString, asynParamInt32, &this->getCSYS_);
    createParam(SmarpodCSYSStatusString, asynParamInt32, &this->csysStatus_);
    // 初始化轴数组
    pAxes_ = (SmarpodMotorAxis**) calloc(numAxes, sizeof(SmarpodMotorAxis*));

    // 初始化互斥锁
    mutex_ = epicsMutexCreate();
    //初始化校正状态
    setIntegerParam(motorCalibrateStatus_, 0);
    setIntegerParam(motorReferenceStatus_, 0);
    setIntegerParam(standbyStatus_, 0);
    setIntegerParam(standby_, 0);
    setIntegerParam(referenceZDirection_, 512);
    setIntegerParam(sensorMode_,1);
    // 初始化设备型号代码（固定为model_的值）
    setIntegerParam(modelCode_, model_);
    // 初始化配置的locator参数
    setDeviceStringParam(locatorParam_, locator_);
    // 更新库版本信息
    updateLibraryVersion();
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
    startPoller(movingPollPeriod, idlePollPeriod, 10);
    getCurrentCoordinateSystem();
    scanAllParameters();
}
void SmarpodMotorController::scanAllParameters() {
    printf("\n=== 扫描所有参数索引 ===\n");
    
    int index = 0;
    const int MAX_PARAMS = 100;  // 设置一个足够大的最大值
    int list = 0;  // 使用默认列表0
    
    while (index < MAX_PARAMS) {
        const char* paramName = nullptr;
        asynParamType paramType;
        
        // 获取参数名称（正确的函数调用方式）
        asynStatus status = getParamName(list, index, &paramName);
        
        if (status == asynSuccess && paramName != nullptr && strlen(paramName) > 0) {
            // 获取参数类型
            status = getParamType(list, index, &paramType);
            const char* typeStr = "Unknown";
            
            if (status == asynSuccess) {
                switch(paramType) {
                    case asynParamInt32: typeStr = "Int32"; break;
                    case asynParamFloat64: typeStr = "Float64"; break;
                    case asynParamInt32Array: typeStr = "Int32Array"; break;
                    case asynParamFloat64Array: typeStr = "Float64Array"; break;
                    case asynParamOctet: typeStr = "Octet"; break;
                    default: typeStr = "Other";
                }
            }
            
            printf("列表 %d, 索引 %2d: %-30s (类型: %s)\n", 
                   list, index, paramName, typeStr);
        }
        
        index++;
    }
    
    printf("========================\n\n");
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
    asynStatus status=asynSuccess;
    SmarpodMotorAxis *pAxis;
    int axis;
    pAxis = getAxis(pasynUser);
    if (!pAxis) return asynError;
    axis = pAxis->axisNo_;
    pAxis->setIntegerParam(function, value);
    if (function==motorCalibrate_){
        calibrateSensor();
    }
    else if (function == referenceMethod_) {
        Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_METHOD, static_cast<unsigned int>(value));
        setIntegerParam(referenceMethod_, value);
        printf("参考方法已设置为: %d\n", value);
    }
    // Z轴参考方向设置
    else if (function == referenceZDirection_) {
        printf("Z轴参考方向已设置为: 0x%X\n", value);
        Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_ZDIRECTION, static_cast<unsigned int>(value));
        setIntegerParam(referenceZDirection_, value);
        printf("Z轴参考方向已设置为: 0x%X\n", value);
    }
    // X轴参考方向设置
    else if (function == referenceXDirection_) {
        Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_XDIRECTION, static_cast<unsigned int>(value));
        setIntegerParam(referenceXDirection_, value);
        printf("X轴参考方向已设置为: 0x%X\n", value);
    }
    // Y轴参考方向设置
    else if (function == referenceYDirection_) {
        Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_YDIRECTION, static_cast<unsigned int>(value));
        setIntegerParam(referenceYDirection_, value);
        printf("Y轴参考方向已设置为: 0x%X\n", value);
    }
    // 参考频率设置
    else if (function == referenceFrequency_) {
        Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_AND_CAL_FREQUENCY, static_cast<unsigned int>(value));
        setIntegerParam(referenceFrequency_, value);
        printf("参考频率已设置为: %d Hz\n", value);
    }
    else if (function==motorReference_){
        findReferenceMarks();
    }
    else if (function==setToZero_){
        setCurrentPoseAsZero();
    }
    else if (function==moveToZero_){
        moveToZero();
    }
    else if (function == motorStop_) {
        int stopHolding;
        int holdTime;
        getIntegerParam(stopHolding_, &stopHolding);
        getIntegerParam(stopHoldingTime_, &holdTime);
        printf("stopHolding=%d,holdTime=%d\n",stopHolding,holdTime);
        if(stopHolding==0|| holdTime==0){
            status = pAxis->stop();}
        else{
            status = pAxis->stopAndHold(holdTime);}
    }
    else if (function == sensorMode_) {
        setSensorMode(static_cast<unsigned int>(value));
    }
    else if (function == standby_) {
        if (value == 1) {
            Smarpod_Status result = enterStandbyMode();
            setIntegerParam(standby_, 0);
            if (result != SMARPOD_OK) {
                printf("进入Standby模式失败: %d\n", result);
            }
        }
    }
    else if (function == setPivot_) {
        if (value == 1) {  // 当设置为1时触发
            printf("\n=== 应用虚拟旋转中心点设置 ===\n");
            setPivot(pivot_);  // 调用你已实现的setPivot函数
            // 重置触发值
            setIntegerParam(setPivot_, 0);
        }
    }
    else if (function == setCSYS_) {
    if (value == 1) {
        printf("接收到设置坐标系触发信号\n");
        setCoordinateSystem();
    }
    }
    else if (function == getCSYS_) {
        if (value == 1) {
            printf("接收到获取坐标系触发信号\n");
            getCurrentCoordinateSystem();
        }
    }
    else if (function == maxFrequency_) {
        // 处理最大频率设置
        status = setMaxFrequency(static_cast<unsigned int>(value));
        if (status == asynSuccess) {
            // 更新参数值
            setIntegerParam(maxFrequency_, value);
        }
    }
    else{
        asynMotorController::writeInt32(pasynUser, value);
    }
    callParamCallbacks();
    return status;
}
asynStatus SmarpodMotorController::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    asynStatus status = asynError;
    SmarpodMotorAxis *pAxis;
    int axis;
    pAxis = getAxis(pasynUser);
    if (!pAxis) return asynError;
    axis = pAxis->axisNo_;
    pAxis->setDoubleParam(function, value);
    //printf("axis=%d",axis);
    //SmarpodMotorAxis *pCurrentAxis；
   /* if (function == motorRecOffset_ || function == motorRecResolution_ || function == motorRecEncoderResolution_) 
    {
        // 直接返回成功，避免write error
        return asynSuccess;
    }*/
    // 处理Pivot参数
    if (function == pivotX_) {
        printf("value=%f,pivot_[0]=%f\n",value,pivot_[0]);
        pivot_[0] = value/SMARPOD_LINE_PULSES_PER_STEP;
        setDoubleParam(pivotX_, value);
        printf("设置Pivot X坐标: %.6f m\n", value);
    } 
    else if (function == pivotY_) {
        pivot_[1] = value/SMARPOD_LINE_PULSES_PER_STEP;
        setDoubleParam(pivotY_, value);
        printf("设置Pivot Y坐标: %.6f m\n", value);
    } 
    else if (function == pivotZ_) {
        pivot_[2] = value/SMARPOD_LINE_PULSES_PER_STEP;
        setDoubleParam(pivotZ_, value);
        printf("设置Pivot Z坐标: %.6f m\n", value);
    }
    else if (function == csysX_) {
        setDoubleParam(csysX_, value);
    }
    else if (function == csysY_) {
        setDoubleParam(csysY_, value);
    }
    else if (function == csysZ_) {
        setDoubleParam(csysZ_, value);
    }
    else if (function == csysRX_) {
        // 限制旋转角度范围
        if (fabs(value) <= 45.0) {
            setDoubleParam(csysRX_, value);
        } else {
            printf("RX角度=%f,超出范围[-45°, 45°]，忽略设置\n",value);
        }
    }
    else if (function == csysRY_) {
        if (fabs(value) <= 45.0) {
            setDoubleParam(csysRY_, value);
        } else {
            printf("RY角度=%f,超出范围[-45°, 45°]，忽略设置\n",value);
        }
    }
    else if (function == csysRZ_) {
        if (fabs(value) <= 45.0) {
            setDoubleParam(csysRZ_, value);
        } else {
            printf("RZ角度=%f,超出范围[-45°, 45°]，忽略设置\n",value);
        }
    }
    else{
        status = asynMotorController::writeFloat64(pasynUser, value);
    }
    callParamCallbacks();
    return status;
}
asynStatus SmarpodMotorController::readInt32(asynUser *pasynUser, epicsInt32 *value) {
    // 获取函数索引
    int function = pasynUser->reason;
    *value = 0; // 默认值
    
    // 处理校准相关
    if (function == motorCalibrate_) {
        getIntegerParam(motorCalibrate_, value);
    }
    // 处理参考方法
    else if (function == referenceMethod_) {
        getIntegerParam(referenceMethod_, value);
    }
    // 处理参考方向
    else if (function == referenceZDirection_) {
        getIntegerParam(referenceZDirection_, value);
    }
    else if (function == referenceXDirection_) {
        getIntegerParam(referenceXDirection_, value);
    }
    else if (function == referenceYDirection_) {
        getIntegerParam(referenceYDirection_, value);
    }
    // 处理参考频率
    else if (function == referenceFrequency_) {
        getIntegerParam(referenceFrequency_, value);
    }
    // 处理参考触发
    else if (function == motorReference_) {
        getIntegerParam(motorReference_, value);
    }
    // 处理归零相关
    else if (function == setToZero_) {
        getIntegerParam(setToZero_, value);
    }
    else if (function == moveToZero_) {
        getIntegerParam(moveToZero_, value);
    }
    // 处理停止相关
    else if (function == motorStop_) {
        getIntegerParam(motorStop_, value);
    }
    else if (function == stopHolding_) {
        getIntegerParam(stopHolding_, value);
    }
    else if (function == stopHoldingTime_) {
        getIntegerParam(stopHoldingTime_, value);
    }
    // 处理传感器模式
    else if (function == sensorMode_) {
        getIntegerParam(sensorMode_, value);
    }
    // 处理Standby模式
    else if (function == standby_) {
        getIntegerParam(standby_, value);
    }
    else if (function == standbyStatus_) {
        getIntegerParam(standbyStatus_, value);
    }
    // 处理Pivot设置
    else if (function == setPivot_) {
        getIntegerParam(setPivot_, value);
    }
    // 处理坐标系设置
    else if (function == setCSYS_) {
        getIntegerParam(setCSYS_, value);
    }
    else if (function == getCSYS_) {
        getIntegerParam(getCSYS_, value);
    }
    else if (function == csysStatus_) {
        getIntegerParam(csysStatus_, value);
    }
    // 处理状态参数
    else if (function == motorCalibrateStatus_) {
        getIntegerParam(motorCalibrateStatus_, value);
    }
    else if (function == motorReferenceStatus_) {
        getIntegerParam(motorReferenceStatus_, value);
    }
    else if (function == moveStatus_) {
        getIntegerParam(moveStatus_, value);
    }
    else if (function == reachableStatus_) {
        getIntegerParam(reachableStatus_, value);
    }
    else if (function == connectStatus_) {
        getIntegerParam(connectStatus_, value);
    } 
    else if (function == deviceID_) {
        getIntegerParam(deviceID_, value);
    } 
    else if (function == modelCode_) {
        getIntegerParam(modelCode_, value);
    }
    else if (function == maxFrequency_) {
        getIntegerParam(maxFrequency_, value);
    }
    // 其他参数交给父类处理
    else {
        return asynMotorController::readInt32(pasynUser, value);
    }
    
    return asynSuccess;
}
asynStatus SmarpodMotorController::readFloat64(asynUser *pasynUser, epicsFloat64 *value) {
    // 获取函数索引
    int function = pasynUser->reason;
    *value = 0.0; // 默认值
    // 处理Pivot参数
    if (function == pivotX_) {
        getDoubleParam(pivotX_, value);
    }
    else if (function == pivotY_) {
        getDoubleParam(pivotY_, value);
    }
    else if (function == pivotZ_) {
        getDoubleParam(pivotZ_, value);
    }
    // 处理坐标系参数
    else if (function == csysX_) {
        getDoubleParam(csysX_, value);
    }
    else if (function == csysY_) {
        getDoubleParam(csysY_, value);
    }
    else if (function == csysZ_) {
        getDoubleParam(csysZ_, value);
    }
    else if (function == csysRX_) {
        getDoubleParam(csysRX_, value);
    }
    else if (function == csysRY_) {
        getDoubleParam(csysRY_, value);
    }
    else if (function == csysRZ_) {
        getDoubleParam(csysRZ_, value);
        printf("\n\ncsysRZ_=%f\n\n",value);
    }
    // 电机相关参数
    else if (function == motorPosition_) {
        getDoubleParam(motorPosition_, value);
    }
    else if (function == motorVelocity_) {
        getDoubleParam(motorVelocity_, value);
    }
    else if (function == motorAccel_) {
        getDoubleParam(motorAccel_, value);
    }
    else if (function == motorEncoderPosition_) {
        getDoubleParam(motorEncoderPosition_, value);
    }
    else {
        return asynMotorController::readFloat64(pasynUser, value);
    }
    return asynSuccess;
}
asynStatus SmarpodMotorController::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason) 
{
    int function = pasynUser->reason;
    *nActual = 0;
    *eomReason = ASYN_EOM_EOS;
    
    // 处理所有字符串类型的设备信息参数
    if (function == scanDevices_ || function == libraryVersion_ || 
        function == locatorParam_ || function == modelName_ || 
        function == systemLocator_) {
        
        // 使用asynPortDriver的getStringParam方法获取值
        char buffer[4096];
        this->getStringParam(function, sizeof(buffer), buffer);
        strncpy(value, buffer, maxChars);
        *nActual = strlen(value);
        
        // 确保字符串以'\0'结尾
        if (*nActual >= maxChars) {
            *nActual = maxChars - 1;
            value[*nActual] = '\0';
        }
        return asynSuccess;
    }
    
    // 其他字符串参数交给父类处理
    return asynMotorController::readOctet(pasynUser, value, maxChars, nActual, eomReason);
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
    if (isConnected_) {
        getSensorMode(&sensorMode);
        unsigned int freq;
        getMaxFrequency(&freq);
        setIntegerParam(maxFrequency_, freq);
    }
    //configureSystem();
    // 检查参考状态
    printf("参考状态: %s\n", isReferenced() ? "已参考" : "未参考");
    if (!isReferenced()) {
        //findReferenceMarks();
    }

    // 获取当前姿态
    Smarpod_Pose pose;
    if(Smarpod_GetPose(smarpodID_, &pose) == SMARPOD_OK) {
        printf("当前姿态:\n");
        printf("  位置: X=%.6f mm, Y=%.6f mm, Z=%.6f mm\n", 
               pose.positionX*1000, pose.positionY*1000, pose.positionZ*1000);
        printf("  旋转: Rx=%.3f°, Ry=%.3f°, Rz=%.3f°\n", 
               pose.rotationX, pose.rotationY, pose.rotationZ);
    }
    unsigned int method, zDir, xDir, yDir, freq;
    Smarpod_Get_ui(smarpodID_, SMARPOD_FREF_METHOD, &method);
    Smarpod_Get_ui(smarpodID_, SMARPOD_FREF_ZDIRECTION, &zDir);
    Smarpod_Get_ui(smarpodID_, SMARPOD_FREF_XDIRECTION, &xDir);
    Smarpod_Get_ui(smarpodID_, SMARPOD_FREF_YDIRECTION, &yDir);
    Smarpod_Get_ui(smarpodID_, SMARPOD_FREF_AND_CAL_FREQUENCY, &freq);
    // 更新驱动参数并同步到PV
    setIntegerParam(referenceMethod_, method);
    setIntegerParam(referenceZDirection_, zDir);
    setIntegerParam(referenceXDirection_, xDir);
    setIntegerParam(referenceYDirection_, yDir);
    setIntegerParam(referenceFrequency_, freq);
    printf("\n=== 执行参考点查找使用的参数 ===\n");
    printf("参考方法: %d\n", method);
    printf("Z轴参考方向: %d (0x%X)\n", zDir, zDir);
    printf("X轴参考方向: %d (0x%X)\n", xDir, xDir);
    printf("Y轴参考方向: %d (0x%X)\n", yDir, yDir);
    printf("参考频率: %d Hz\n", freq);
    printf("================================\n\n");
    double currentPivot[3];
    getPivot(currentPivot);
    callParamCallbacks();
    epicsMutexUnlock(mutex_);
    return isConnected_ ? asynSuccess : asynError;
}
asynStatus SmarpodMotorController::checkDeviceStatus() {
    printf("\n===================================== Smarpod设备详细信息 =====================================\n");
    // 1. 扫描到的设备列表
    char systems_buffer[4096] = {0};
    unsigned int buffer_size = sizeof(systems_buffer);
    Smarpod_FindSystems("", systems_buffer, &buffer_size);
    printf("📡 扫描到的设备: %s\n", systems_buffer);
    setDeviceStringParam(scanDevices_, systems_buffer);  // 保存到PV
    
    // 2. Smarpod库版本（已通过updateLibraryVersion更新，此处仅打印）
    char libraryVersion[256];
    // 调用原型：getStringParam(int index, int maxChars, char *value)
    asynStatus status = getStringParam(
        libraryVersion_,                           // 参数索引（int）
        static_cast<int>(sizeof(libraryVersion)),  // 缓冲区最大长度（int）
        libraryVersion                             // 结果缓冲区（char*）
    );
    printf("🔧 Smarpod库版本号: %s\n", libraryVersion);
    
    // 3. 打开设备连接
    Smarpod_Status st = Smarpod_Open(&smarpodID_, model_, locator_, "");
    if(st == SMARPOD_OK) {
        printf("✅ 设备连接状态: 已连接\n");
        isConnected_ = true;
        setIntegerParam(connectStatus_, 1);  // 连接状态：1=已连接
        setIntegerParam(deviceID_, smarpodID_);  // 设备ID
        printf("🆔 Smarpod设备ID: %u\n", smarpodID_);
        // 6. Smarpod_GetSystemLocator返回值
        char systemLocator[4096] = {0};
        buffer_size = sizeof(systemLocator);
        if(Smarpod_GetSystemLocator(smarpodID_, systemLocator, &buffer_size) == SMARPOD_OK) {
            setDeviceStringParam(systemLocator_, systemLocator);
            printf("📍 Smarpod_GetSystemLocator返回值: %s (长度: %u字节)\n", systemLocator, buffer_size);
        } else {
            setDeviceStringParam(systemLocator_, "获取失败");
            printf("📍 Smarpod_GetSystemLocator返回值: 获取失败\n");
        }
        
    } else {
        printf("❌ 设备连接状态: 断开（错误码: %d）\n", st);
        isConnected_ = false;
        setIntegerParam(connectStatus_, 0);  // 连接状态：0=断开
        setIntegerParam(deviceID_, 0);
        setDeviceStringParam(systemLocator_, "未连接");
        printf("🆔 Smarpod设备ID: 0（未连接）\n");
    }
    
    // 7. 配置的locator_参数
    printf("🔗 配置的locator参数: %s\n", locator_);
    setDeviceStringParam(locatorParam_, locator_);  // 保存到PV
    
    // 8. 设备型号名称（如P-CLL:10115）
    const char *modelName = "未知型号";
    if(Smarpod_GetModelName(model_, &modelName) == SMARPOD_OK) {
        char fullModelName[64];
        sprintf(fullModelName, "%s:%u", modelName, model_);  // 格式：P-CLL:10115
        setDeviceStringParam(modelName_, fullModelName);
        printf("🏷️  设备型号名称: %s\n", fullModelName);
    } else {
        setDeviceStringParam(modelName_, "获取失败");
        printf("🏷️  设备型号名称: 获取失败\n");
    }
    unsigned int freq;
    if (getMaxFrequency(&freq) == asynSuccess) {
        setIntegerParam(maxFrequency_, freq);
        callParamCallbacks();
    }
    // 9. 设备型号代码
    printf("🆎 设备型号代码: %u\n", model_);
    setIntegerParam(modelCode_, model_);  // 保存到PV
    
    printf("========================================================================================\n\n");
    
    // 触发参数更新，同步到PV
    callParamCallbacks();
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
    printf("ni yi jing jin ru moving ！\n");
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
    printf("\ncurrentPose: X=%.6f m, Y=%.6f m, Z=%.6f m，Rotation: Rx=%.6f°, Ry=%.6f°, Rz=%.6f°\n\n",currentPose.positionX, currentPose.positionY, currentPose.positionZ,currentPose.rotationX, currentPose.rotationY, currentPose.rotationZ);
        unsigned int moveStatus;
        if (Smarpod_GetMoveStatus(pC_->smarpodID_, &moveStatus) == SMARPOD_OK) {
            int isMoving = (moveStatus == SMARPOD_MOVING) ? 1 : 0;
            if (isMoving == 0) {
                targetPose = currentPose;
                if(axisNo_<3) {
                    position=position/SMARPOD_LINE_PULSES_PER_STEP;
                }
                else{
                    position=position/SMARPOD_ROTATION_PULSES_PER_STEP;
                }
                if (relative == 0) {
                    // 绝对运动
                    printf("ni yi jing jin 绝对运动 ！\n");
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
                    printf("ni yi jing jin 相对运动 ！\n");
                    switch(axisNo_) {
                        case 0: targetPose.positionX += position; break;
                        case 1: targetPose.positionY += position; break;
                        case 2: targetPose.positionZ += position; break;
                        case 3: targetPose.rotationX += position; break;
                        case 4: targetPose.rotationY += position; break;
                        case 5: targetPose.rotationZ += position; break;
                    }
                }
                printf("\ntargetPose 2: X=%.6f m, Y=%.6f m, Z=%.6f m，Rotation: Rx=%.6f°, Ry=%.6f°, Rz=%.6f°\n\n",targetPose.positionX, targetPose.positionY, targetPose.positionZ,targetPose.rotationX, targetPose.rotationY, targetPose.rotationZ);
                int reachable;
                result = Smarpod_IsPoseReachable(pC_->smarpodID_, &targetPose, &reachable);
                setIntegerParam(pC_->reachableStatus_, !reachable);
                printf("===========reachable=%d\n",reachable);
                if (result != SMARPOD_OK || !reachable) {
                    printf("The result of the test on reachability is error\n");
                    status = asynError;
                } else {
                    result = Smarpod_Move(pC_->smarpodID_, &targetPose, SMARPOD_HOLDTIME_INFINITE, 0);
                    if (result != SMARPOD_OK) {
                        printf("Smarpod_Move failed with error code: %d\n", result);
                        status = asynError;
                    } else {
                        printf("Axis %d move command sent successfully\n", axisNo_);
                        //setIntegerParam(pC_->motorStatusDone_, 0);
                    }
                }
                callParamCallbacks();
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
    pC_->wakeupPoller(); 
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
    int findReference;
    getIntegerParam(motorReferenceStatus_, &findReference);
    printf("findReference  1  ==%d\n",findReference);
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法查找参考点！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    setIntegerParam(motorReferenceStatus_, 2);
    callParamCallbacks();
    // 获取当前配置的参数
    int method, zDir, xDir, yDir, freq;
    getIntegerParam(referenceMethod_, &method);
    getIntegerParam(referenceZDirection_, &zDir);
    getIntegerParam(referenceXDirection_, &xDir);
    getIntegerParam(referenceYDirection_, &yDir);
    getIntegerParam(referenceFrequency_, &freq);
    // 应用配置参数
    Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_METHOD, method);
    Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_ZDIRECTION, zDir);
    Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_XDIRECTION, xDir);
    Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_YDIRECTION, yDir);
    Smarpod_Set_ui(smarpodID_, SMARPOD_FREF_AND_CAL_FREQUENCY, freq);
    getIntegerParam(motorReferenceStatus_, &findReference);
    printf("findReference  2  ==%d\n",findReference);
    printf("开始查找参考点（设备ID: %u）...\n", smarpodID_);
    Smarpod_Status st = Smarpod_FindReferenceMarks(smarpodID_);
    if (st == SMARPOD_OK) {
        printf("✅ 参考点查找成功！\n");
        isReferenced();
    getIntegerParam(motorReferenceStatus_, &findReference);
    printf("findReference  3  ==%d\n",findReference);
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
    setIntegerParam(motorReferenceStatus_, referenced);
    callParamCallbacks();
    epicsMutexUnlock(mutex_);
    if (st == SMARPOD_OK && referenced == SMARPOD_TRUE) {
        return true;
    } else {
        return false;
    }
}
// 设置当前姿态设为零
asynStatus SmarpodMotorController::setCurrentPoseAsZero() {
    asynStatus status;
    epicsMutexLock(mutex_);
    if (!isConnected_) {
        printf("❌ 设备未连接，无法设置当前姿态为零！\n");
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    Smarpod_Pose csys;
    Smarpod_GetPose(smarpodID_, &csys);
    printf("设置当前姿态为坐标原点...\n");
    Smarpod_Status st = Smarpod_SetCurrentPoseAsZero(smarpodID_);
    if (st == SMARPOD_OK) {
        // 重置本地存储
        //memset(&currentCS_, 0, sizeof(Smarpod_Pose));
        
        setDoubleParam(csysX_, csys.positionX*1000);
        setDoubleParam(csysY_, csys.positionY*1000);
        setDoubleParam(csysZ_, csys.positionZ*1000);
        setDoubleParam(csysRX_, csys.rotationX);
        setDoubleParam(csysRY_, csys.rotationY);
        setDoubleParam(csysRZ_, csys.rotationZ);
        // 更新PV
        printf("✅ 当前姿态已设为坐标原点\n");
        callParamCallbacks();
            double csysX, csysY, csysZ, csysRX, csysRY, csysRZ;
        getDoubleParam(csysX_, &csysX);
        getDoubleParam(csysY_, &csysY);
        getDoubleParam(csysZ_, &csysZ);
        getDoubleParam(csysRX_, &csysRX);
        getDoubleParam(csysRY_, &csysRY);
        getDoubleParam(csysRZ_, &csysRZ);
        printf("坐标系参数: X=%.6f, Y=%.6f, Z=%.6f, RX=%.3f, RY=%.3f, RZ=%.3f\n",
               csysX, csysY, csysZ, csysRX, csysRY, csysRZ);
        epicsMutexUnlock(mutex_);
        status= asynSuccess;
    } else {
        printf("❌ 设置当前姿态为零失败，错误码: %d\n", st);
        epicsMutexUnlock(mutex_);
        status= asynError;
    }
    return status;
}
//移动到零位
asynStatus SmarpodMotorController::moveToZero() {
    epicsMutexLock(mutex_);
    asynStatus status = asynSuccess;
    static const char *functionName = "moveToZero";
    Smarpod_Pose currentPose;
    Smarpod_Pose targetPose;  
    Smarpod_Status result;  
    Smarpod_SetAcceleration(smarpodID_,SMARPOD_TRUE,1);
    Smarpod_SetSpeed(smarpodID_,SMARPOD_TRUE,1);
    if (Smarpod_GetPose(smarpodID_, &currentPose) == SMARPOD_OK) {
        unsigned int moveStatus;
        if (Smarpod_GetMoveStatus(smarpodID_, &moveStatus) == SMARPOD_OK) {
            int isMoving = (moveStatus == SMARPOD_MOVING) ? 1 : 0;
            if (isMoving == 0) {
                targetPose.positionX = 0.0;
                targetPose.positionY = 0.0;  
                targetPose.positionZ = 0.0;
                targetPose.rotationX = 0.0;
                targetPose.rotationY = 0.0;
                targetPose.rotationZ = 0.0;
                int reachable;
                result = Smarpod_IsPoseReachable(smarpodID_, &targetPose, &reachable);
                setIntegerParam(reachableStatus_, reachable);
                if (result != SMARPOD_OK || !reachable) {
                    printf("The result of the test on reachability is error\n");
                    status = asynError;
                } else {
                    result = Smarpod_Move(smarpodID_, &targetPose, SMARPOD_HOLDTIME_INFINITE, 0);
                    if (result != SMARPOD_OK) {
                        printf("Smarpod_Move failed with error code: %d\n", result);
                        status = asynError;
                    } else {
                        printf("moveToZero command sent successfully\n");
                        callParamCallbacks();
                    }
                }
            }
        } else {
            printf("Failed to get move status\n");
            status = asynError;
        }
    } else {
        printf("Failed to get current pose\n");
        status = asynError;
    }
    wakeupPoller(); 
    epicsMutexUnlock(mutex_);
    return status;
}
asynStatus SmarpodMotorAxis::stop()
{
    printf("轴%d: 执行停止运动操作！\n", axisNo_);
    epicsMutexLock(pC_->mutex_);
    asynStatus status = asynSuccess;
    static const char *functionName = "stop";
    // 检查设备是否已连接
    if (!pC_->isConnected_) {
        printf("轴%d: 设备未连接，无法执行停止操作\n", axisNo_);
        status = asynError;
        epicsMutexUnlock(pC_->mutex_);
        return status;
    }
    // 获取当前运动状态
    unsigned int moveStatus;
    if (Smarpod_GetMoveStatus(pC_->smarpodID_, &moveStatus) == SMARPOD_OK) {
            // 执行停止操作
            Smarpod_Status result = Smarpod_Stop(pC_->smarpodID_);
            if (result == SMARPOD_OK) {
                printf("轴%d: 停止指令发送成功\n", axisNo_);
                // 更新状态参数
                pC_->setIntegerParam(axisNo_, pC_->motorStatusMoving_, 0);
                pC_->setIntegerParam(axisNo_, pC_->motorStatusDone_, 1);
                pC_->callParamCallbacks(axisNo_);
                // 立即更新当前位置
                axesCurrentPosition();
            } 
            else if (result == SMARPOD_STOPPED_ERROR) {
                printf("轴%d: 停止失败 - 设备正在执行参考点查找或校准操作\n", axisNo_);
                status = asynError;
            } 
            else {
                printf("轴%d: 停止操作失败，错误码: %d\n", axisNo_, result);
                status = asynError;
            } 
    } 
    else {
        printf("轴%d: 获取运动状态失败\n", axisNo_);
        status = asynError;
    }
    
    pC_->wakeupPoller();
    epicsMutexUnlock(pC_->mutex_);
    return status;
}

/**
 * 停止并保持位置
 * 对应API：Smarpod_StopAndHold
 * @param holdTime 保持时间（毫秒），默认无限保持
 */
asynStatus SmarpodMotorAxis::stopAndHold(unsigned int holdTime)
{
    printf("轴%d: 执行停止并保持位置操作（保持时间: %u ms）\n", axisNo_, holdTime);
    epicsMutexLock(pC_->mutex_);
    asynStatus status = asynSuccess;
    static const char *functionName = "stopAndHold";
    // 检查设备是否已连接
    if (!pC_->isConnected_) {
        printf("轴%d: 设备未连接，无法执行停止保持操作\n", axisNo_);
        status = asynError;
        epicsMutexUnlock(pC_->mutex_);
        return status;
    }
    // 验证保持时间范围（0-60000ms或无限）
    if (holdTime <0 || (holdTime > 60000)) {
        printf("轴%d: 保持时间超出范围（0-60000ms），使用默认值\n", axisNo_);
        holdTime = SMARPOD_HOLDTIME_INFINITE;
    }
    // 获取当前运动状态
    unsigned int moveStatus;
    if (Smarpod_GetMoveStatus(pC_->smarpodID_, &moveStatus) == SMARPOD_OK) {
            // 执行停止并保持操作
            Smarpod_Status result = Smarpod_StopAndHold(pC_->smarpodID_, holdTime);
            if (result == SMARPOD_OK) {
                printf("轴%d: 停止并保持指令发送成功\n", axisNo_);
                
                // 更新状态参数
                pC_->setIntegerParam(axisNo_, pC_->motorStatusMoving_, 0);
                pC_->setIntegerParam(axisNo_, pC_->motorStatusDone_, 1);
                pC_->callParamCallbacks(axisNo_);
                
                // 立即更新当前位置
                axesCurrentPosition();
            } 
            else if (result == SMARPOD_STOPPED_ERROR) {
                printf("轴%d: 停止保持失败 - 设备正在执行参考点查找或校准操作\n", axisNo_);
                status = asynError;
            } 
            else {
                printf("轴%d: 停止保持操作失败，错误码: %d\n", axisNo_, result);
                status = asynError;
            }
    } 
    else {
        printf("轴%d: 获取运动状态失败\n", axisNo_);
        status = asynError;
    }
    pC_->wakeupPoller();
    epicsMutexUnlock(pC_->mutex_);
    return status;
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
            setDoubleParam(pivotX_, verifyPivot[0]);
            setDoubleParam(pivotY_, verifyPivot[1]);
            setDoubleParam(pivotZ_, verifyPivot[2]);
            callParamCallbacks();
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
    for(int i=0;i<3;i++)
        pivot[i] *=SMARPOD_LINE_PULSES_PER_STEP;
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
// 进入Standby模式
asynStatus SmarpodMotorController::enterStandbyMode() {
    // 检查当前状态是否允许进入Standby
    unsigned int moveStatus;
    Smarpod_Status result = Smarpod_GetMoveStatus(smarpodID_, &moveStatus);
    if (result != SMARPOD_OK) {
        printf("获取移动状态失败: %d\n", result);
        return (result == SMARPOD_OK) ? asynSuccess : asynError;
    }
    // 检查是否在参考或校准中
    if (moveStatus == SMARPOD_MOVING || moveStatus == SMARPOD_REFERENCING || moveStatus == SMARPOD_CALIBRATING) {
        printf("当前状态不允许进入Standby模式（参考或校准中）\n");
        return asynError;
    }
    // 设置切换中状态
    setIntegerParam(standbyStatus_, 2);
    callParamCallbacks();
    // 执行Standby命令
    result = Smarpod_Standby(smarpodID_);
    if (result == SMARPOD_OK) {
        printf("进入Standby模式命令已发送，等待状态切换...\n");
        // 等待状态切换完成（150ms）
        epicsThreadSleep(0.15);
        // 检查最终状态
        result = Smarpod_GetMoveStatus(smarpodID_, &moveStatus);
        if (result == SMARPOD_OK && moveStatus == SMARPOD_STANDBY) {
            setIntegerParam(standbyStatus_, 1);
            printf("成功进入Standby模式\n");
        } else {
            printf("Standby模式切换失败，当前状态: %d\n", moveStatus);
            setIntegerParam(standbyStatus_, 0);
            result = SMARPOD_BUSY_ERROR;
        }
    } else {
        printf("进入Standby模式失败: %d\n", result);
        setIntegerParam(standbyStatus_, 0);
    }
    callParamCallbacks();
    return (result == SMARPOD_OK) ? asynSuccess : asynError;
}
// 设置坐标系
asynStatus SmarpodMotorController::setCoordinateSystem() {
    epicsMutexLock(mutex_);
    double csysX, csysY, csysZ, csysRX, csysRY, csysRZ;
    getDoubleParam(csysX_, &csysX);
    getDoubleParam(csysY_, &csysY);
    getDoubleParam(csysZ_, &csysZ);
    getDoubleParam(csysRX_, &csysRX);
    getDoubleParam(csysRY_, &csysRY);
    getDoubleParam(csysRZ_, &csysRZ);

    // 创建坐标系参数结构体
    Smarpod_Pose csys;
    csys.positionX = csysX/1000;
    csys.positionY = csysY/1000;
    csys.positionZ = csysZ/1000;
    csys.rotationX = csysRX;
    csys.rotationY = csysRY;
    csys.rotationZ = csysRZ;
    // 更新状态
    setIntegerParam(csysStatus_, 1);
    callParamCallbacks();
    printf("开始设置坐标系参数...\n");
    printf("坐标系参数: X=%.6f, Y=%.6f, Z=%.6f, RX=%.3f, RY=%.3f, RZ=%.3f\n",
           csysX, csysY, csysZ, csysRX, csysRY, csysRZ);
    // 检查旋转角度范围 [-45°, 45°]
    if (fabs(csysRX) > 45.0 || fabs(csysRY) > 45.0 || fabs(csysRZ) > 45.0) {
        printf("错误：旋转角度超出范围[-45°, 45°]！\n");
        setIntegerParam(csysStatus_, -1);
        setIntegerParam(setCSYS_, 0);
        callParamCallbacks();
        epicsMutexUnlock(mutex_);
        return asynError;
    }

    // 执行设置命令
    Smarpod_Status result = Smarpod_SetCoordinateSystem(smarpodID_, &csys);
    if (result == SMARPOD_OK) {
        printf("坐标系设置成功！\n");
        setIntegerParam(csysStatus_, 0);
        // 设置成功后立即获取一次确认
        getCurrentCoordinateSystem();
    } else {
        printf("坐标系设置失败，错误码: %d\n", result);
        setIntegerParam(csysStatus_, -1);
        getCurrentCoordinateSystem();
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    // 复位触发信号
    setIntegerParam(setCSYS_, 0);
    callParamCallbacks();
    epicsMutexUnlock(mutex_);
    return asynSuccess;
}

// 获取当前坐标系
asynStatus SmarpodMotorController::getCurrentCoordinateSystem() {
    printf("获取当前坐标系参数...\n");
    Smarpod_Pose csys;
    Smarpod_Status result = Smarpod_GetCoordinateSystem(smarpodID_, &csys);
    if (result == SMARPOD_OK) {
        epicsMutexLock(mutex_);
        // 更新PV值

        setDoubleParam(csysX_, csys.positionX*1000);
        setDoubleParam(csysY_, csys.positionY*1000);
        setDoubleParam(csysZ_, csys.positionZ*1000);
        setDoubleParam(csysRX_, csys.rotationX);
        setDoubleParam(csysRY_, csys.rotationY);
        setDoubleParam(csysRZ_, csys.rotationZ);
        printf("坐标系参数获取成功：\n");
        printf("  X=%.6f m, Y=%.6f m, Z=%.6f m\n", csys.positionX, csys.positionY, csys.positionZ);
        printf("  RX=%.6f°, RY=%.6f°, RZ=%.6f°\n", csys.rotationX, csys.rotationY, csys.rotationZ);
        
        setIntegerParam(csysStatus_, 0);
        setIntegerParam(getCSYS_, 0);
        callParamCallbacks();
        epicsMutexUnlock(mutex_);
    } else {
        printf("坐标系参数获取失败，错误码: %d\n", result);
        epicsMutexLock(mutex_);
        setIntegerParam(csysStatus_, -1);
        setIntegerParam(getCSYS_, 0);
        callParamCallbacks();
        epicsMutexUnlock(mutex_);
        return asynError;
    }
    return asynSuccess;
}
// 辅助方法：设置字符串参数（使用正确的asynPortDriver API）
asynStatus SmarpodMotorController::setDeviceStringParam(int function, const char* value) {
    epicsMutexLock(mutex_);
    asynStatus status = asynSuccess;
    
    if (value && strlen(value) > 0) {
        status = this->setStringParam(function, value);
    } else {
        status = this->setStringParam(function, "N/A");
    }
    
    epicsMutexUnlock(mutex_);
    return status;
}
// 辅助方法：更新Smarpod库版本信息
void SmarpodMotorController::updateLibraryVersion() {
    unsigned int major, minor, update;
    Smarpod_GetDLLVersion(&major, &minor, &update);
    char versionStr[32];
    sprintf(versionStr, "%u.%u.%u", major, minor, update);
    // 使用正确的方法设置字符串参数
    setDeviceStringParam(libraryVersion_, versionStr);
    printf("🔧 Smarpod库版本: %s\n", versionStr);
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

/*asynMotorAxis* SmarpodMotorController::getAxis(int axisNo) {
    if (axisNo >= 0 && axisNo < numAxes_ && pAxes_[axisNo] != NULL) {
        return pAxes_[axisNo];
    }
    return nullptr;
}*/
/** Returns a pointer to a SmarpodMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SmarpodMotorAxis* SmarpodMotorController::getAxis(asynUser *pasynUser)
{
  return static_cast<SmarpodMotorAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to a SmarpodMotorAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
SmarpodMotorAxis* SmarpodMotorController::getAxis(int axisNo)
{
  return static_cast<SmarpodMotorAxis*>(asynMotorController::getAxis(axisNo));
}

void SmarpodMotorAxis::updateAllAxes() {
    if (!pC_->isConnected_) return;
    //printf("updateAllAxes   moveStatus\n！");
    // 2. 获取运动状态
    unsigned int moveStatus;
    bool isMoving = false;
    
    if (Smarpod_GetMoveStatus(pC_->smarpodID_, &moveStatus) == SMARPOD_OK) {
        isMoving = (moveStatus == SMARPOD_MOVING || 
                   moveStatus == SMARPOD_CALIBRATING || 
                   moveStatus == SMARPOD_REFERENCING);
        
        // 使用控制器的参数索引（通过pC_访问）
         setIntegerParam(pC_->motorStatusDone_, !isMoving);
         setIntegerParam(pC_->motorStatusMoving_, isMoving);
         callParamCallbacks();
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
    pC_->callParamCallbacks(axisNo_);
}

// 修正位置更新方法（使用控制器的参数）
void SmarpodMotorAxis::axesCurrentPosition() {
    Smarpod_Pose currentPose;
    if (Smarpod_GetPose(pC_->smarpodID_, &currentPose) != SMARPOD_OK) {
        printf("获取失败！");
        return;
    }
    static int count = 0;
    count++;
    if(count>=65535)count=0;
    double position = 0.0;
    switch(axisNo_) {
        case 0: position = currentPose.positionX * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 1: position = currentPose.positionY * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 2: position = currentPose.positionZ * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 3: position = currentPose.rotationX * SMARPOD_ROTATION_PULSES_PER_STEP; break;
        case 4: position = currentPose.rotationY * SMARPOD_ROTATION_PULSES_PER_STEP; break;
        case 5: position = currentPose.rotationZ * SMARPOD_ROTATION_PULSES_PER_STEP; break;
    }
      // 获取各项参数值用于打印
    double motorPosition, motorEncoderPosition;
    double motorRecResolution, motorRecEncoderResolution, motorRecOffset;
    // 使用控制器的参数索引更新当前轴位置
    setDoubleParam(pC_->motorPosition_, position);
    setDoubleParam(pC_->motorEncoderPosition_, position);
    callParamCallbacks();
    pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &motorPosition);
    pC_->getDoubleParam(axisNo_, pC_->motorEncoderPosition_, &motorEncoderPosition);
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &motorRecResolution);
    pC_->getDoubleParam(axisNo_, pC_->motorRecEncoderResolution_, &motorRecEncoderResolution);
    pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &motorRecOffset);
    /*printf("\ncount=%d，currentPose: X=%.6f m, Y=%.6f m, Z=%.6f m，Rotation: Rx=%.6f°, Ry=%.6f°, Rz=%.6f°\n\n",count,currentPose.positionX, currentPose.positionY, currentPose.positionZ,currentPose.rotationX, currentPose.rotationY, currentPose.rotationZ);*/
}
//同步反馈值给目标值     
/*void SmarpodMotorAxis::axesCurrentPosition() {
    Smarpod_Pose currentPose;
    if (Smarpod_GetPose(pC_->smarpodID_, &currentPose) != SMARPOD_OK) {
        printf("获取失败！");
        return;
    }
    static int count = 0;
    count++;
    if(count>=65535)count=0;
    double position = 0.0;
    switch(axisNo_) {
        case 0: position = currentPose.positionX * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 1: position = currentPose.positionY * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 2: position = currentPose.positionZ * SMARPOD_LINE_PULSES_PER_STEP; break;
        case 3: position = currentPose.rotationX * SMARPOD_ROTATION_PULSES_PER_STEP; break;
        case 4: position = currentPose.rotationY * SMARPOD_ROTATION_PULSES_PER_STEP; break;
        case 5: position = currentPose.rotationZ * SMARPOD_ROTATION_PULSES_PER_STEP; break;
    }
      // 获取各项参数值用于打印
    double motorPosition, motorEncoderPosition;
    double motorRecResolution, motorRecEncoderResolution, motorRecOffset;
    // 使用控制器的参数索引更新当前轴位置
    setDoubleParam(pC_->motorPosition_, position);
    setDoubleParam(pC_->motorEncoderPosition_, position);
    callParamCallbacks();
    pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &motorPosition);
    pC_->getDoubleParam(axisNo_, pC_->motorEncoderPosition_, &motorEncoderPosition);
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &motorRecResolution);
    pC_->getDoubleParam(axisNo_, pC_->motorRecEncoderResolution_, &motorRecEncoderResolution);
    pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &motorRecOffset);
    /*printf("\ncount=%d，currentPose: X=%.6f m, Y=%.6f m, Z=%.6f m，Rotation: Rx=%.6f°, Ry=%.6f°, Rz=%.6f°\n\n",count,currentPose.positionX, currentPose.positionY, currentPose.positionZ,currentPose.rotationX, currentPose.rotationY, currentPose.rotationZ);*/
//}
asynStatus SmarpodMotorController::poll()
{
    
    static int count = 0;
    count++;
    for (int i = 0; i < numAxes_; i++) {
        if (pAxes_[i]) {
            pAxes_[i]->axesCurrentPosition();  // 更新位置
            pAxes_[i]->updateAllAxes();        // 更新状态
        }
    }
    return asynSuccess;
}
asynStatus SmarpodMotorAxis::poll(bool *moving) {
    // 更新当前轴的位置
    /*updateAllAxes();
    axesCurrentPosition();*/
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

