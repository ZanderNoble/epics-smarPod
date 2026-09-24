#include "registryFunction.h"
#include "epicsExport.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <SmarPod.h>
#include <epicsMutex.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* This is the same for lin and rot positioners
 * lin: controller cm --> driver nm. Because of this the user can use the positioner for cm ranges
 * rot: controller deg --> driver udeg. Because of this the user can use the positioner for deg ranges*/
#define SMARPOD_LINE_PULSES_PER_STEP        1000000000
#define SMARPOD_ROTATION_PULSES_PER_STEP    1000000
#define SMARPOD_MODEL_10115 10115
#define SMARPOD_TIMEOUT 5000
#define SMARPOD_FALSE 0
#define SMARPOD_TRUE  1

#define SmarPodMotorMoveStatusString           "MOVE_STATUS"
#define SmarPodMotorCalibratingString          "CAL"
#define SmarPodMotorStatusCalibrateString      "CALIBRATE_STATUS"
#define SmarPodMotorReferencingString          "REFERENCE"
#define SmarPodMotorStatusReferenceString      "REFERENCE_STATUS"
#define SmarPodMoveToZeroString                "MOVE_TO_ZERO"
class SmarpodMotorAxis : public asynMotorAxis {
public:
    SmarpodMotorAxis(class SmarpodMotorController *pC, int axisNo);
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
    asynStatus poll(bool*moving) override;
    void updateAllAxes();
    void axesCurrentPosition();
protected:
    SmarpodMotorController *pC_;
    int axisNo_;  // 直接保存轴号
    
private:
    friend class SmarpodMotorController;
};

class epicsShareClass SmarpodMotorController : public asynMotorController {
public:
    SmarpodMotorController(const char* portName, const char* locator, 
                          int numAxes, double movingPollPeriod, double idlePollPeriod,
                          unsigned int model=SMARPOD_MODEL_10115,int unusedMask=0);
    asynStatus configureController(); // 配置控制器（离线配置，无需打开设备）
    asynStatus configureSystem();// 配置系统（在线配置，需已打开设备）
    asynStatus connect();
    asynStatus disconnect();
    asynStatus checkDeviceStatus();
    asynStatus calibrateSensor();// 传感器校准
    asynStatus findReferenceMarks();// 查找参考点（新增独立函数）
    bool isReferenced();// 检查参考状态
    asynStatus setSensorMode(unsigned int mode);    // 设置传感器模式
    asynStatus getSensorMode(unsigned int* mode);    // 获取当前传感器模式
    asynStatus setMaxFrequency(unsigned int frequency);// 设置最大频率
    asynStatus getMaxFrequency(unsigned int* frequency);// 获取最大频率 
    asynStatus setPivot(const double pivot[3]);// 设置虚拟旋转中心点
    asynStatus getPivot(double pivot[3]);// 获取虚拟旋转中心点
    asynStatus setCoordinateSystem(const Smarpod_Pose* csys);
    asynStatus getCoordinateSystem(Smarpod_Pose* csys);
    asynStatus setCurrentPoseAsZero();
    asynStatus setAxesOrientation(double rx, double ry, double rz);
    asynStatus getAxesOrientation(double* rx, double* ry, double* rz);
    asynStatus logSmarpodError(Smarpod_Status status, const char* context);
    asynMotorAxis* getAxis(int axisNo) override;
    asynStatus PrintStatus(const char* operation, Smarpod_Status status);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    void updateCoordinateSystemPV();
    void updateAxesOrientationPV();
    int getAxisIndex(asynUser *pasynUser);  // 添加这个声明
    asynStatus poll() override;
protected:
    SmarpodMotorAxis **pAxes_;  
    unsigned int smarpodID_;
    unsigned int model_;
    char locator_[256];
    bool isConnected_;
    epicsMutexId mutex_;
    int PivotX_Set_;
    int PivotY_Set_;
    int PivotZ_Set_;
    int PivotX_Get_;
    int PivotY_Get_;
    int PivotZ_Get_;
    // 用于存储当前旋转中心值的成员变量
    double currentPivot_[3];
    int moveStatus_;
    int motorCalibrate_;
    int motorReference_;
    int moveToZero_;
    int motorCalibrateStatus_;
    int motorReferenceStatus_;
    // PV参数索引
    int CS_X_Set_;
    int CS_Y_Set_;
    int CS_Z_Set_;
    int CS_Rx_Set_;
    int CS_Ry_Set_;
    int CS_Rz_Set_;
    int CS_X_Get_;
    int CS_Y_Get_;
    int CS_Z_Get_;
    int CS_Rx_Get_;
    int CS_Ry_Get_;
    int CS_Rz_Get_;
    int CS_Set_Exec_;
    int CS_Zero_Exec_;

    int AO_Rx_Set_;
    int AO_Ry_Set_;
    int AO_Rz_Set_;
    int AO_Rx_Get_;
    int AO_Ry_Get_;
    int AO_Rz_Get_;
    int AO_Set_Exec_;

    // 存储当前位置信息
    Smarpod_Pose currentCS_;
    double currentAO_[3];
private:
    static const char* driverName;
    friend class SmarpodMotorAxis;
};
