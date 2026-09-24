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
#define SMARPOD_FREF_METHOD        1000
#define SMARPOD_FREF_ZDIRECTION    1002
#define SMARPOD_FREF_XDIRECTION    1003
#define SMARPOD_FREF_YDIRECTION    1004
// 传感器模式常量定义
#define SMARPOD_SENSORS_DISABLED   0   // 传感器禁用
#define SMARPOD_SENSORS_ENABLED    1   // 传感器启用
#define SMARPOD_SENSORS_POWERSAVE  2   // 传感器节能模式

// 新增设备信息参数字符串常量
#define SmarpodScanDevicesString       "SCAN_DEVICES"        // 扫描到的设备列表
#define SmarpodConnectStatusString     "CONNECT_STATUS"      // 连接状态（0=断开，1=连接）
#define SmarpodLibraryVersionString    "LIBRARY_VERSION"     // Smarpod库版本（格式：x.y.z）
#define SmarpodDeviceIDString          "DEVICE_ID"           // 设备ID
#define SmarpodLocatorString           "LOCATOR"             // 配置的locator_参数
#define SmarpodModelNameString         "MODEL_NAME"          // 设备型号名称（如P-CLL:10115）
#define SmarpodSystemLocatorString     "SYSTEM_LOCATOR"      // Smarpod_GetSystemLocator返回值
#define SmarpodModelCodeString         "MODEL_CODE"          // 设备型号代码（如10115）

#define SmarPodPivotXString         "PIVOT_X"
#define SmarPodPivotYString         "PIVOT_Y"
#define SmarPodPivotZString         "PIVOT_Z"
#define SmarPodSetPivotString       "SET_PIVOT"
#define SmarpodStandbyString        "STANDBY"
#define SmarpodStandbyStatusString  "STANDBY_STATUS"
#define SmarpodReachableStatusString   "REACHABLE_STATUS"  // 可达性状态
#define SmarpodCSYS_XString          "CSYS_X"          // 坐标系X偏移 (m)
#define SmarpodCSYS_YString          "CSYS_Y"          // 坐标系Y偏移 (m)
#define SmarpodCSYS_ZString          "CSYS_Z"          // 坐标系Z偏移 (m)
#define SmarpodCSYS_RXString         "CSYS_RX"         // 坐标系RX旋转 (deg)
#define SmarpodCSYS_RYString         "CSYS_RY"         // 坐标系RY旋转 (deg)
#define SmarpodCSYS_RZString         "CSYS_RZ"         // 坐标系RZ旋转 (deg)
#define SmarpodSetCSYSString         "SET_CSYS"        // 设置坐标系触发
#define SmarpodGetCSYSString         "GET_CSYS"        // 获取坐标系触发
#define SmarpodCSYSStatusString      "CSYS_STATUS"     // 坐标系状态
#define SmarpodMAXFrequencyString    "MAX_FREQUENCY"   // 最大频率

#define SmarPodMotorMoveStatusString                "MOVE_STATUS"
#define SmarPodMotorCalibratingString               "CAL"
#define SmarPodMotorStatusCalibrateString           "CALIBRATE_STATUS"
#define SmarPodMotorReferencingString               "REFERENCE"
#define SmarPodMotorStatusReferenceString           "REFERENCE_STATUS"
#define SmarPodMotorReferenceMethodString           "REFERENCE_METHOD"
#define SmarPodMotorReferenceZSaftDirectionString   "REFERENCE_ZDIRECTION"
#define SmarPodMotorReferenceXSaftDirectionString   "REFERENCE_XDIRECTION"
#define SmarPodMotorReferenceYSaftDirectionString   "REFERENCE_YDIRECTION"
#define SmarPodMotorReferenceFrequencyString        "REFERENCE_FREQUENCY"
#define SmarPodMoveToZeroString                     "MOVE_TO_ZERO"
#define SmarPodSetToZeroString                      "SET_TO_ZERO"
#define SmarPodStopAndHoldingString                 "STOP_AND_HOLDING"
#define SmarPodStopHoldingTimeString                "STOP_WITH_HOLDING_TIME"
#define SmarPodSensorModeString                     "SENSOR_MODE"

class SmarpodMotorAxis : public asynMotorAxis {
public:
    SmarpodMotorAxis(class SmarpodMotorController *pC, int axisNo);
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
    asynStatus poll(bool *moving) override;
    asynStatus stop();
    asynStatus stopAndHold(unsigned int holdTime = SMARPOD_HOLDTIME_INFINITE);
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
    void scanAllParameters();
    asynStatus configureController(); // 配置控制器（离线配置，无需打开设备）
    asynStatus configureSystem();// 配置系统（在线配置，需已打开设备）
    asynStatus connect();
    asynStatus disconnect();
    asynStatus checkDeviceStatus();
    asynStatus calibrateSensor();// 传感器校准
    asynStatus findReferenceMarks();// 查找参考点（新增独立函数）
    bool isReferenced();// 检查参考状态
    asynStatus setCurrentPoseAsZero();
    asynStatus moveToZero();
    asynStatus setSensorMode(unsigned int mode);    // 设置传感器模式
    asynStatus getSensorMode(unsigned int* mode);    // 获取当前传感器模式
    asynStatus setPivot(const double pivot[3]);// 设置虚拟旋转中心点
    asynStatus getPivot(double pivot[3]);// 获取虚拟旋转中心点
    asynStatus enterStandbyMode();
    asynStatus setCoordinateSystem();
    asynStatus getCurrentCoordinateSystem();
    asynStatus setMaxFrequency(unsigned int frequency);
    asynStatus getMaxFrequency(unsigned int* frequency);
    asynStatus logSmarpodError(Smarpod_Status status, const char* context);
    SmarpodMotorAxis* getAxis(asynUser *pasynUser);
    SmarpodMotorAxis* getAxis(int axisNo) override;
    asynStatus PrintStatus(const char* operation, Smarpod_Status status);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
    int getAxisIndex(asynUser *pasynUser);  // 添加这个声明
    // 新增辅助方法：更新库版本信息
    void updateLibraryVersion();
    // 新增辅助方法：设置字符串参数（简化调用）
    asynStatus setDeviceStringParam(int function, const char* value);
    asynStatus poll() override;
protected:
    SmarpodMotorAxis **pAxes_;  
    unsigned int smarpodID_;
    unsigned int model_;
    char locator_[256];
    bool isConnected_;
    epicsMutexId mutex_;
    int moveStatus_;
    int motorCalibrate_;
    int motorReference_;
    int moveToZero_;
    int setToZero_;
    int stopHolding_;
    int stopHoldingTime_;
    int motorCalibrateStatus_;
    int motorReferenceStatus_;
    int referenceMethod_;           // FREF_METHOD 参数
    int referenceZDirection_;       // FREF_ZDIRECTION 参数
    int referenceXDirection_;       // FREF_XDIRECTION 参数  
    int referenceYDirection_;       // FREF_YDIRECTION 参数
    int referenceFrequency_;        // FREF_AND_CAL_FREQUENCY 参数
    int sensorMode_;  // 传感器模式参数索引
    double pivot_[3];  // 存储当前旋转中心点
    int pivotX_;       // X轴参数索引
    int pivotY_;       // Y轴参数索引
    int pivotZ_;       // Z轴参数索引
    int setPivot_;     // 设置Pivot的触发参数索引
    int standby_;              // 进入Standby模式触发
    int standbyStatus_;        // Standby状态读取
    int reachableStatus_;  // 1:不可达, 0:可达,
    int csysX_;          // 坐标系X偏移
    int csysY_;          // 坐标系Y偏移
    int csysZ_;          // 坐标系Z偏移
    int csysRX_;         // 坐标系RX旋转
    int csysRY_;         // 坐标系RY旋转
    int csysRZ_;         // 坐标系RZ旋转
    int setCSYS_;           // 设置坐标系触发
    int getCSYS_;           // 获取坐标系触发
    int csysStatus_;        // 坐标系状态 (0:空闲, 1:设置中, 2:获取中, -1:错误)
        // 新增设备信息参数索引
    int scanDevices_;
    int connectStatus_;
    int serialNumber_;
    int firmwareVersion_;
    int libraryVersion_;
    int deviceID_;
    int locatorParam_;
    int modelName_;
    int systemLocator_;
    int modelCode_;
    int maxFrequency_;
private:
    static const char* driverName;
    friend class SmarpodMotorAxis;
};
