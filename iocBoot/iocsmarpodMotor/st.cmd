#!../../bin/linux-x86_64/smarpodMotor

# 加载环境变量
< envPaths

# 切换到项目根目录
cd "${TOP}"

# 1. 加载设备数据库定义
dbLoadDatabase "dbd/smarpodMotor.dbd"
smarpodMotor_registerRecordDeviceDriver pdbbase

# 2. 通过substitution文件批量加载多轴记录
dbLoadTemplate("$(TOP)/iocBoot/iocsmarpodMotor/smarpodMCS2.substitutions")

# 3. 创建Smarpod控制器实例（6轴，端口名SMARPOD需与substitution文件中的PORT匹配）
# 参数：端口名、设备地址、轴数、运动轮询周期(ms)、空闲轮询周期(ms)、设备型号、未使用轴掩码
smarpodMotorCreate("SMARPOD", "network:sn:MCS2-00020183", 6, 10, 20, 10115, 0)

# 4. 初始化IOC
cd "${TOP}/iocBoot/${IOC}"
iocInit

# 5. 可选：打印加载的记录信息（调试用）
dbl

# 6. 可选：启动自动保存（如需持久化参数）
# save_restoreSet_status_prefix("B2:")
# save_restoreSet_IncompleteSetsOk(1)
# save_restoreSet_DatedBackupFiles(1)
# set_savefile_path("${TOP}/iocBoot/${IOC}/autosave")
# dbLoadRecords("$(AUTOSAVE)/db/save_restoreStatus.db", "P=B2:")
# autosave_init("${TOP}/iocBoot/${IOC}/autosave/recovery.req", 30)
# autosave_init("${TOP}/iocBoot/${IOC}/autosave/settings.req", 60)

