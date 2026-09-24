TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = smarpodMotor
# smarpodMotor.dbd will be created and installed
DBD += smarpodMotor.dbd

# smarpodMotor.dbd will be made up from these files:
smarpodMotor_DBD += base.dbd
smarpodMotor_DBD += asyn.dbd
smarpodMotor_DBD += motorSupport.dbd
smarpodMotor_DBD += smarpodMotorSupport.dbd
# Include dbd files from all support applications:
#smarpodMotor_DBD += xxx.dbd
USR_INCLUDES += -I/usr/local/include
# Smarpod库路径和链接
#USR_LDFLAGS += -L/usr/local/lib -Wl,-Bdynamic 

#smarpodMotor_LIBS += smarpod smaractctl smaractio smaractlog
# smarpodMotor_LIBS += smarpod 
# smarpodMotor_LIBS += smaractctl 
# smarpodMotor_LIBS += smaractio 
# smarpodMotor_LIBS += smaractlog
smarpodMotor_LIBS += motor
smarpodMotor_LIBS += asyn
smarpodMotor_SRCS += smarpodMCS2Controller.cpp
# Add all the support libraries needed by this IOC
#smarpodMotor_LIBS += xxx

# smarpodMotor_registerRecordDeviceDriver.cpp derives from smarpodMotor.dbd
smarpodMotor_SRCS += smarpodMotor_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
smarpodMotor_SRCS_DEFAULT += smarpodMotorMain.cpp
smarpodMotor_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#smarpodMotor_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
smarpodMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

