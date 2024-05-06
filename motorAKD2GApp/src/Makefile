TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += akd2g

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install AKD2G.dbd into <top>/dbd
DBD += akd2gSupport.dbd

akd2gSupport_DBD += akd2g.dbd

# specify all source files to be compiled and added to the library
akd2g_SRCS += akd2g_driver.cpp

akd2g_LIBS += motor asyn
akd2g_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

