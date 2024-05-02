# ../../bin/${EPICS_HOST_ARCH}/motorAKD2G st.cmd
< envPaths

< settings.iocsh

dbLoadDatabase("../../dbd/iocmotorAKD2GLinux.dbd")
iocmotorAKD2GLinux_registerRecordDeviceDriver(pdbbase)

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
