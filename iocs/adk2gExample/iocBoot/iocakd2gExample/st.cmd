# ../../bin/${EPICS_HOST_ARCH}/akd2gExample st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocakd2gExampleLinux.dbd")
iocakd2gExampleLinux_registerRecordDeviceDriver(pdbbase)

< settings.iocsh

< akd2g.iocsh

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
