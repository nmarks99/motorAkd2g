#include <asynOctetSyncIO.h>
#include <cstdio>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "akd2g_driver.hpp"

constexpr int NUM_PARAMS = 0;

/// \brief Create a new AKD2GMotorController object
///
/// \param[in] portName             The name of the asyn port that will be created for this driver
/// \param[in] AKD2GPortName        The name of the drvAsynIPPort that was created previously
/// \param[in] numAxes              The number of axes that this controller supports
/// \param[in] movingPollPeriod     The time between polls when any axis is moving
/// \param[in] idlePollPeriod       The time between polls when no axis is moving
AKD2GMotorController::AKD2GMotorController(const char *portName, const char *AKD2GMotorPortName, int numAxes,
                                           double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_PARAMS,
                          0, // No additional interfaces beyond the base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    asynStatus status;
    int axis;
    AKD2GMotorAxis *pAxis;
    static const char *functionName = "AKD2GMotorController::AKD2GMotorController";

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(AKD2GMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to AKD2G motor controller\n",
                  functionName);
    } else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Connect successful\n");
    }

    // Create AKD2GMotorAxis object for each axis
    // if not done here, user must call AKD2GMotorCreateAxis from cmd file
    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new AKD2GMotorAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/// \breif Creates a new AKD2GMotorController object.
///
/// Configuration command, called directly or from iocsh
/// \param[in] portName             The name of the asyn port that will be created for this driver
/// \param[in] AKD2GMotorPortName   The name of the drvAsynIPPPort that was created previously
/// \param[in] numAxes              The number of axes that this controller supports
/// \param[in] movingPollPeriod     The time in ms between polls when any axis is moving
/// \param[in] idlePollPeriod       The time in ms between polls when no axis is moving
extern "C" int AKD2GMotorCreateController(const char *portName, const char *AKD2GMotorPortName, int numAxes,
                                          int movingPollPeriod, int idlePollPeriod) {
    AKD2GMotorController *pAKD2GMotorController = new AKD2GMotorController(
        portName, AKD2GMotorPortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    pAKD2GMotorController = NULL;
    return (asynSuccess);
}

/// \brief Reports on status of the driver
/// \param[in] fp The file pointer on which report information will be written
/// \param[in] level The level of report detail desired
/// If level > 0 then information is printed about each axis.
/// After printing controller-specific information it calls asynMotorController::report()
void AKD2GMotorController::report(FILE *fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "AKD2G Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

/// \brief Returns a pointer to a AKD2GMotorAxis object
/// \param[in] asynUser structure that encodes the axis index number
/// \returns NULL if the axis number encoded in pasynUser is invalid
AKD2GMotorAxis *AKD2GMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<AKD2GMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

/// \brief Returns a pointer to a AKD2GMotorAxis object
/// \param[in] axisNo Axis index number
/// \returns NULL if the axis number is invalid
AKD2GMotorAxis *AKD2GMotorController::getAxis(int axisNo) {
    return static_cast<AKD2GMotorAxis *>(asynMotorController::getAxis(axisNo));
}

// =============
// AKD2GMotorAxis
// =============

/// \breif Creates a new VirtualMotorAxis object.
/// \param[in] pC Pointer to the VirtualMotorController to which this axis belongs.
/// \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
///
/// Initializes register numbers, etc.
/// Note: the following constructor needs to be modified to accept the stepSize argument if
/// AKD2GMotorCreateAxis will be called from iocsh, which is necessary for controllers that work in EGU
/// (engineering units) instead of steps.
AKD2GMotorAxis::AKD2GMotorAxis(AKD2GMotorController *pC, int axisNo) : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;

    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "AKD2GMotorAxis created with axis index %d\n", axisIndex_);

    callParamCallbacks();
}

/// \brief Report on the axis
void AKD2GMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}

/// \brief Stop the axis
asynStatus AKD2GMotorAxis::stop(double acceleration) {

    asynStatus asyn_status;
    
    sprintf(pC_->outString_, "AXIS%d.STOP", this->axisNo_);
    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "Sending: AXIS%d.STOP\n", this->axisIndex_);
    asyn_status = pC_->writeReadController();

    callParamCallbacks();
    return asyn_status ? asynError : asynSuccess; 
}

/// \brief Move the axis
asynStatus AKD2GMotorAxis::move(double position, int relative, double min_velocity, double max_velocity,
                                double acceleration) {
    return asynSuccess;
}

/// \brief home the axis
asynStatus AKD2GMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    return asynSuccess;
}

/// \brief Poll the axis
asynStatus AKD2GMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;
    
    sprintf(pC_->outString_, "AXIS%d.MOTIONSTAT", this->axisIndex_);
    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "AXIS%d.MOTIONSTAT\n", this->axisIndex_);
    asyn_status = pC_->writeReadController();
    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "Read: %s\n", pC_->inString_);

    callParamCallbacks();

    return asyn_status ? asynError : asynSuccess;
}

/// \brief Enable closed loop
asynStatus AKD2GMotorAxis::setClosedLoop(bool closedLoop) { return asynSuccess; }

// ==================
// iosch registration
// ==================

static const iocshArg AKD2GMotorCreateControllerArg0 = {"asyn port name", iocshArgString};
static const iocshArg AKD2GMotorCreateControllerArg1 = {"Controller port name", iocshArgString};
static const iocshArg AKD2GMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg AKD2GMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg AKD2GMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const AKD2GMotorCreateControllerArgs[] = {
    &AKD2GMotorCreateControllerArg0, &AKD2GMotorCreateControllerArg1, &AKD2GMotorCreateControllerArg2,
    &AKD2GMotorCreateControllerArg3, &AKD2GMotorCreateControllerArg4};
static const iocshFuncDef AKD2GMotorCreateControllerDef = {"AKD2GMotorCreateController", 5,
                                                           AKD2GMotorCreateControllerArgs};

static void AKD2GMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    AKD2GMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void AKD2GMotorRegister(void) {
    iocshRegister(&AKD2GMotorCreateControllerDef, AKD2GMotorCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(AKD2GMotorRegister);
}
