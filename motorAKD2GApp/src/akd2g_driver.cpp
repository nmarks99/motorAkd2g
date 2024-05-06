#include <asynOctetSyncIO.h>
#include <cstdio>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <sstream>

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
    }

    if (numAxes > 2) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Requested %d axes but only 2 are supported\n", numAxes);
    }

    // Create AKD2GMotorAxis object for each axis
    // if not done here, user must call AKD2GMotorCreateAxis from cmd file
    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new AKD2GMotorAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

extern "C" int AKD2GMotorCreateController(const char *portName, const char *AKD2GMotorPortName, int numAxes,
                                          int movingPollPeriod, int idlePollPeriod) {
    AKD2GMotorController *pAKD2GMotorController = new AKD2GMotorController(
        portName, AKD2GMotorPortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    pAKD2GMotorController = NULL;
    return (asynSuccess);
}

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

AKD2GMotorAxis::AKD2GMotorAxis(AKD2GMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC), axis_cmd(axisNo + 1) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "AKD2GMotorAxis created with axis index %d\n", axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    callParamCallbacks();
}

void AKD2GMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}

asynStatus AKD2GMotorAxis::stop(double acceleration) {

    asynStatus asyn_status = asynSuccess;

    std::string cmd = axis_cmd.cmd.at(AxisCmd::Stop);

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", cmd.c_str());
    // sprintf(pC_->outString_, "%s", cmd.str().c_str());
    // asyn_status = pC_->writeReadController();

    callParamCallbacks();
    return asyn_status;
}

asynStatus AKD2GMotorAxis::move(double position, int relative, double min_velocity, double max_velocity,
                                double acceleration) {

    asynStatus asyn_status = asynSuccess;

    std::stringstream ss;
    ss << position << ","
        << relative << ","
        << min_velocity << ","
        << max_velocity << ","
        << acceleration;

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", ss.str().c_str());
    callParamCallbacks();
    return asyn_status;
}

asynStatus AKD2GMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus asyn_status = asynSuccess;
    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", ss.str().c_str());
    callParamCallbacks();
    return asyn_status;
}

asynStatus AKD2GMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;

    std::string cmd = axis_cmd.cmd.at(AxisCmd::MotionStatus);

    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", cmd.c_str());
    // sprintf(pC_->outString_, "%s", cmd.str().c_str());
    // asyn_status = pC_->writeReadController();
    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "Read: %s\n", pC_->inString_);

    callParamCallbacks();
    return asyn_status;
}

/// \brief Enable closed loop
asynStatus AKD2GMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;
    std::string cmd;

    if (closedLoop) {
        cmd = axis_cmd.cmd.at(AxisCmd::Enable);
    } else {
        cmd = axis_cmd.cmd.at(AxisCmd::Disable);
    }

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", cmd.c_str());
    // sprintf(pC_->outString_, "AXIS%d.EN", this->axisIndex_);
    // asyn_status = pC_->writeReadController();
    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "Read: %s\n", pC_->inString_);
    return asyn_status;
}

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
