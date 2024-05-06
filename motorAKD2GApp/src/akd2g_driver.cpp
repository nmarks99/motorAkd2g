#include <asynOctetSyncIO.h>
#include <cstdio>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <sstream>

#include "akd2g_driver.hpp"

constexpr int NUM_PARAMS = 0;

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
    
    // Only 2 axes are supported
    if (numAxes > 2) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Requested %d axes but only 2 are supported\n",
                  numAxes);
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

AKD2GMotorAxis *AKD2GMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<AKD2GMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

AKD2GMotorAxis *AKD2GMotorController::getAxis(int axisNo) {
    return static_cast<AKD2GMotorAxis *>(asynMotorController::getAxis(axisNo));
}

void AKD2GMotorAxis::sub_axis_index() {
    for (auto m = axis_cmd_map.begin(); m != axis_cmd_map.end(); ++m) {
        size_t index_rep = m->second.find("#");
        if (index_rep != std::string::npos) {
            m->second.replace(index_rep, 1, std::to_string(axisIndex_));
        }
    }
}

AKD2GMotorAxis::AKD2GMotorAxis(AKD2GMotorController *pC, int axisNo) : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "AKD2GMotorAxis created with axis index %d\n", axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    sub_axis_index();

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

    // std::string cmd = axis_cmd.cmd.at(AxisCmd::Stop);
    std::string cmd = axis_cmd_map.at(AxisCmd::Stop);

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
    ss << position << "," << relative << "," << min_velocity << "," << max_velocity << "," << acceleration;

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

    std::string cmd = axis_cmd_map.at(AxisCmd::MotionStatus);

    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", cmd.c_str());
    // sprintf(pC_->outString_, "%s", cmd.str().c_str());
    // asyn_status = pC_->writeReadController();
    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "Read: %s\n", pC_->inString_);

    callParamCallbacks();
    return asyn_status;
}

asynStatus AKD2GMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;

    std::string cmd = closedLoop ? axis_cmd_map.at(AxisCmd::Enable) : axis_cmd_map.at(AxisCmd::Disable);

    // if (closedLoop) {
    // cmd = axis_cmd_map.at(AxisCmd::Enable);
    // } else {
    // cmd = axis_cmd_map.at(AxisCmd::Disable);
    // }

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
