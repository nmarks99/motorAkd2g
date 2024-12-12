#include <asynOctetSyncIO.h>
#include <cstdio>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <sstream>

#include "akd2g_driver.hpp"

constexpr int NUM_PARAMS = 0;

Akd2gMotorController::Akd2gMotorController(const char *portName, const char *Akd2gMotorPortName, int numAxes,
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
    Akd2gMotorAxis *pAxis;
    static const char *functionName = "Akd2gMotorController::Akd2gMotorController";

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(Akd2gMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Akd2g motor controller\n",
                  functionName);
    }
    
    // Only 2 axes are supported
    if (numAxes > 2) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Requested %d axes but only 2 are supported\n",
                  numAxes);
    }

    // Create Akd2gMotorAxis object for each axis
    // if not done here, user must call Akd2gMotorCreateAxis from cmd file
    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new Akd2gMotorAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

extern "C" int Akd2gMotorCreateController(const char *portName, const char *Akd2gMotorPortName, int numAxes,
                                          int movingPollPeriod, int idlePollPeriod) {
    Akd2gMotorController *pAkd2gMotorController = new Akd2gMotorController(
        portName, Akd2gMotorPortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    pAkd2gMotorController = NULL;
    return (asynSuccess);
}

void Akd2gMotorController::report(FILE *fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "Akd2g Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

Akd2gMotorAxis *Akd2gMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<Akd2gMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

Akd2gMotorAxis *Akd2gMotorController::getAxis(int axisNo) {
    return static_cast<Akd2gMotorAxis *>(asynMotorController::getAxis(axisNo));
}

void Akd2gMotorAxis::sub_axis_index() {
    for (auto m = axis_cmd_map.begin(); m != axis_cmd_map.end(); ++m) {
        size_t index_rep = m->second.find("#");
        if (index_rep != std::string::npos) {
            m->second.replace(index_rep, 1, std::to_string(axisIndex_));
        }
    }
}

Akd2gMotorAxis::Akd2gMotorAxis(Akd2gMotorController *pC, int axisNo) : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "Akd2gMotorAxis created with axis index %d\n", axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    sub_axis_index();

    callParamCallbacks();
}

void Akd2gMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}

asynStatus Akd2gMotorAxis::stop(double acceleration) {

    asynStatus asyn_status = asynSuccess;

    // std::string cmd = axis_cmd.cmd.at(AxisCmd::Stop);
    std::string cmd = axis_cmd_map.at(AxisCmd::Stop);

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", cmd.c_str());
    // sprintf(pC_->outString_, "%s", cmd.str().c_str());
    // asyn_status = pC_->writeReadController();

    callParamCallbacks();
    return asyn_status;
}

asynStatus Akd2gMotorAxis::move(double position, int relative, double min_velocity, double max_velocity,
                                double acceleration) {

    asynStatus asyn_status = asynSuccess;

    std::stringstream ss;
    ss << position << "," << relative << "," << min_velocity << "," << max_velocity << "," << acceleration;

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", ss.str().c_str());
    callParamCallbacks();
    return asyn_status;
}

asynStatus Akd2gMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus asyn_status = asynSuccess;
    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", ss.str().c_str());
    callParamCallbacks();
    return asyn_status;
}

asynStatus Akd2gMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;

    std::string cmd = axis_cmd_map.at(AxisCmd::MotionStatus);

    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", cmd.c_str());
    // sprintf(pC_->outString_, "%s", cmd.str().c_str());
    // asyn_status = pC_->writeReadController();
    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "Read: %s\n", pC_->inString_);

    callParamCallbacks();
    return asyn_status;
}

asynStatus Akd2gMotorAxis::setClosedLoop(bool closedLoop) {
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

static const iocshArg Akd2gMotorCreateControllerArg0 = {"asyn port name", iocshArgString};
static const iocshArg Akd2gMotorCreateControllerArg1 = {"Controller port name", iocshArgString};
static const iocshArg Akd2gMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg Akd2gMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg Akd2gMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const Akd2gMotorCreateControllerArgs[] = {
    &Akd2gMotorCreateControllerArg0, &Akd2gMotorCreateControllerArg1, &Akd2gMotorCreateControllerArg2,
    &Akd2gMotorCreateControllerArg3, &Akd2gMotorCreateControllerArg4};
static const iocshFuncDef Akd2gMotorCreateControllerDef = {"Akd2gMotorCreateController", 5,
                                                           Akd2gMotorCreateControllerArgs};

static void Akd2gMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    Akd2gMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void Akd2gMotorRegister(void) {
    iocshRegister(&Akd2gMotorCreateControllerDef, Akd2gMotorCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(Akd2gMotorRegister);
}
