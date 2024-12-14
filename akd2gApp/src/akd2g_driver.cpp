#include <asynOctetSyncIO.h>
#include <cstdio>
#include <cstdlib>
#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <iocsh.h>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <string_view>

#include "akd2g_driver.hpp"

constexpr int NUM_PARAMS = 0;


// actual res is 100 microdegrees
// When MRES=1, units will be whole number microdegrees
constexpr double DRIVER_RESOLUTION = 1e6; 

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
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to AKD2G controller\n",
                  functionName);
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

void Akd2gMotorAxis::replace_axis_index() {
    constexpr std::string_view pholder = "AXIS#";
    constexpr size_t pholder_len = pholder.length();
    std::string new_str = "AXIS" + std::to_string(axisIndex_);

    for (auto m = cmd_map_.begin(); m != cmd_map_.end(); ++m) {
        size_t ind = m->second.find(pholder);
        if (ind != std::string::npos) {
            m->second.replace(ind, pholder_len, new_str);
        }
    }
}

Akd2gMotorAxis::Akd2gMotorAxis(Akd2gMotorController *pC, int axisNo) : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "Akd2gMotorAxis created with axis index %d\n", axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    replace_axis_index();

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

    // asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "%s\n", cmd_map_.at(Command::AxisStop).c_str());
    sprintf(pC_->outString_, "%s", cmd_map_.at(Command::AxisStop).c_str());
    asyn_status = pC_->writeReadController();

    callParamCallbacks();
    return asyn_status;
}

asynStatus Akd2gMotorAxis::move(double position, int relative, double min_velocity, double max_velocity,
                                double acceleration) {
    asynStatus asyn_status = asynSuccess;
    
    std::stringstream cmd_ss;

    // Set to absolute motion
    // only supporting absolute move commands to the controller for now
    cmd_ss << cmd_map_.at(Command::AxisMTControl) << " 0";
    std::cout << cmd_ss.str() << std::endl;
    cmd_ss.str(""); cmd_ss.clear();

    // Set target position for motion task
    cmd_ss << cmd_map_.at(Command::AxisMTPosition) << " " << std::to_string(position/DRIVER_RESOLUTION);
    std::cout << cmd_ss.str() << std::endl;
    cmd_ss.str(""); cmd_ss.clear();

    // Set velocity for motion task
    cmd_ss << cmd_map_.at(Command::AxisMTVelocity) << " " << std::to_string(max_velocity);
    std::cout << cmd_ss.str() << std::endl;
    cmd_ss.str(""); cmd_ss.clear();

    // Set acceleration for motion task
    cmd_ss << cmd_map_.at(Command::AxisMTAccel) << " " << std::to_string(acceleration);
    std::cout << cmd_ss.str() << std::endl;
    cmd_ss.str(""); cmd_ss.clear();

    // Set deceleration for motion task to same as acceleration
    cmd_ss << cmd_map_.at(Command::AxisMTDecel) << " " << std::to_string(acceleration);
    std::cout << cmd_ss.str() << std::endl;
    cmd_ss.str(""); cmd_ss.clear();

    // sprintf(pC_->outString_, "%s", cmd_map_.at(Command::AxisMTPosition).c_str()); // TODO:


    callParamCallbacks();
    return asyn_status;
}

asynStatus Akd2gMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus asyn_status = asynSuccess;
    callParamCallbacks();
    return asyn_status;
}

asynStatus Akd2gMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;
  
    // Get moving done status
    // TODO: should also check AXIS#.MOTIONSTAT ?
    sprintf(pC_->outString_, "%s", cmd_map_.at(Command::AxisMTRunning).c_str());
    asyn_status = pC_->writeReadController();
    try {
        *moving = std::stoi(pC_->inString_);
    } catch (std::exception &err) { // will be std::out_of_range or std::invalid_argument
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s\n", err.what());
    }
    setIntegerParam(pC_->motorStatusDone_, not *moving);
    setIntegerParam(pC_->motorStatusMoving_, *moving);

    // Read axis position
    sprintf(pC_->outString_, "%s", cmd_map_.at(Command::AxisPosition).c_str());
    asyn_status = pC_->writeReadController();
    std::string in_str(pC_->inString_);
    size_t rm_start = in_str.find("[");
    if (rm_start != std::string::npos) {
        in_str.erase(in_str.begin()+rm_start, in_str.end());
    }
    
    double position_deg = 0.0;
    try {
        position_deg = std::stof(in_str);
        std::cout << position_deg << std::endl;
    } catch (std::exception &err){
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s\n", err.what());
    }
    int long_position_udeg = DRIVER_RESOLUTION * position_deg;      // convert to nanometer "steps"
    setDoubleParam(pC_->motorPosition_, long_position_udeg);  // RRBV [nanometers]
    setDoubleParam(pC_->motorEncoderPosition_, long_position_udeg);


    callParamCallbacks();
    return asyn_status;
}

asynStatus Akd2gMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;

    std::string cmd = closedLoop ? cmd_map_.at(Command::AxisEnable) : cmd_map_.at(Command::AxisDisable);
    sprintf(pC_->outString_, "%s", cmd.c_str());
    asyn_status = pC_->writeReadController();

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
