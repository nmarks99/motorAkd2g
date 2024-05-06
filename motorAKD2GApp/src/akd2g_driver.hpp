#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "akd2g_commands.hpp"

class epicsShareClass AKD2GMotorAxis : public asynMotorAxis {
    public:
        AKD2GMotorAxis(class AKD2GMotorController *pC, int axisNo);

        void report(FILE *fp, int level);
        asynStatus stop(double acceleration);
        asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
        asynStatus poll(bool *moving);
        asynStatus setClosedLoop(bool closedLoop);
        asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
        
    private:
        AKD2GMotorController *pC_;
        int axisIndex_;
        AKD2GAxisCmd axis_cmd;
    
    friend class AKD2GMotorController;
};

class epicsShareClass AKD2GMotorController : public asynMotorController {
    public:
        AKD2GMotorController(const char *portName,
                            const char *AKD2GMotorController,
                            int numAxes, double movingPollPeriod, double idlePollPeriod);
        void report(FILE *fp, int level);
        AKD2GMotorAxis* getAxis(asynUser *pasynUser);
        AKD2GMotorAxis* getAxis(int axisNo);

    // protected:
    // integer asyn param indices defined here

    friend class AKD2GMotorAxis;

};
