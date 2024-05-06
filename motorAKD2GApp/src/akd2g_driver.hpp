#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"
#include <map>

enum class AxisCmd : int { Stop, Enable, Disable, MotionStatus, Active, HomeAcc };

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

    // replaces "#" in axis_cmd_map keys with axisIndex_
    void sub_axis_index();

    // AXIS#.COMMAND commands from the AKD2G user manual
    std::map<AxisCmd, std::string> axis_cmd_map{
        {AxisCmd::Stop, "AXIS#.STOP"},
        {AxisCmd::Enable, "AXIS#.EN"},
        {AxisCmd::Disable, "AXIS#.DIS"},
        {AxisCmd::Active, "AXIS#.ACTIVE"},
        {AxisCmd::MotionStatus, "AXIS#.MOTIONSTATS"},
        {AxisCmd::HomeAcc, "AXIS#.HOME.ACC"},
    };

    friend class AKD2GMotorController;
};

class epicsShareClass AKD2GMotorController : public asynMotorController {
  public:
    /// \brief Create a new AKD2GMotorController object
    ///
    /// \param[in] portName             The name of the asyn port that will be created for this driver
    /// \param[in] AKD2GPortName        The name of the drvAsynIPPort that was created previously
    /// \param[in] numAxes              The number of axes that this controller supports
    /// \param[in] movingPollPeriod     The time between polls when any axis is moving
    /// \param[in] idlePollPeriod       The time between polls when no axis is moving
    AKD2GMotorController(const char *portName, const char *AKD2GMotorController, int numAxes,
                         double movingPollPeriod, double idlePollPeriod);
    void report(FILE *fp, int level);

    /// \brief Returns a pointer to a AKD2GMotorAxis object
    /// \param[in] asynUser structure that encodes the axis index number
    /// \returns NULL if the axis number encoded in pasynUser is invalid
    AKD2GMotorAxis *getAxis(asynUser *pasynUser);

    /// \brief Returns a pointer to a AKD2GMotorAxis object
    /// \param[in] axisNo Axis index number
    /// \returns NULL if the axis number is invalid
    AKD2GMotorAxis *getAxis(int axisNo);

    // protected:
    // integer asyn param indices defined here

    friend class AKD2GMotorAxis;
};
