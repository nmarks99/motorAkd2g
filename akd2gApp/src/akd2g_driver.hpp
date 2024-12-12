#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"
#include <map>

enum class AxisCmd : int { Stop, Enable, Disable, MotionStatus, Active, HomeAcc };

class epicsShareClass Akd2gMotorAxis : public asynMotorAxis {
  public:

    Akd2gMotorAxis(class Akd2gMotorController *pC, int axisNo);
    void report(FILE *fp, int level);
    asynStatus stop(double acceleration);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setClosedLoop(bool closedLoop);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);

  private:
    Akd2gMotorController *pC_;
    int axisIndex_;

    // replaces "#" in axis_cmd_map keys with axisIndex_
    void sub_axis_index();

    // AXIS#.COMMAND commands from the Akd2g user manual
    std::map<AxisCmd, std::string> axis_cmd_map{
        {AxisCmd::Stop, "AXIS#.STOP"},
        {AxisCmd::Enable, "AXIS#.EN"},
        {AxisCmd::Disable, "AXIS#.DIS"},
        {AxisCmd::Active, "AXIS#.ACTIVE"},
        {AxisCmd::MotionStatus, "AXIS#.MOTIONSTATS"},
        {AxisCmd::HomeAcc, "AXIS#.HOME.ACC"},
    };

    friend class Akd2gMotorController;
};

class epicsShareClass Akd2gMotorController : public asynMotorController {
  public:
    /// \brief Create a new Akd2gMotorController object
    ///
    /// \param[in] portName             The name of the asyn port that will be created for this driver
    /// \param[in] Akd2gPortName        The name of the drvAsynIPPort that was created previously
    /// \param[in] numAxes              The number of axes that this controller supports
    /// \param[in] movingPollPeriod     The time between polls when any axis is moving
    /// \param[in] idlePollPeriod       The time between polls when no axis is moving
    Akd2gMotorController(const char *portName, const char *Akd2gMotorController, int numAxes,
                         double movingPollPeriod, double idlePollPeriod);
    void report(FILE *fp, int level);

    /// \brief Returns a pointer to a Akd2gMotorAxis object
    /// \param[in] asynUser structure that encodes the axis index number
    /// \returns NULL if the axis number encoded in pasynUser is invalid
    Akd2gMotorAxis *getAxis(asynUser *pasynUser);

    /// \brief Returns a pointer to a Akd2gMotorAxis object
    /// \param[in] axisNo Axis index number
    /// \returns NULL if the axis number is invalid
    Akd2gMotorAxis *getAxis(int axisNo);

    // protected:
    // integer asyn param indices defined here

    friend class Akd2gMotorAxis;
};
