#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"
#include <sstream>
#include <unordered_map>

enum class Command : int {
    // Misc.
    AxisOpMode,
    AxisUnitProtary,
    AxisUnitVrotary,
    AxisUnitACCrotary,
    AxisStop,
    AxisEnable,
    AxisDisable,
    AxisMotionStat,
    AxisActive,
    AxisPosition,
    AxisSTOActive,

    // Homing
    AxisHomeMove,
    AxisHomeVelocity,
    AxisHomeAccel,
    AxisHomeDecel,
    AxisHomeDir,
    AxisHomeFound,

    // Motion Task
    AxisMTPosition,
    AxisMTVelocity,
    AxisMTControl,
    AxisMTAccel,
    AxisMTDecel,
    AxisMTNext,     // should be -1
    AxisMTTimeNext, // doesn't matter
    AxisMTMove,
    AxisMTRunning,
};

// Base commands without arguments
inline const std::unordered_map<Command, std::string> cmd_map_template{
    {Command::AxisOpMode, "AXIS#.OPMODE"},

    {Command::AxisUnitProtary, "AXIS#.UNIT.PROTARY"},
    {Command::AxisUnitVrotary, "AXIS#.UNIT.VROTARY"},
    {Command::AxisUnitACCrotary, "AXIS#.UNIT.ACCROTARY"},

    {Command::AxisStop, "AXIS#.STOP"},
    {Command::AxisEnable, "AXIS#.EN"},
    {Command::AxisDisable, "AXIS#.DIS"},
    {Command::AxisActive, "AXIS#.ACTIVE"},
    {Command::AxisMotionStat, "AXIS#.MOTIONSTAT"},
    {Command::AxisPosition, "AXIS#.PL.FB"},
    {Command::AxisSTOActive, "AXIS#.SAFE.STO.ACTIVE"},

    {Command::AxisHomeMove, "AXIS#.HOME.MOVE"},
    {Command::AxisHomeVelocity, "AXIS#.HOME.V"},
    {Command::AxisHomeAccel, "AXIS#.HOME.ACC"},
    {Command::AxisHomeDecel, "AXIS#.HOME.DEC"},
    {Command::AxisHomeDir, "AXIS#.HOME.DIR"},
    {Command::AxisHomeFound, "AXIS#.MOTIONSTAT.HOMEFOUND"},

    // This EPICS driver uses Motion Task 0 exclusively
    {Command::AxisMTPosition, "AXIS#.MT.P 0"},
    {Command::AxisMTVelocity, "AXIS#.MT.V 0"},
    {Command::AxisMTControl, "AXIS#.MT.CNTL 0"},
    {Command::AxisMTAccel, "AXIS#.MT.ACC 0"},
    {Command::AxisMTDecel, "AXIS#.MT.DEC 0"},
    {Command::AxisMTNext, "AXIS#.MT.MTNEXT 0"},
    {Command::AxisMTTimeNext, "AXIS#.MT.TNEXT 0"},
    {Command::AxisMTMove, "AXIS#.MT.Move 0"},
    {Command::AxisMTRunning, "AXIS#.MT.RUNNING 0"},
};

class epicsShareClass Akd2gMotorAxis : public asynMotorAxis {
  public:
    Akd2gMotorAxis(class Akd2gMotorController *pC, int axisNo);
    void report(FILE *fp, int level);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setClosedLoop(bool closedLoop);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity,
                    double acceleration);

  private:
    Akd2gMotorController *pC_;
    int axisIndex_;
    std::unordered_map<Command, std::string> cmd_map_ = cmd_map_template;

    // Replace "#" with the axisIndex_ in cmd_map_
    void replace_axis_index();

    template <typename T> std::string fmt_cmd(Command cmd, T arg) {
        std::ostringstream oss;
        oss << cmd_map_.at(cmd) << " " << arg;
        return oss.str();
    };

    bool is_enabled();
    int dumb_ = 0;

    friend class Akd2gMotorController;
};

class epicsShareClass Akd2gMotorController : public asynMotorController {
  public:
    /// \brief Create a new Akd2gMotorController object
    ///
    /// \param[in] portName             The name of the asyn port that will be created for this
    /// driver
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

    friend class Akd2gMotorAxis;
};
