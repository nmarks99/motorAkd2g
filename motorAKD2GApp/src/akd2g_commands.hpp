#include <map>
#include <string>

enum class AxisCmd { Stop, Enable, Disable, MotionStatus, Active, HomeAcc };

struct AKD2GAxisCmd {

    AKD2GAxisCmd(int axis_index);

    std::map<AxisCmd, std::string> cmd{
      {AxisCmd::Stop, "AXIS#.STOP"},
      {AxisCmd::Enable, "AXIS#.EN"},
      {AxisCmd::Disable, "AXIS#.DIS"},
      {AxisCmd::MotionStatus, "AXIS#.MOTIONSTATS"},
      {AxisCmd::Active, "AXIS#.ACTIVE"},
      {AxisCmd::HomeAcc, "AXIS#.HOME.ACC"},
    };
};
