#include "akd2g_commands.hpp"

AKD2GAxisCmd::AKD2GAxisCmd(int axis_index) {
    // replace "#" with axisIndex_ during construction
    for (auto m = cmd.begin(); m != cmd.end(); ++m) {
        size_t index_rep = m->second.find("#");
        if (index_rep != std::string::npos) {
            m->second.replace(index_rep, 1, std::to_string(axis_index));
        }
    }
}
