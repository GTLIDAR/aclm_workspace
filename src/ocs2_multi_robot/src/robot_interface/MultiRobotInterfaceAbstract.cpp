#include "ocs2_multi_robot/robot_interface/MultiRobotInterfaceAbstract.h"

namespace ocs2 {
namespace multi_robot {

MultiRobotInterfaceAbstract::MultiRobotInterfaceAbstract(
    const std::string& taskFile, const std::string& libraryFolder, bool verbose) :
    taskFile_(taskFile),
    libraryFolder_(libraryFolder),
    verbose_(verbose) {
        loadData::loadCppDataType(taskFile, "mpc.timeHorizon", mpcSettings_.timeHorizon_);
        loadData::loadCppDataType(taskFile, "mpc.solutionTimeWindow", mpcSettings_.solutionTimeWindow_);
        loadData::loadCppDataType(taskFile, "mpc.coldStart", mpcSettings_.coldStart_);
        loadData::loadCppDataType(taskFile, "mpc.debugPrint", mpcSettings_.debugPrint_);
        loadData::loadCppDataType(taskFile, "mpc.mrtDesiredFrequency", mpcSettings_.mrtDesiredFrequency_);
        loadData::loadCppDataType(taskFile, "mpc.mpcDesiredFrequency", mpcSettings_.mpcDesiredFrequency_);
    }

} // namespace multi_robot
} // namespace ocs2