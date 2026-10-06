#pragma once

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

class MultiRobotInterfaceAbstract : public RobotInterface {
public:
MultiRobotInterfaceAbstract(const std::string& taskFile, 
    const std::string& libraryFolder, bool verbose);

virtual ~MultiRobotInterfaceAbstract() = default;

const vector_t& getInitialState() { return initialState_; }

const OptimalControlProblem& getOptimalControlProblem() const override { return problems_.front(); }
const OptimalControlProblem& getOptimalControlProblem(int index) const { return problems_[index]; }

const std::vector<OptimalControlProblem>& getOptimalControlProblems() const { return problems_; }

const RolloutBase& getRollout() const { return *rolloutPtrs_.front(); }
const RolloutBase& getRollout(int index) const { return *rolloutPtrs_[index]; }

sqp::Settings& sqpSettings() { return sqpSettings_; }
mpc::Settings& mpcSettings() { return mpcSettings_; }
ddp::Settings& ddpSettings() { return ddpSettings_; }

const Initializer& getInitializer() const override { return *centroidalInitializerPtrs_.front(); }
const Initializer& getInitializer(int index) const { return *centroidalInitializerPtrs_[index]; }

std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

std::shared_ptr<SwitchedModelReferenceManagerWithTerrain> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

protected:  
ddp::Settings ddpSettings_;
sqp::Settings sqpSettings_;
mpc::Settings mpcSettings_;

std::vector<OptimalControlProblem> problems_;
std::vector<std::unique_ptr<RolloutBase>> rolloutPtrs_;
std::vector<std::unique_ptr<Initializer>> centroidalInitializerPtrs_;

std::shared_ptr<SwitchedModelReferenceManagerWithTerrain> referenceManagerPtr_;

std::string taskFile_;
std::string libraryFolder_;
bool verbose_;

vector_t initialState_;

};

} // namespace multi_robot
} // namespace ocs2