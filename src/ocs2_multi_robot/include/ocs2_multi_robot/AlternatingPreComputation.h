#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

// #include "ocs2_quadruped/common/ModelSettings.h"
// #include "ocs2_quadruped/constraint/EndEffectorLinearConstraint.h"
// #include "ocs2_quadruped/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/AlternatingTargetTrajectories.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Mainly for caching and computing the frozen decision variables or constants from previous solution
 */
class AlternatingPreComputation : public PreComputation {
 public:
  AlternatingPreComputation(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                            const std::vector<vector_t>& handlePositions,
                            const AlternatingTargetTrajectories& previousSolution,
                            size_t numDualVariables);
  ~AlternatingPreComputation() override = default;

  AlternatingPreComputation* clone() const override;

  void updatePreviousSolution(const AlternatingTargetTrajectories& newSolution) {
    previousSolution_ = newSolution;
  }

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  void requestFinal(RequestSet request, scalar_t t, const vector_t& x) override;

  const vector_t& getRelativeHandlePositions(int robotId) const { return handlePositions_[robotId]; };

  const vector_t& getPrevHandlePositionInWorld(int robotId) const { return prevHandlePositionsInWorld_[robotId]; };

  const vector_t& getPrevRobotState(int robotId) const { return prevRobotStates_[robotId]; };

  const vector_t& getPrevRobotInput(int robotId) const { return prevRobotInputs_[robotId]; };

  const vector_t& getPrevDualVariable(int dualVariableId) const { return prevDualVariables_[dualVariableId]; };

  const vector_t& getPrevCargoState() const { return prevCargoState_; };

  const vector_t& getPrevCargoInput() const { return prevCargoInput_; };

  size_t getHandleCount() const { return handlePositions_.size(); }
  size_t getNumDualVariables() const { return numDualVariables_; }
  size_t getNumRobots() const { return numRobots_; }
  SwitchedModelReferenceManagerWithTerrain* getReferenceManagerPtr() const { return referenceManagerPtr_; }

 private:
  AlternatingPreComputation(const AlternatingPreComputation& other) = default;

  SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  std::vector<vector_t> handlePositions_;
  AlternatingTargetTrajectories previousSolution_;
  std::vector<vector_t> prevDualVariables_;

  std::vector<vector_t> prevHandlePositionsInWorld_;
  std::vector<vector_t> prevRobotStates_;
  std::vector<vector_t> prevRobotInputs_;
  vector_t prevCargoState_;
  vector_t prevCargoInput_;

  size_t numRobots_;
  size_t numDualVariables_;

};

}  // namespace multi_robot
}  // namespace ocs2
