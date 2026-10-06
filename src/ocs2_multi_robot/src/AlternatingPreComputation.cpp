#include "ocs2_multi_robot/AlternatingPreComputation.h"
#include <iostream>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AlternatingPreComputation::AlternatingPreComputation(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                            const std::vector<vector_t>& handlePositions,
                            const AlternatingTargetTrajectories& previousSolution,
                            size_t numDualVariables)
    : referenceManagerPtr_(&referenceManager), handlePositions_(handlePositions), previousSolution_(previousSolution), numDualVariables_(numDualVariables) {
  numRobots_ = handlePositions_.size();
  prevHandlePositionsInWorld_.resize(numRobots_); // excluding the cargo
  prevRobotStates_.resize(numRobots_);  // excluding the cargo state
  prevRobotInputs_.resize(numRobots_);  // excluding the cargo input
  prevDualVariables_.resize(numDualVariables_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AlternatingPreComputation* AlternatingPreComputation::clone() const {
  auto* clonePtr = new AlternatingPreComputation(*referenceManagerPtr_, handlePositions_, previousSolution_, numDualVariables_);
  // std::cerr << "[AlternatingPreComputation] Cloned from " << this << " to " << clonePtr
  //           << " handleCount=" << clonePtr->handlePositions_.size() << '\n';
  return clonePtr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AlternatingPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
//   if (!request.containsAny(Request::Dynamics + Request::Cost + Request::Constraint + Request::SoftConstraint)) {
//     return;
//   }
  // std::cerr << "[AlternatingPreComputation::request] this=" << this << " t=" << t << " handleCount="
  //           << handlePositions_.size() << " numRobots=" << numRobots_ << " numDualVariables="
  //           << numDualVariables_ << '\n';

  const auto prevState = previousSolution_.getDesiredState(t);
  const auto prevInput = previousSolution_.getDesiredInput(t);
  const auto prevDualVariable = previousSolution_.getDesiredDualVariable(t);

  // Cache the cargo state as the first few elements (the order of full state is at the end)
  prevCargoState_ = prevState.tail(CARGO_STATE_DIM);
  prevCargoInput_ = prevInput.tail(numRobots_ * ARM_CONTACT_DIM);

  for (size_t robotId = 0; robotId < numRobots_; robotId++) {
    const size_t stateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
    const size_t inputOffset = robotId * ALTERNATING_SINGLE_ROBOT_INPUT_DIM;
    // Cache the robot state and input
    prevRobotStates_[robotId] = prevState.segment(stateOffset, SINGLE_ROBOT_STATE_DIM);
    prevRobotInputs_[robotId] = prevInput.segment(inputOffset, ALTERNATING_SINGLE_ROBOT_INPUT_DIM);
  }

  // Compute the handles positions in the world frame based on the previous solution
  const auto cargoPos = prevCargoState_.segment(0,3); // cargo COM position
  const auto cargoOri = prevCargoState_.segment(6,3); // cargo ZYX euler angles

  // Transformation from cargo to world
  for (size_t handleId = 0; handleId < numRobots_; handleId++) {
    const auto handlePosRelLinear = handlePositions_[handleId].head(3);
    prevHandlePositionsInWorld_[handleId] = getRotationMatrixFromZyxEulerAngles<scalar_t>(cargoOri) * handlePosRelLinear + cargoPos;
  }

  for (size_t dualVariableId = 0; dualVariableId < numDualVariables_; dualVariableId++) {
    prevDualVariables_[dualVariableId] = prevDualVariable.segment(dualVariableId*DUAL_VARIABLE_DIM, DUAL_VARIABLE_DIM);
  }

  // std::cerr << "[AlternatingPreComputation::request] Cached previous solution at time " << t << '\n';

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AlternatingPreComputation::requestFinal(RequestSet request, scalar_t t, const vector_t& x) {
  const auto prevState = previousSolution_.getDesiredState(t);
  // Cache the cargo state as the first few elements (the order of full state is at the end)
  prevCargoState_ = prevState.tail(CARGO_STATE_DIM);
  // const auto prevDualVariable = previousSolution_.getDesiredDualVariable(t);

  for (size_t robotId = 0; robotId < numRobots_; robotId++) {
    const size_t stateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
    // Cache the robot state and input
    prevRobotStates_[robotId] = prevState.segment(stateOffset, SINGLE_ROBOT_STATE_DIM);
  }

  // Compute the handles positions in the world frame based on the previous solution
  const auto cargoPos = prevCargoState_.segment(0,3); // cargo COM position
  const auto cargoOri = prevCargoState_.segment(6,3); // cargo ZYX euler angles

  // Transformation from cargo to world
  for (size_t handleId = 0; handleId < numRobots_; handleId++) {
    const auto handlePosRelLinear = handlePositions_[handleId].head(3);
    prevHandlePositionsInWorld_[handleId] = getRotationMatrixFromZyxEulerAngles<scalar_t>(cargoOri) * handlePosRelLinear + cargoPos;
  }

  // for (size_t dualVariableId = 0; dualVariableId < numDualVariables_; dualVariableId++) {
  //   prevDualVariables_[dualVariableId] = prevDualVariable.segment(dualVariableId*DUAL_VARIABLE_DIM, DUAL_VARIABLE_DIM);
  // }
}

}  // namespace quadruped
}  // namespace ocs2