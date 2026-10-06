#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include <ocs2_quadruped/common/utils.h>
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

/**
 * State-input tracking cost used for intermediate times
 */
class MultiRobotWCargoTrackingCost final : public QuadraticStateInputCost {
public:
  MultiRobotWCargoTrackingCost(matrix_t Q, matrix_t R, const scalar_t& robot_mass, const scalar_t& cargo_mass, 
                              size_t numRobots, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
          : QuadraticStateInputCost(std::move(Q), std::move(R)), robot_mass_(robot_mass), cargo_mass_(cargo_mass), 
            numRobots_(numRobots), referenceManagerPtr_(&referenceManager) {}

  ~MultiRobotWCargoTrackingCost() override = default;
  MultiRobotWCargoTrackingCost* clone() const override { return new MultiRobotWCargoTrackingCost(*this); }

private:
  MultiRobotWCargoTrackingCost(const MultiRobotWCargoTrackingCost& rhs) = default;

  // Here derivation just means the difference between desired value and actual value, not 'dy/dx'
  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const size_t stateDim = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
    const size_t inputDim = numRobots_ * SINGLE_ROBOT_INPUT_DIM + numRobots_ * ARM_CONTACT_DIM;
    
    // Always track the full state for multi-robot formulation
    const vector_t desiredState = targetTrajectories.getDesiredState(time);
    if (desiredState.size() < stateDim) {
      throw std::runtime_error("[MultiRobotWCargoTrackingCost] Desired state size (" + 
                               std::to_string(desiredState.size()) + ") is smaller than expected stateDim (" + 
                               std::to_string(stateDim) + ")");
    }
    const vector_t xNominal = desiredState.head(stateDim);
    
    if (state.size() < stateDim) {
      throw std::runtime_error("[MultiRobotWCargoTrackingCost] State size (" + 
                               std::to_string(state.size()) + ") is smaller than expected stateDim (" + 
                               std::to_string(stateDim) + ")");
    }

    vector_t stateDeviation = state.head(stateDim) - xNominal;

    vector_t uNominal(input.rows());
    uNominal.setZero();
    
    // Compute force average per stance leg
    const size_t totalStanceLegs = std::count(contactFlags.begin(), contactFlags.end(), true);
    if (totalStanceLegs > 0) {
      vector3_t forceAverage(0.0, 0.0, (robot_mass_ * numRobots_ + cargo_mass_) * 9.81 / static_cast<scalar_t>(totalStanceLegs));
      
      // Set forces for each robot's feet
      for (size_t robot = 0; robot < numRobots_; ++robot) {
        for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
          const size_t contactIndex = robot * QUADRUPED_FOOT_NUM + ee;
          if (contactFlags[contactIndex]) {
            uNominal.segment(robot * SINGLE_ROBOT_INPUT_DIM + ee * QUADRUPED_CONTACT_DIM, 3) = forceAverage;
          }
        }
      }
    }

    // Get the average manipulation force per robot
    const vector3_t manipulationForceAverage(0.0, 0.0, cargo_mass_ * 9.81 / static_cast<scalar_t>(numRobots_));
    for (size_t arm = 0; arm < numRobots_; ++arm) {
      uNominal.segment(numRobots_ * SINGLE_ROBOT_INPUT_DIM + arm * ARM_CONTACT_DIM, 3) = manipulationForceAverage;
    }
    
    return {stateDeviation, input - uNominal};
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t robot_mass_;
  const scalar_t cargo_mass_;
  const size_t numRobots_;
};

/**
 * State tracking cost used for the final time
 */
class MultiRobotWCargoStateTrackingCost final : public QuadraticStateCost {
public:
  MultiRobotWCargoStateTrackingCost(matrix_t Q, size_t numRobots, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
          : QuadraticStateCost(std::move(Q)), numRobots_(numRobots), referenceManagerPtr_(&referenceManager) {}

  ~MultiRobotWCargoStateTrackingCost() override = default;
  MultiRobotWCargoStateTrackingCost* clone() const override { return new MultiRobotWCargoStateTrackingCost(*this); }

private:
  MultiRobotWCargoStateTrackingCost(const MultiRobotWCargoStateTrackingCost& rhs) = default;

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const size_t stateDim = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
    const vector_t desiredState = targetTrajectories.getDesiredState(time);
    if (desiredState.size() < stateDim) {
      throw std::runtime_error("[MultiRobotWCargoStateTrackingCost] Desired state size (" + 
                               std::to_string(desiredState.size()) + ") is smaller than expected stateDim (" + 
                               std::to_string(stateDim) + ")");
    }
    const vector_t xNominal = desiredState.head(stateDim);
    if (state.size() < stateDim) {
      throw std::runtime_error("[MultiRobotWCargoStateTrackingCost] State size (" + 
                               std::to_string(state.size()) + ") is smaller than expected stateDim (" + 
                               std::to_string(stateDim) + ")");
    }
    return state.head(stateDim) - xNominal;
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const size_t numRobots_;
};

}  // namespace multi_robot
}  // namespace ocs2

