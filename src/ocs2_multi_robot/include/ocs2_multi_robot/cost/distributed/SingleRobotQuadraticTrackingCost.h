#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include <ocs2_quadruped/common/utils.h>
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/cost/distributed/AlternatingQuadraticTrackingCost.h"
#include "ocs2_multi_robot/AlternatingPreComputation.h"

namespace ocs2 {
namespace multi_robot {

/**
 * State-input tracking cost used for intermediate times
 */
class SingleRobotQuadraticTrackingCost final : public AlternatingQuadraticTrackingCost {
public:
  SingleRobotQuadraticTrackingCost(matrix_t Q, matrix_t R, vector_t rho, size_t robotId,
                             const scalar_t& mass, const scalar_t& cargoMass, size_t numRobots,
                             const SwitchedModelReferenceManagerWithTerrain& referenceManager)
    : AlternatingQuadraticTrackingCost(std::move(Q), std::move(R), std::move(rho), mass, referenceManager) {
    robotId_ = robotId;
    cargoMass_ = cargoMass;
    numRobots_ = numRobots;
  }
  ~SingleRobotQuadraticTrackingCost() override = default;
  SingleRobotQuadraticTrackingCost* clone() const override { return new SingleRobotQuadraticTrackingCost(*this); }

private:
  SingleRobotQuadraticTrackingCost(const SingleRobotQuadraticTrackingCost& rhs) = default;

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories) const override {
    const vector_t fullState = targetTrajectories.getDesiredState(time);
    const vector_t xNominal = fullState.segment(robotId_ * SINGLE_ROBOT_STATE_DIM, SINGLE_ROBOT_STATE_DIM);
    
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const size_t offset = robotId_ * QUADRUPED_FOOT_NUM;
    const quadruped::contact_flag_t contactFlagsSingleRobot = quadruped::contact_flag_t{contactFlags[offset + 0], contactFlags[offset + 1], contactFlags[offset + 2], contactFlags[offset + 3]};
    const auto numStanceLegs = quadruped::numberOfClosedContacts(contactFlagsSingleRobot);
    vector_t uNominal(input.rows());
    uNominal.setZero();
    
    // Leg forces: average force per stance leg
    vector3_t legForceAverage(0.0, 0.0, (mass_ + cargoMass_ / static_cast<scalar_t>(numRobots_)) * 9.81 / static_cast<scalar_t>(numStanceLegs));
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      if (contactFlags[offset + ee])
        uNominal.segment(ee*QUADRUPED_CONTACT_DIM, 3) = legForceAverage;
    }
    
    // Arm force: negative average force (force applied ON the robot, opposite to what robot applies)
    // The cargo weight is distributed among all arms (NUM_ROBOTS), so each arm carries cargoMass * g / NUM_ROBOTS
    // For a single robot, the force applied ON it is negative (opposite direction to what the robot applies)
    // Only set the force part (first 3 elements), not the torque part
    vector3_t armForceAverage(0.0, 0.0, -cargoMass_ * 9.81 / static_cast<scalar_t>(numRobots_));
    uNominal.segment(SINGLE_ROBOT_INPUT_DIM, 3) = armForceAverage;

    return {state - xNominal, input - uNominal};
  }

  std::pair<std::vector<matrix_t>, std::vector<vector_t>>
    getAugmentedLagrangianInputDeviations(scalar_t time, const vector_t& input, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const override {
    // No state AL terms for cargo tracking
    const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
    std::vector<vector_t> augmentedLagrangianInputDeviations;
    std::vector<matrix_t> augmentedLagrangianInputCoefficients;

    matrix_t A_i = matrix_t::Zero(ARM_CONTACT_DIM, input.rows());
    A_i.block(0, SINGLE_ROBOT_INPUT_DIM, ARM_CONTACT_DIM, ARM_CONTACT_DIM) = matrix_t::Identity(ARM_CONTACT_DIM, ARM_CONTACT_DIM);
    augmentedLagrangianInputCoefficients.push_back(A_i);

    // The deviation is: u_i^robot + u_i^cargo + w_i
    // where u_i^robot is the arm input for this robot (at SINGLE_ROBOT_INPUT_DIM)
    // and u_i^cargo is the corresponding arm input from cargo (at robotId_*ARM_CONTACT_DIM in cargo input)
    // According to the base class comment, ALInputDeviations[i] should be (B_i * u_i - u_augmented_i),
    // which is the extracted segment (ARM_CONTACT_DIM,), not a full-size vector.
    const auto& prevCargoInput = preCompAlternating.getPrevCargoInput();
    const auto& prevDualVar = preCompAlternating.getPrevDualVariable(robotId_);
    
    // Debug: check dimensions and values
    const size_t cargoInputDim = preCompAlternating.getNumRobots() * ARM_CONTACT_DIM;
    if (prevCargoInput.size() != cargoInputDim) {
      throw std::runtime_error("[SingleRobotQuadraticTrackingCost] prevCargoInput size mismatch: expected " + 
                               std::to_string(cargoInputDim) + ", got " + std::to_string(prevCargoInput.size()));
    }
    if (robotId_ * ARM_CONTACT_DIM + ARM_CONTACT_DIM > prevCargoInput.size()) {
      throw std::runtime_error("[SingleRobotQuadraticTrackingCost] prevCargoInput index out of bounds: robotId=" + 
                               std::to_string(robotId_) + ", offset=" + std::to_string(robotId_ * ARM_CONTACT_DIM));
    }
    
    const vector_t robotArmInput = input.segment(SINGLE_ROBOT_INPUT_DIM, ARM_CONTACT_DIM);
    const vector_t cargoArmInput = prevCargoInput.segment(robotId_ * ARM_CONTACT_DIM, ARM_CONTACT_DIM);
    
    // Debug: print values to diagnose Robot 1 dual variable issue
    // The consensus constraint is: u_i^robot + u_i^cargo = 0
    // So the augmented Lagrangian term is: u_i^robot + u_i^cargo + w_i
    const vector_t consensusResidual = robotArmInput + cargoArmInput;
    const vector_t augmentedLagTerm = consensusResidual + prevDualVar;
    
    // Store the AL term as the extracted segment (ARM_CONTACT_DIM,), not as a full-size vector
    // This matches the mathematical definition: B_i * u_i - u_augmented_i
    augmentedLagrangianInputDeviations.push_back(augmentedLagTerm);

    return {augmentedLagrangianInputCoefficients, augmentedLagrangianInputDeviations};
  };

  size_t numArm_;
  size_t robotId_;
  size_t numRobots_;
  scalar_t cargoMass_;

};

/**
 * State tracking cost used for the final time
 */
class SingleRobotQuadraticStateTrackingCost final : public QuadraticStateCost {
public:
  SingleRobotQuadraticStateTrackingCost(matrix_t Q, size_t robotId, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
          : QuadraticStateCost(std::move(Q)), robotId_(robotId), referenceManagerPtr_(&referenceManager) {}

  ~SingleRobotQuadraticStateTrackingCost() override = default;
  SingleRobotQuadraticStateTrackingCost* clone() const override { return new SingleRobotQuadraticStateTrackingCost(*this); }

private:
  SingleRobotQuadraticStateTrackingCost(const SingleRobotQuadraticStateTrackingCost& rhs) = default;

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const vector_t xNominal = targetTrajectories.getDesiredState(time).segment(robotId_ * SINGLE_ROBOT_STATE_DIM, SINGLE_ROBOT_STATE_DIM);
        // if (time < 0.01 && robotId_ == 1) {
      // std::cerr << "[SingleRobotQuadraticStateTrackingCost] Robot " << robotId_ 
      //           << " xNominal(0:2)=" << xNominal.head(3).transpose()
      //           << " state(0:2)=" << state.head(3).transpose()
      //           << std::endl;
    return state - xNominal;
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const size_t robotId_;
};

}  // namespace multi_robot
}  // namespace ocs2