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
class CargoQuadraticTrackingCost final : public AlternatingQuadraticTrackingCost {
public:
  CargoQuadraticTrackingCost(matrix_t Q, matrix_t R, vector_t rho, size_t numArm,
                             const scalar_t& mass, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
    : AlternatingQuadraticTrackingCost(Q, R, rho, mass, referenceManager) {
    numArm_ = numArm;
    if (rho.size() != numArm) {
      throw std::runtime_error("[CargoQuadraticTrackingCost] The number of dual variables does not match the number of arms!");
    }
  }
  ~CargoQuadraticTrackingCost() override = default;
  CargoQuadraticTrackingCost* clone() const override { return new CargoQuadraticTrackingCost(*this); }

private:
  CargoQuadraticTrackingCost(const CargoQuadraticTrackingCost& rhs) = default;

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories) const override {
    const vector_t xNominal = targetTrajectories.getDesiredState(time).tail(CARGO_STATE_DIM);
    vector_t uNominal(input.rows());
    uNominal.setZero();
    vector3_t forceAverage(0.0, 0.0, mass_ * 9.81 / numArm_);
    for (size_t i = 0; i < numArm_; i++) {
      uNominal.segment(i*ARM_CONTACT_DIM, 3) = forceAverage;
    }
    return {state - xNominal, input - uNominal};
  }

  std::pair<std::vector<matrix_t>, std::vector<vector_t>>
    getAugmentedLagrangianInputDeviations(scalar_t time, const vector_t& input, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const override {
    // No state AL terms for cargo tracking
    const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
    std::vector<vector_t> augmentedLagrangianInputDeviations;
    std::vector<matrix_t> augmentedLagrangianInputCoefficients;

    for (size_t i = 0; i < numArm_; i++) {
      matrix_t A_i = matrix_t::Zero(ARM_CONTACT_DIM, input.rows());
      A_i.block(0, i*ARM_CONTACT_DIM, ARM_CONTACT_DIM, ARM_CONTACT_DIM) = matrix_t::Identity(ARM_CONTACT_DIM, ARM_CONTACT_DIM);
      augmentedLagrangianInputCoefficients.push_back(A_i);
      
      // Consensus constraint: u_i^cargo + u_i^robot = 0
      // AL term: u_i^cargo + u_i^robot + w_i
      // where:
      //   u_i^cargo is the cargo input segment for robot i: input.segment(i*ARM_CONTACT_DIM, ARM_CONTACT_DIM)
      //   u_i^robot is the robot i's arm force from previous solution: getPrevRobotInput(i).segment(SINGLE_ROBOT_INPUT_DIM, ARM_CONTACT_DIM)
      //   w_i is the dual variable for robot i: getPrevDualVariable(i)
      const vector_t cargoArmInput = input.segment(i*ARM_CONTACT_DIM, ARM_CONTACT_DIM);
      const vector_t robotArmInput = preCompAlternating.getPrevRobotInput(i).segment(SINGLE_ROBOT_INPUT_DIM, ARM_CONTACT_DIM);
      const vector_t dualVar = preCompAlternating.getPrevDualVariable(i);
      const vector_t augmentedLagTerm = cargoArmInput + robotArmInput + dualVar;
      
      // Store the AL term as the extracted segment (ARM_CONTACT_DIM,), not as a full-size vector
      // This matches the mathematical definition: B_i * u_i - u_augmented_i
      augmentedLagrangianInputDeviations.push_back(augmentedLagTerm);
    }
    return {augmentedLagrangianInputCoefficients, augmentedLagrangianInputDeviations};
  };

  size_t numArm_;

};

/**
 * State tracking cost used for the final time
 */
class CargoQuadraticStateTrackingCost final : public QuadraticStateCost {
public:
  CargoQuadraticStateTrackingCost(matrix_t Q, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
          : QuadraticStateCost(Q), referenceManagerPtr_(&referenceManager) {}

  ~CargoQuadraticStateTrackingCost() override = default;
  CargoQuadraticStateTrackingCost* clone() const override { return new CargoQuadraticStateTrackingCost(*this); }

private:
  CargoQuadraticStateTrackingCost(const CargoQuadraticStateTrackingCost& rhs) = default;

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const vector_t xNominal = targetTrajectories.getDesiredState(time).tail(CARGO_STATE_DIM);
    return state - xNominal;
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
};

}  // namespace multi_robot
}  // namespace ocs2