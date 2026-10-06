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
class TwoQuadWCargoTrackingCost final : public QuadraticStateInputCost {
public:
  TwoQuadWCargoTrackingCost(matrix_t Q, matrix_t R, const scalar_t& robot_mass, const scalar_t& cargo_mass, 
                          const SwitchedModelReferenceManagerWithTerrain& referenceManager)
          : QuadraticStateInputCost(std::move(Q), std::move(R)), robot_mass_(robot_mass), cargo_mass_(cargo_mass), referenceManagerPtr_(&referenceManager) {}

  ~TwoQuadWCargoTrackingCost() override = default;
  TwoQuadWCargoTrackingCost* clone() const override { return new TwoQuadWCargoTrackingCost(*this); }

private:
  TwoQuadWCargoTrackingCost(const TwoQuadWCargoTrackingCost& rhs) = default;

  // Here derivation just means the difference between desired value and actual value, not 'dy/dx'
  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const quadruped::contact_flag_t contactFlagsSingleRobot = quadruped::contact_flag_t{contactFlags[0], contactFlags[1], contactFlags[2], contactFlags[3]};
    const auto numStanceLegs = quadruped::numberOfClosedContacts(contactFlagsSingleRobot);
    // Always track the first 24 elements for all formulations
    const vector_t xNominal = targetTrajectories.getDesiredState(time).head(STATE_DIM_TWO_QUAD_W_CARGO);

    // ! Attention here
    vector_t uNominal(input.rows());
    uNominal.setZero();
    vector3_t forceAverage(0.0, 0.0, (robot_mass_ + cargo_mass_ / 2.0) * 9.81 / numStanceLegs);
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      if (contactFlags[ee])
        uNominal.segment(ee*QUADRUPED_CONTACT_DIM, 3) = forceAverage;
    }
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      if (contactFlags[ee])
        uNominal.segment(ee*QUADRUPED_CONTACT_DIM+24, 3) = forceAverage;
    }
    uNominal[50] = (cargo_mass_ * 9.81) / 2.0; // cargo vertical force
    uNominal[56] = (cargo_mass_ * 9.81) / 2.0; // cargo vertical force
    return {state.head(STATE_DIM_TWO_QUAD_W_CARGO) - xNominal, input - uNominal};
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t robot_mass_;
  const scalar_t cargo_mass_;
};

/**
 * State tracking cost used for the final time
 */
class TwoQuadWCargoStateTrackingCostAE final : public QuadraticStateCost {
public:
  TwoQuadWCargoStateTrackingCostAE(matrix_t Q, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
          : QuadraticStateCost(std::move(Q)), referenceManagerPtr_(&referenceManager) {}

  ~TwoQuadWCargoStateTrackingCostAE() override = default;
  TwoQuadWCargoStateTrackingCostAE* clone() const override { return new TwoQuadWCargoStateTrackingCostAE(*this); }

private:
  TwoQuadWCargoStateTrackingCostAE(const TwoQuadWCargoStateTrackingCostAE& rhs) = default;

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const vector_t xNominal = targetTrajectories.getDesiredState(time).head(STATE_DIM_TWO_QUAD_W_CARGO);
    return state.head(STATE_DIM_TWO_QUAD_W_CARGO) - xNominal;
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
};

}  // namespace multi_robot
}  // namespace ocs2