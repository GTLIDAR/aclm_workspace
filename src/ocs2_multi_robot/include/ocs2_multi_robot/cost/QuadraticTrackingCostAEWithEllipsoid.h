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
class QuadraticTrackingCostAEWithEllipsoid final : public QuadraticStateInputCost {
public:
  QuadraticTrackingCostAEWithEllipsoid(matrix_t Q, matrix_t R, const scalar_t& mass,
                          const SwitchedModelReferenceManagerWithTerrain& referenceManager)
          : QuadraticStateInputCost(std::move(Q), std::move(R)), mass_(mass), referenceManagerPtr_(&referenceManager) {}

  ~QuadraticTrackingCostAEWithEllipsoid() override = default;
  QuadraticTrackingCostAEWithEllipsoid* clone() const override { return new QuadraticTrackingCostAEWithEllipsoid(*this); }

private:
  QuadraticTrackingCostAEWithEllipsoid(const QuadraticTrackingCostAEWithEllipsoid& rhs) = default;

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const quadruped::contact_flag_t contactFlagsSingleRobot = quadruped::contact_flag_t{contactFlags[0], contactFlags[1], contactFlags[2], contactFlags[3]};
    const auto numStanceLegs = quadruped::numberOfClosedContacts(contactFlagsSingleRobot);
    // Always track the first 24 elements for all formulations
    const vector_t xNominal = targetTrajectories.getDesiredState(time).head(STATE_DIM_AE_WITH_ELLIPSOID);

    vector_t uNominal(input.rows());
    uNominal.setZero();
    vector3_t forceAverage(0.0, 0.0, mass_ * 9.81 / numStanceLegs);
    for (size_t ee = 0; ee < ROBOTS_FOOT_NUM; ee++) {
      if (contactFlags[ee])
        uNominal.segment(ee*QUADRUPED_CONTACT_DIM, 3) = forceAverage;
    }
    return {state.head(STATE_DIM_AE_WITH_ELLIPSOID) - xNominal, input - uNominal};
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t mass_;
};

/**
 * State tracking cost used for the final time
 */
class QuadraticStateTrackingCostAEWithEllipsoid final : public QuadraticStateCost {
public:
  QuadraticStateTrackingCostAEWithEllipsoid(matrix_t Q, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
          : QuadraticStateCost(std::move(Q)), referenceManagerPtr_(&referenceManager) {}

  ~QuadraticStateTrackingCostAEWithEllipsoid() override = default;
  QuadraticStateTrackingCostAEWithEllipsoid* clone() const override { return new QuadraticStateTrackingCostAEWithEllipsoid(*this); }

private:
  QuadraticStateTrackingCostAEWithEllipsoid(const QuadraticStateTrackingCostAEWithEllipsoid& rhs) = default;

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const vector_t xNominal = targetTrajectories.getDesiredState(time).head(STATE_DIM_AE_WITH_ELLIPSOID);
    return state.head(STATE_DIM_AE_WITH_ELLIPSOID) - xNominal;
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
};

}  // namespace multi_robot
}  // namespace ocs2