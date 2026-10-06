#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/StateCost.h>

#include <ocs2_quadruped/common/utils.h>
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

/**
 * State tracking cost used for the final time
 * cost = 0.5 * x' Q x + 
 *        sum_i(0.5 * rho_i * (x_i - x_augmented_i)^2)
 */
class AlternatingQuadraticStateTrackingCost : public StateCost {
public:
  AlternatingQuadraticStateTrackingCost(matrix_t Q, vector_t rho, const scalar_t& mass,
                                        const SwitchedModelReferenceManagerWithTerrain& referenceManager);
  ~AlternatingQuadraticStateTrackingCost() override = default;
  AlternatingQuadraticStateTrackingCost* clone() const override;

  /** Get cost term value */
  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const final;

  /** Get cost term quadratic approximation */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                  const TargetTrajectories& targetTrajectories,
                                                                  const PreComputation& preComp) const final;

protected:
  AlternatingQuadraticStateTrackingCost(const AlternatingQuadraticStateTrackingCost& rhs) = default;

  virtual vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const;

  virtual std::pair<std::vector<matrix_t>, std::vector<vector_t>> getAugmentedLagrangianStateDeviations(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
    return {std::vector<matrix_t>(), std::vector<vector_t>()};
  };

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t mass_;

private:
  size_t numDualVariables_;
  vector_t rho_;
  matrix_t Q_;
};

}  // namespace multi_robot
}  // namespace ocs2