#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/StateInputCost.h>

#include <ocs2_quadruped/common/utils.h>
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

/**
 * State-input tracking cost used for intermediate times with Augmented Lagrangian terms
 * cost = 0.5 * x' Q x + 0.5 * u' R u +
 *        sum_i(0.5 * rho_i * (A_i * x_i - x_augmented_i)^2) + sum_j(0.5 * rho_j * (B_j * u_j - u_augmented_j)^2)
 */
class AlternatingQuadraticTrackingCost : public StateInputCost {
public:
  AlternatingQuadraticTrackingCost(matrix_t Q, matrix_t R, vector_t rho,
                                   const scalar_t& mass,
                                   const SwitchedModelReferenceManagerWithTerrain& referenceManager);
  ~AlternatingQuadraticTrackingCost() override = default;
  AlternatingQuadraticTrackingCost* clone() const override;

  /** Get cost term value */
  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                    const PreComputation&) const override;

  /** Get cost term quadratic approximation */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                    const TargetTrajectories& targetTrajectories,
                                                    const PreComputation&) const final;

protected:
  AlternatingQuadraticTrackingCost(const AlternatingQuadraticTrackingCost& rhs) = default;

  // Here derivation just means the difference between desired value and actual value, not 'dy/dx'
  virtual std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const TargetTrajectories& targetTrajectories) const;

  /** Get augmented Lagrangian state deviations */
  /** Return: {A_i, A_i * x_i - x_augmented_i} */
  virtual std::pair<std::vector<matrix_t>, std::vector<vector_t>> getAugmentedLagrangianStateDeviations(scalar_t time, const vector_t& state,
                                                        const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
    return {std::vector<matrix_t>(), std::vector<vector_t>()};
  };

  /** Get augmented Lagrangian input deviations */
  /** Return: {B_i, B_i * u_i - u_augmented_i} */
  virtual std::pair<std::vector<matrix_t>, std::vector<vector_t>> getAugmentedLagrangianInputDeviations(scalar_t time, const vector_t& input,
                                                        const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
    return {std::vector<matrix_t>(), std::vector<vector_t>()};
  };

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t mass_;

private:
  size_t numDualVariables_;
  vector_t rho_;
  matrix_t Q_;
  matrix_t R_;
};

}  // namespace multi_robot
}  // namespace ocs2