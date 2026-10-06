#include "ocs2_multi_robot/cost/distributed/AlternatingQuadraticStateTrackingCost.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AlternatingQuadraticStateTrackingCost::AlternatingQuadraticStateTrackingCost(matrix_t Q,
  vector_t rho, const scalar_t& mass,
  const SwitchedModelReferenceManagerWithTerrain& referenceManager)
    : Q_(std::move(Q)), rho_(std::move(rho)), mass_(mass), referenceManagerPtr_(&referenceManager) {
  numDualVariables_ = rho_.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AlternatingQuadraticStateTrackingCost* AlternatingQuadraticStateTrackingCost::clone() const {
  return new AlternatingQuadraticStateTrackingCost(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t AlternatingQuadraticStateTrackingCost::getValue(scalar_t time, const vector_t& state,
                                           const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  vector_t stateDeviation = getStateDeviation(time, state, targetTrajectories);

  scalar_t cost = 0.0;
  cost = 0.5 * stateDeviation.dot(Q_ * stateDeviation);
  const auto [ALStateCoefficients, ALStateDeviations] = getAugmentedLagrangianStateDeviations(time, state, targetTrajectories, preComp);
  if (ALStateCoefficients.size() != 0) {
    for (size_t i = 0; i < ALStateCoefficients.size(); i++) {
      cost += 0.5 * rho_[i] * ALStateDeviations[i].dot(ALStateDeviations[i]);
    }
  }
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation AlternatingQuadraticStateTrackingCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                        const TargetTrajectories& targetTrajectories,
                                                                                        const PreComputation& preComp) const {
  const vector_t stateDeviation = getStateDeviation(time, state, targetTrajectories);

  ScalarFunctionQuadraticApproximation L;
  L.dfdxx = Q_;
  L.dfdx.noalias() = Q_ * stateDeviation;
  L.f = 0.5 * stateDeviation.dot(L.dfdx);

  const auto [ALStateCoefficients, ALStateDeviations] = getAugmentedLagrangianStateDeviations(time, state, targetTrajectories, preComp);
  if (ALStateCoefficients.size() != 0) {
    for (size_t i = 0; i < ALStateCoefficients.size(); i++) {
      L.f += 0.5 * rho_[i] * ALStateDeviations[i].dot(ALStateDeviations[i]);
      // Gradient: d/dx [0.5 * rho * (A_i * x + deviation)^2] = rho * A_i^T * (A_i * x + deviation) = rho * A_i^T * ALStateDeviations[i]
      L.dfdx.noalias() += rho_[i] * ALStateCoefficients[i].transpose() * ALStateDeviations[i];
      // Add Hessian contribution: d^2/dx^2 [0.5 * rho * (A_i * x + deviation)^2] = rho * A_i^T * A_i
      L.dfdxx.noalias() += rho_[i] * ALStateCoefficients[i].transpose() * ALStateCoefficients[i];
    }
  }

  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t AlternatingQuadraticStateTrackingCost::getStateDeviation(scalar_t time, const vector_t& state,
                                                                              const TargetTrajectories& targetTrajectories) const {
  return state - targetTrajectories.getDesiredState(time);
}

} // namespace multi_robot
} // namespace ocs2