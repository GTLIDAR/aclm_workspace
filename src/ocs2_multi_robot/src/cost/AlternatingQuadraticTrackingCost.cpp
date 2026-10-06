#include "ocs2_multi_robot/cost/distributed/AlternatingQuadraticTrackingCost.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AlternatingQuadraticTrackingCost::AlternatingQuadraticTrackingCost(matrix_t Q, matrix_t R,
  vector_t rho, const scalar_t& mass,
  const SwitchedModelReferenceManagerWithTerrain& referenceManager)
    : Q_(std::move(Q)), R_(std::move(R)), rho_(std::move(rho)), mass_(mass), referenceManagerPtr_(&referenceManager) {
  numDualVariables_ = rho_.size();
  // std::cerr << "Q = " << Q_ << std::endl;
  // std::cerr << "R = " << R_ << std::endl;
  // std::cerr << "rho = " << rho_.transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AlternatingQuadraticTrackingCost* AlternatingQuadraticTrackingCost::clone() const {
  return new AlternatingQuadraticTrackingCost(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t AlternatingQuadraticTrackingCost::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                           const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  vector_t stateDeviation, inputDeviation;
  std::tie(stateDeviation, inputDeviation) = getStateInputDeviation(time, state, input, targetTrajectories);

  scalar_t cost = 0.0;
  cost = 0.5 * stateDeviation.dot(Q_ * stateDeviation) + 0.5 * inputDeviation.dot(R_ * inputDeviation);
  const auto [ALStateCoefficients, ALStateDeviations] = getAugmentedLagrangianStateDeviations(time, state, targetTrajectories, preComp);
  const auto [ALInputCoefficients, ALInputDeviations] = getAugmentedLagrangianInputDeviations(time, input, targetTrajectories, preComp);
  
  if (ALStateCoefficients.size() != 0) {
    for (size_t i = 0; i < ALStateCoefficients.size(); i++) {
      cost += 0.5 * rho_[i] * ALStateDeviations[i].dot(ALStateDeviations[i]);
    }
  }
  if (ALInputCoefficients.size() != 0) {
    for (size_t i = 0; i < ALInputCoefficients.size(); i++) {
      cost += 0.5 * rho_[i+ALStateCoefficients.size()] * ALInputDeviations[i].dot(ALInputDeviations[i]);
    }
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation AlternatingQuadraticTrackingCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const TargetTrajectories& targetTrajectories,
                                                                                        const PreComputation& preComp) const {
  vector_t stateDeviation, inputDeviation;
  std::tie(stateDeviation, inputDeviation) = getStateInputDeviation(time, state, input, targetTrajectories);

  ScalarFunctionQuadraticApproximation L;
  L.dfdxx = Q_;
  L.dfduu = R_;
  L.dfdux = matrix_t::Zero(R_.rows(), Q_.cols());
  L.dfdx.noalias() = Q_ * stateDeviation;
  L.dfdu.noalias() = R_ * inputDeviation;
  L.f = 0.5 * stateDeviation.dot(L.dfdx) + 0.5 * inputDeviation.dot(L.dfdu);

  const auto [ALStateCoefficients, ALStateDeviations] = getAugmentedLagrangianStateDeviations(time, state, targetTrajectories, preComp);
  const auto [ALInputCoefficients, ALInputDeviations] = getAugmentedLagrangianInputDeviations(time, input, targetTrajectories, preComp);
  if (ALStateCoefficients.size() != 0) {
    for (size_t i = 0; i < ALStateCoefficients.size(); i++) {
      L.f += 0.5 * rho_[i] * ALStateDeviations[i].dot(ALStateDeviations[i]);
      // Gradient: d/dx [0.5 * rho * (A_i * x + deviation)^2] = rho * A_i^T * (A_i * x + deviation) = rho * A_i^T * ALStateDeviations[i]
      L.dfdx.noalias() += rho_[i] * ALStateCoefficients[i].transpose() * ALStateDeviations[i];
      // Add Hessian contribution: d^2/dx^2 [0.5 * rho * (A_i * x + deviation)^2] = rho * A_i^T * A_i
      L.dfdxx.noalias() += rho_[i] * ALStateCoefficients[i].transpose() * ALStateCoefficients[i];
    }
  }
  if (ALInputCoefficients.size() != 0) {
    for (size_t i = 0; i < ALInputCoefficients.size(); i++) {
      L.f += 0.5 * rho_[i+ALStateCoefficients.size()] * ALInputDeviations[i].dot(ALInputDeviations[i]);
      // Gradient: d/du [0.5 * rho * (B_i * u + deviation)^2] = rho * B_i^T * (B_i * u + deviation) = rho * B_i^T * ALInputDeviations[i]
      L.dfdu.noalias() += rho_[i+ALStateCoefficients.size()] * ALInputCoefficients[i].transpose() * ALInputDeviations[i];
      // Add Hessian contribution: d^2/du^2 [0.5 * rho * (B_i * u + deviation)^2] = rho * B_i^T * B_i
      L.dfduu.noalias() += rho_[i+ALStateCoefficients.size()] * ALInputCoefficients[i].transpose() * ALInputCoefficients[i];
    }
  }
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, vector_t> AlternatingQuadraticTrackingCost::getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const TargetTrajectories& targetTrajectories) const {
  const vector_t stateDeviation = state - targetTrajectories.getDesiredState(time);
  const vector_t inputDeviation = input - targetTrajectories.getDesiredInput(time);
  return {stateDeviation, inputDeviation};
}

} // namespace multi_robot
} // namespace ocs2