#include "ocs2_multi_robot/constraint/distributed/SingleRobotManipulationFrictionConeConstraint.h"

#include <stdexcept>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace multi_robot {

namespace {
matrix3_t orientationFromEuler(const vector_t& pose) {
  if (pose.size() < 6) {
    throw std::runtime_error("[SingleRobotManipulationFrictionConeConstraint] Handle pose must contain position and orientation.");
  }
  const vector3_t orientation = pose.tail(3);
  return getRotationMatrixFromZyxEulerAngles<scalar_t>(orientation);
}
}  // namespace

SingleRobotManipulationFrictionConeConstraint::SingleRobotManipulationFrictionConeConstraint(
    FrictionConeConstraint::Config config, size_t robotIndex, const vector_t& handlePoseInCargoFrame)
    : StateInputConstraint(ConstraintOrder::Quadratic),
      config_(std::move(config)),
      robotIndex_(robotIndex),
      cargo_R_handle_(orientationFromEuler(handlePoseInCargoFrame)) {}

const AlternatingPreComputation& SingleRobotManipulationFrictionConeConstraint::castPreComputation(
    const PreComputation& preComp) const {
  const auto* altPreComp = dynamic_cast<const AlternatingPreComputation*>(&preComp);
  if (altPreComp == nullptr) {
    throw std::runtime_error("[SingleRobotManipulationFrictionConeConstraint] Expected AlternatingPreComputation.");
  }
  return *altPreComp;
}

vector3_t SingleRobotManipulationFrictionConeConstraint::getForceInWorld(const vector_t& input) const {
  const size_t offset = SINGLE_ROBOT_INPUT_DIM;
  if (offset + 3 > input.size()) {
    throw std::runtime_error("[SingleRobotManipulationFrictionConeConstraint] Invalid arm force indexing.");
  }
  return input.segment(offset, 3);
}

matrix3_t SingleRobotManipulationFrictionConeConstraint::computeHandleWorldRotation(const PreComputation& preComp) const {
  const auto& alternatingPreComp = castPreComputation(preComp);
  // Use precomputation's cargo state directly
  const vector_t& cargoState = alternatingPreComp.getPrevCargoState();
  if (cargoState.size() < 9) {
    throw std::runtime_error("[SingleRobotManipulationFrictionConeConstraint] Cached cargo state too small.");
  }
  const vector3_t cargoEuler = cargoState.segment(6, 3);
  const matrix3_t w_R_cargo = getRotationMatrixFromZyxEulerAngles<scalar_t>(cargoEuler);
  const matrix3_t w_R_handle = w_R_cargo * cargo_R_handle_;
  // For robot: the force direction is opposite to cargo (robot pulls, cargo pushes)
  // The handle's z-axis points outward from cargo (normal direction for cargo)
  // For robot, we need to flip the z-axis so positive z in handle frame means robot is pulling
  // This ensures the friction cone constraint works correctly: mu * Fz >= sqrt(Fx^2 + Fy^2)
  // where Fz is positive when robot applies pulling force
  matrix3_t h_R_w = w_R_handle.transpose();
  // Flip z-axis: negate the third column (z-axis direction in handle frame)
  h_R_w.col(2) = -h_R_w.col(2);
  return h_R_w;
}

vector_t SingleRobotManipulationFrictionConeConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                                const PreComputation& preComp) const {
  const matrix3_t h_R_w = computeHandleWorldRotation(preComp);
  const vector3_t localForce = h_R_w * getForceInWorld(input);
  return coneConstraint(localForce);
}

VectorFunctionLinearApproximation SingleRobotManipulationFrictionConeConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  const matrix3_t h_R_w = computeHandleWorldRotation(preComp);
  const vector3_t localForce = h_R_w * getForceInWorld(input);

  const auto localForceDerivatives = computeLocalForceDerivatives(h_R_w);
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = coneConstraint(localForce);
  linearApproximation.dfdx = matrix_t::Zero(1, state.size());
  linearApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
  return linearApproximation;
}

VectorFunctionQuadraticApproximation SingleRobotManipulationFrictionConeConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  const matrix3_t h_R_w = computeHandleWorldRotation(preComp);
  const vector3_t localForce = h_R_w * getForceInWorld(input);

  const auto localForceDerivatives = computeLocalForceDerivatives(h_R_w);
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  VectorFunctionQuadraticApproximation approximation;
  approximation.f = coneConstraint(localForce);
  approximation.dfdx = matrix_t::Zero(1, state.size());
  approximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
  approximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(state.size()));
  approximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(input.size(), coneDerivatives));
  approximation.dfdux.emplace_back(matrix_t::Zero(input.size(), state.size()));
  return approximation;
}

vector_t SingleRobotManipulationFrictionConeConstraint::coneConstraint(const vector3_t& localForces) const {
  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);
  const scalar_t cone = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm;
  return (vector_t(1) << cone).finished();
}

SingleRobotManipulationFrictionConeConstraint::LocalForceDerivatives
SingleRobotManipulationFrictionConeConstraint::computeLocalForceDerivatives(const matrix3_t& h_R_w) const {
  LocalForceDerivatives derivatives;
  derivatives.dF_du = h_R_w;
  return derivatives;
}

SingleRobotManipulationFrictionConeConstraint::ConeLocalDerivatives
SingleRobotManipulationFrictionConeConstraint::computeConeLocalDerivatives(const vector3_t& localForces) const {
  const auto F_x_square = localForces.x() * localForces.x();
  const auto F_y_square = localForces.y() * localForces.y();
  const auto F_tangent_square = F_x_square + F_y_square + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);
  const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square;

  ConeLocalDerivatives coneDerivatives{};
  coneDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm;
  coneDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm;
  coneDerivatives.dCone_dF(2) = config_.frictionCoefficient;

  coneDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config_.regularization) / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 2) = 0.0;
  coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1);
  coneDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config_.regularization) / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(1, 2) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 0) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 1) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 2) = 0.0;

  return coneDerivatives;
}

SingleRobotManipulationFrictionConeConstraint::ConeDerivatives
SingleRobotManipulationFrictionConeConstraint::computeConeConstraintDerivatives(
    const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const {
  ConeDerivatives coneDerivatives;
  coneDerivatives.dCone_du.noalias() = localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.dCone_dF;
  coneDerivatives.d2Cone_du2.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;
  return coneDerivatives;
}

matrix_t SingleRobotManipulationFrictionConeConstraint::frictionConeInputDerivative(size_t inputDim,
                                                                                   const ConeDerivatives& coneDerivatives) const {
  matrix_t dhdu = matrix_t::Zero(1, inputDim);
  dhdu.block<1, 3>(0, SINGLE_ROBOT_INPUT_DIM) = coneDerivatives.dCone_du.transpose();
  return dhdu;
}

matrix_t SingleRobotManipulationFrictionConeConstraint::frictionConeSecondDerivativeInput(size_t inputDim,
                                                                                         const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdudu = matrix_t::Zero(inputDim, inputDim);
  ddhdudu.block<3, 3>(SINGLE_ROBOT_INPUT_DIM, SINGLE_ROBOT_INPUT_DIM) = coneDerivatives.d2Cone_du2;
  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdudu;
}

matrix_t SingleRobotManipulationFrictionConeConstraint::frictionConeSecondDerivativeState(size_t stateDim) const {
  matrix_t ddhdxdx = matrix_t::Zero(stateDim, stateDim);
  ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdxdx;
}

}  // namespace multi_robot
}  // namespace ocs2


