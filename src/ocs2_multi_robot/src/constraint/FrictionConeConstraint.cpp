/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_multi_robot/constraint/FrictionConeConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace multi_robot {


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::FrictionConeConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager, Config config,
                                               size_t contactPointIndex, size_t index, size_t stateOffset)
    : StateInputConstraint(ConstraintOrder::Quadratic),
      referenceManagerPtr_(&referenceManager),
      heightMapPtr_(std::move(referenceManager.getHeightMap())),
      config_(std::move(config)),
      Index_(index),
      contactPointIndex_(contactPointIndex),
      robotStateOffset_(stateOffset){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrictionConeConstraint::setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {
  t_R_w_.setIdentity();
  throw std::runtime_error("[FrictionConeConstraint] setSurfaceNormalInWorld() is not implemented!");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix3_t FrictionConeConstraint::setRotationMatrixOfTerrain(scalar_t x, scalar_t y) {
  matrix3_t w_R_t;
  w_R_t.col(0) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Tangent1, x, y);
  w_R_t.col(1) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Tangent2, x, y);
  w_R_t.col(2) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Normal, x, y);
  return w_R_t;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FrictionConeConstraint::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FrictionConeConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                          const PreComputation& preComp) const {
  const auto forcesInWorldFrame = input.segment(Index_*QUADRUPED_CONTACT_DIM, QUADRUPED_CONTACT_DIM);
  matrix3_t w_R_t;
  const size_t footInRobot = contactPointIndex_ % QUADRUPED_FOOT_NUM;
  const size_t footStateOffset = robotStateOffset_ + 12 + footInRobot * QUADRUPED_CONTACT_DIM;
  scalar_t pos_x = state[footStateOffset];
  scalar_t pos_y = state[footStateOffset + 1];
  w_R_t.col(0) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Tangent1, pos_x, pos_y);
  w_R_t.col(1) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Tangent2, pos_x, pos_y);
  w_R_t.col(2) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Normal, pos_x, pos_y);
  const vector3_t localForce = w_R_t.transpose() * forcesInWorldFrame;
  return coneConstraint(localForce);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation FrictionConeConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const PreComputation& preComp) const {
  const vector3_t forcesInWorldFrame = input.segment(Index_*QUADRUPED_CONTACT_DIM, QUADRUPED_CONTACT_DIM);
  matrix3_t w_R_t;
  const size_t footInRobot = contactPointIndex_ % QUADRUPED_FOOT_NUM;
  const size_t footStateOffset = robotStateOffset_ + 12 + footInRobot * QUADRUPED_CONTACT_DIM;
  scalar_t pos_x = state[footStateOffset];
  scalar_t pos_y = state[footStateOffset + 1];
  w_R_t.col(0) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Tangent1, pos_x, pos_y);
  w_R_t.col(1) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Tangent2, pos_x, pos_y);
  w_R_t.col(2) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Normal, pos_x, pos_y);
  const vector3_t localForce = w_R_t.transpose() * forcesInWorldFrame;

  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame, w_R_t.transpose());
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = coneConstraint(localForce);
  linearApproximation.dfdx = matrix_t::Zero(1, state.size());
  linearApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation FrictionConeConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                       const vector_t& input,
                                                                                       const PreComputation& preComp) const {
  const vector3_t forcesInWorldFrame = input.segment(Index_*QUADRUPED_CONTACT_DIM, QUADRUPED_CONTACT_DIM);
  matrix3_t w_R_t;
  const size_t footInRobot = contactPointIndex_ % QUADRUPED_FOOT_NUM;
  const size_t footStateOffset = robotStateOffset_ + 12 + footInRobot * QUADRUPED_CONTACT_DIM;
  scalar_t pos_x = state[footStateOffset];
  scalar_t pos_y = state[footStateOffset + 1];
  w_R_t.col(0) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Tangent1, pos_x, pos_y);
  w_R_t.col(1) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Tangent2, pos_x, pos_y);
  w_R_t.col(2) = heightMapPtr_->GetNormalizedBasis(HeightMap::Direction::Normal, pos_x, pos_y);
  const vector3_t localForce = w_R_t.transpose() * forcesInWorldFrame;

  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame, w_R_t.transpose());
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f = coneConstraint(localForce);
  quadraticApproximation.dfdx = matrix_t::Zero(1, state.size());
  quadraticApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
  quadraticApproximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(state.size(), coneDerivatives));
  quadraticApproximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(input.size(), coneDerivatives));
  quadraticApproximation.dfdux.emplace_back(matrix_t::Zero(input.size(), state.size()));
  return quadraticApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::LocalForceDerivatives FrictionConeConstraint::computeLocalForceDerivatives(
    const vector3_t& forcesInWorldFrame, const matrix3_t& t_R_w) const {
  LocalForceDerivatives localForceDerivatives{};
  localForceDerivatives.dF_du = t_R_w;
  return localForceDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::ConeLocalDerivatives FrictionConeConstraint::computeConeLocalDerivatives(const vector3_t& localForces) const {
  const auto F_x_square = localForces.x() * localForces.x();
  const auto F_y_square = localForces.y() * localForces.y();
  const auto F_tangent_square = F_x_square + F_y_square + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);
  const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square;  // = F_tangent_square ^ (3/2)

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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FrictionConeConstraint::coneConstraint(const vector3_t& localForces) const {
  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);
  const scalar_t coneConstraint = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm;
  return (vector_t(1) << coneConstraint).finished();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::ConeDerivatives FrictionConeConstraint::computeConeConstraintDerivatives(
    const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const {
  ConeDerivatives coneDerivatives;
  // First order derivatives
  coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

  // Second order derivatives
  coneDerivatives.d2Cone_du2.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;

  return coneDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t dhdu = matrix_t::Zero(1, inputDim);
  dhdu.block<1, 3>(0, 3 * Index_) = coneDerivatives.dCone_du;
  return dhdu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdudu = matrix_t::Zero(inputDim, inputDim);
  ddhdudu.block<3, 3>(3 * Index_, 3 * Index_) = coneDerivatives.d2Cone_du2;
  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdudu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdxdx = matrix_t::Zero(stateDim, stateDim);
  ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdxdx;
}

}  // namespace multi_robot
}  // namespace ocs2
