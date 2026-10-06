#include "ocs2_multi_robot/constraint/distributed/SingleRobotPerceptiveFootPlacementConstraint.h"
#include <ros/ros.h>

namespace ocs2 {
namespace multi_robot {

SingleRobotPerceptiveFootPlacementConstraint::SingleRobotPerceptiveFootPlacementConstraint(
    SwitchedModelReferenceManagerWithTerrain& referenceManager,
    MultiRobotConvexRegionSelector& convexRegionSelector,
    size_t robotIndex,
    size_t localFootIndex,
    size_t numVertices)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      convexRegionSelectorPtr_(&convexRegionSelector),
      robotIndex_(robotIndex),
      localFootIndex_(localFootIndex),
      globalFootIndex_(robotIndex * QUADRUPED_FOOT_NUM + localFootIndex),
      numVertices_(numVertices) {}

bool SingleRobotPerceptiveFootPlacementConstraint::isActive(scalar_t time) const {
  bool isStance = referenceManagerPtr_->getContactFlags(time)[globalFootIndex_];
  bool hasRegion = convexRegionSelectorPtr_->isFootPlacementActive(globalFootIndex_, time);
  return isStance && hasRegion;
}

vector3_t SingleRobotPerceptiveFootPlacementConstraint::getFootPositionFromState(const vector_t& state) const {
  const size_t footPosOffset = 12 + localFootIndex_ * QUADRUPED_CONTACT_DIM;
  return state.segment<3>(footPosOffset);
}

const FootPlacementParameters& SingleRobotPerceptiveFootPlacementConstraint::getParameters(
    scalar_t time, const PreComputation& preComp) const {
  const auto* preCompPtr = dynamic_cast<const AlternatingPerceptivePreComputation*>(&preComp);
  if (preCompPtr != nullptr) {
    return preCompPtr->getFootPlacementParameters(globalFootIndex_);
  }
  
  fallbackParam_ = computeParametersFallback(time);
  return fallbackParam_;
}

FootPlacementParameters SingleRobotPerceptiveFootPlacementConstraint::computeParametersFallback(scalar_t time) const {
  FootPlacementParameters param;
  
  auto projection = convexRegionSelectorPtr_->getProjection(globalFootIndex_, time);
  if (projection.regionPtr == nullptr) {
    param.valid = false;
    return param;
  }

  auto polygon = convexRegionSelectorPtr_->getConvexPolygon(globalFootIndex_, time);
  if (polygon.is_empty()) {
    param.valid = false;
    return param;
  }

  matrix_t polytopeA;
  vector_t polytopeB;
  std::tie(polytopeA, polytopeB) = getPolygonConstraint(polygon);
  
  matrix_t p_xy = (matrix_t(2, 3) << 1, 0, 0, 0, 1, 0).finished();
  
  const auto& T_pw = projection.regionPtr->transformPlaneToWorld;
  matrix3_t R_inv = T_pw.inverse().linear();
  vector3_t t_inv = T_pw.inverse().translation();
  
  param.a = polytopeA * p_xy * R_inv;
  param.b = polytopeB + polytopeA * (p_xy * t_inv);
  param.valid = true;
  
  return param;
}

vector_t SingleRobotPerceptiveFootPlacementConstraint::getValue(scalar_t time, const vector_t& state, 
                                                                 const vector_t& input,
                                                                 const PreComputation& preComp) const {
  const auto& param = getParameters(time, preComp);
  
  if (!param.valid) {
    return vector_t::Constant(numVertices_, 1.0);
  }

  vector3_t footPos = getFootPositionFromState(state);
  return param.a * footPos + param.b;
}

VectorFunctionLinearApproximation SingleRobotPerceptiveFootPlacementConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(numVertices_, state.size());
  approx.dfdu = matrix_t::Zero(numVertices_, input.size());

  const auto& param = getParameters(time, preComp);
  if (!param.valid) {
    return approx;
  }

  const size_t footPosOffset = 12 + localFootIndex_ * QUADRUPED_CONTACT_DIM;
  
  for (size_t i = 0; i < numVertices_; ++i) {
    approx.dfdx(i, footPosOffset) = param.a(i, 0);
    approx.dfdx(i, footPosOffset + 1) = param.a(i, 1);
    approx.dfdx(i, footPosOffset + 2) = param.a(i, 2);
  }

  return approx;
}

std::pair<matrix_t, vector_t> SingleRobotPerceptiveFootPlacementConstraint::getPolygonConstraint(
    const convex_plane_decomposition::CgalPolygon2d& polygon) const {
  size_t numVerts = polygon.size();
  if (numVerts == 0) {
    return {matrix_t::Zero(numVertices_, 2), vector_t::Constant(numVertices_, 1.0)};
  }

  matrix_t polytopeA = matrix_t::Zero(numVertices_, 2);
  vector_t polytopeB = vector_t::Constant(numVertices_, 1.0);

  for (size_t i = 0; i < std::min(numVerts, numVertices_); i++) {
    size_t j = (i + 1) % numVerts;
    size_t k = (j + 1) % numVerts;
    
    const auto point_a = polygon.vertex(i);
    const auto point_b = polygon.vertex(j);
    const auto point_c = polygon.vertex(k);

    polytopeA.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
    polytopeB(i) = point_a.y() * point_b.x() - point_a.x() * point_b.y();
    
    if (polytopeA.row(i) * (vector_t(2) << point_c.x(), point_c.y()).finished() + polytopeB(i) < 0) {
      polytopeA.row(i) *= -1;
      polytopeB(i) *= -1;
    }
  }

  return {polytopeA, polytopeB};
}

}  // namespace multi_robot
}  // namespace ocs2
