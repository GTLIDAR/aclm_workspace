#include "ocs2_multi_robot/constraint/FootPlacementConstraint.h"
#include <ocs2_quadruped/LeggedRobotPreComputation.h>
#include <ros/ros.h>

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FootPlacementConstraint::FootPlacementConstraint(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                                   size_t contactPointIndex, size_t index, size_t stateOffset)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      heightMapPtr_(std::move(referenceManagerPtr_->getHeightMap())),
      Index_(index),
      contactPointIndex_(contactPointIndex),
      robotStateOffset_(stateOffset) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      heightMapPtr_(std::move(referenceManagerPtr_->getHeightMap())),
      Index_(rhs.Index_),
      contactPointIndex_(rhs.contactPointIndex_),
      robotStateOffset_(rhs.robotStateOffset_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FootPlacementConstraint::isActive(scalar_t time) const {
  // Only active during stance. During swing, the NormalVelocityConstraint already
  // guides the foot along a terrain-aware spline. The height map has near-discontinuities
  // at stair edges that cause solver oscillation if this constraint is active mid-swing.
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                 const PreComputation& preComp) const {
  const size_t footInRobot = contactPointIndex_ % QUADRUPED_FOOT_NUM;
  const size_t footStateOffset = robotStateOffset_ + 12 + footInRobot * QUADRUPED_CONTACT_DIM;
  const auto pos_x = state[footStateOffset];
  const auto pos_y = state[footStateOffset + 1];
  const auto pos_z = state[footStateOffset + 2];
  
  // Get terrain height at foot (x,y) position
  const auto terrain_height = heightMapPtr_->GetHeight(pos_x, pos_y);
  
  // Stance only (isActive already gates swing out)
  const scalar_t upper_bound = 1e-2;
  const scalar_t lower_bound = -1e-2;
  
  // Constraint: foot z should be at terrain height during stance
  // This works correctly for slopes because terrain_height = h(x,y) gives the
  // correct z value at that (x,y) location on the terrain surface
  scalar_t height_error = pos_z - terrain_height;
  
  // Debug: show terrain info
  // Constraint: height_error should be within [lower_bound, upper_bound]
  // c1: height_error >= lower_bound  =>  height_error - lower_bound >= 0
  // c2: height_error <= upper_bound  =>  upper_bound - height_error >= 0
  vector_t constraint_violation(2);
  constraint_violation(0) = height_error - lower_bound;
  constraint_violation(1) = -height_error + upper_bound;
  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  
  const size_t footInRobot = contactPointIndex_ % QUADRUPED_FOOT_NUM;
  const size_t footStateOffset = robotStateOffset_ + 12 + footInRobot * QUADRUPED_CONTACT_DIM;
  const auto pos_x = state[footStateOffset];
  const auto pos_y = state[footStateOffset + 1];
  
  // Get terrain height derivatives
  // height_error = pos_z - h(pos_x, pos_y)
  // d(height_error)/d(pos_x) = -dh/dx
  // d(height_error)/d(pos_y) = -dh/dy
  // d(height_error)/d(pos_z) = 1
  const scalar_t dh_dx = heightMapPtr_->GetDerivativeOfHeightWrt(dim2X, pos_x, pos_y);
  const scalar_t dh_dy = heightMapPtr_->GetDerivativeOfHeightWrt(dim2Y, pos_x, pos_y);
  
  approx.dfdx = matrix_t::Zero(2, state.size());
  
  // First constraint: height_error - lower_bound >= 0
  approx.dfdx(0, footStateOffset + 0) = -dh_dx;
  approx.dfdx(0, footStateOffset + 1) = -dh_dy;
  approx.dfdx(0, footStateOffset + 2) = 1.0;
  
  // Second constraint: -height_error + upper_bound >= 0 (negated)
  approx.dfdx(1, footStateOffset + 0) = dh_dx;
  approx.dfdx(1, footStateOffset + 1) = dh_dy;
  approx.dfdx(1, footStateOffset + 2) = -1.0;
  
  approx.dfdu = matrix_t::Zero(2, input.size());
  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
