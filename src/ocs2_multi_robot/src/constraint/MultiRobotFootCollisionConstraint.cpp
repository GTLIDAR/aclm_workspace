//
// Multi-robot Foot Collision Constraint implementation using SDF
//

#include "ocs2_multi_robot/constraint/MultiRobotFootCollisionConstraint.h"

#include <ros/ros.h>

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiRobotFootCollisionConstraint::MultiRobotFootCollisionConstraint(
    PerceptiveMultiRobotReferenceManager& referenceManager,
    size_t globalFootIndex,
    size_t robotStateOffset,
    scalar_t clearance)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      globalFootIndex_(globalFootIndex),
      robotStateOffset_(robotStateOffset),
      localFootIndex_(globalFootIndex % QUADRUPED_FOOT_NUM),
      clearance_(clearance) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiRobotFootCollisionConstraint::MultiRobotFootCollisionConstraint(
    const MultiRobotFootCollisionConstraint& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      globalFootIndex_(rhs.globalFootIndex_),
      robotStateOffset_(rhs.robotStateOffset_),
      localFootIndex_(rhs.localFootIndex_),
      clearance_(rhs.clearance_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MultiRobotFootCollisionConstraint::isActive(scalar_t time) const {
  // Active during SWING phase only, with buffer around transitions
  // This avoids numerical issues when foot is very close to ground at touchdown/liftoff
  
  const auto& contactFlags = referenceManagerPtr_->getContactFlags(time);
  bool isStanceNow = contactFlags[globalFootIndex_];
  
  // Also check slightly before and after to avoid activating near phase boundaries
  bool isStanceBefore = referenceManagerPtr_->getContactFlags(time - phaseTransitionBuffer_)[globalFootIndex_];
  bool isStanceAfter = referenceManagerPtr_->getContactFlags(time + 0.5 * phaseTransitionBuffer_)[globalFootIndex_];
  
  // Only active if in swing phase AND not near a phase transition
  return !isStanceNow && !isStanceBefore && !isStanceAfter;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector3_t MultiRobotFootCollisionConstraint::getFootPositionFromState(const vector_t& state) const {
  // Foot positions start at offset 12 within each robot's state segment
  // State layout per robot: [com_pos(3), com_vel(3), euler(3), ang_mom(3), foot_pos(12)]
  const size_t footPosOffset = robotStateOffset_ + 12 + localFootIndex_ * QUADRUPED_CONTACT_DIM;
  return state.segment<3>(footPosOffset);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MultiRobotFootCollisionConstraint::getValue(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  
  vector_t g(1);
  vector3_t footPos = getFootPositionFromState(state);
  
  // Try to use SDF from reference manager
  auto sdfPtr = referenceManagerPtr_->getSDF();
  if (sdfPtr) {
    // Query SDF at foot position
    grid_map::Position3 footPos3D(footPos.x(), footPos.y(), footPos.z());
    scalar_t sdfValue = sdfPtr->value(footPos3D);
    
    // Constraint: SDF(foot) - clearance >= 0
    // Positive SDF means foot is in free space (away from obstacles)
    g(0) = sdfValue - clearance_;
    return g;
  }
  
  // Fallback: use height map if available
  auto heightMapPtr = referenceManagerPtr_->getHeightMap();
  if (heightMapPtr) {
    scalar_t terrainZ = heightMapPtr->GetHeight(footPos.x(), footPos.y());
    g(0) = footPos.z() - terrainZ - clearance_;
  } else {
    // No terrain info - constraint is always satisfied
    g(0) = 1.0;
  }
  
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation MultiRobotFootCollisionConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(1, state.size());
  approx.dfdu = matrix_t::Zero(1, input.size());
  
  const size_t footPosOffset = robotStateOffset_ + 12 + localFootIndex_ * QUADRUPED_CONTACT_DIM;
  vector3_t footPos = getFootPositionFromState(state);
  
  // Try to use SDF from reference manager
  auto sdfPtr = referenceManagerPtr_->getSDF();
  if (sdfPtr) {
    // Get SDF gradient (points in direction of increasing distance = away from obstacles)
    grid_map::Position3 footPos3D(footPos.x(), footPos.y(), footPos.z());
    Eigen::Vector3d sdfGradient = sdfPtr->derivative(footPos3D);
    
    // Jacobian: d(constraint)/d(foot_pos) = d(SDF)/d(foot_pos)
    approx.dfdx(0, footPosOffset)     = sdfGradient.x();
    approx.dfdx(0, footPosOffset + 1) = sdfGradient.y();
    approx.dfdx(0, footPosOffset + 2) = sdfGradient.z();
    return approx;
  }
  
  // Fallback: use height-map gradient
  auto heightMapPtr = referenceManagerPtr_->getHeightMap();
  if (heightMapPtr) {
    // Finite difference for terrain gradient
    const scalar_t delta = 0.01;
    scalar_t h_px = heightMapPtr->GetHeight(footPos.x() + delta, footPos.y());
    scalar_t h_nx = heightMapPtr->GetHeight(footPos.x() - delta, footPos.y());
    scalar_t h_py = heightMapPtr->GetHeight(footPos.x(), footPos.y() + delta);
    scalar_t h_ny = heightMapPtr->GetHeight(footPos.x(), footPos.y() - delta);
    
    scalar_t dz_dx = (h_px - h_nx) / (2.0 * delta);
    scalar_t dz_dy = (h_py - h_ny) / (2.0 * delta);
    
    approx.dfdx(0, footPosOffset)     = -dz_dx;
    approx.dfdx(0, footPosOffset + 1) = -dz_dy;
    approx.dfdx(0, footPosOffset + 2) = 1.0;
  }
  
  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
