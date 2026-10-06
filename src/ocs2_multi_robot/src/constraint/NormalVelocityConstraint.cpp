#include "ocs2_multi_robot/constraint/NormalVelocityConstraint.h"
#include <ocs2_quadruped/LeggedRobotPreComputation.h>
#include <ros/ros.h>

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraint::NormalVelocityConstraint(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                                   std::shared_ptr<quadruped::SwingTrajectoryPlanner> swingTrajectoryPlannerPtr,
                                                   size_t contactPointIndex, size_t index,
                                                   size_t robotStateOffset, scalar_t positionErrorGain)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      swingTrajectoryPlannerPtr_(std::move(swingTrajectoryPlannerPtr)),
      contactPointIndex_(contactPointIndex),
      index_(index),
      robotStateOffset_(robotStateOffset),
      positionErrorGain_(positionErrorGain) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraint::NormalVelocityConstraint(const NormalVelocityConstraint& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      contactPointIndex_(rhs.contactPointIndex_),
      index_(rhs.index_),
      robotStateOffset_(rhs.robotStateOffset_),
      positionErrorGain_(rhs.positionErrorGain_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool NormalVelocityConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t NormalVelocityConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                 const PreComputation& preComp) const {
  // Input layout follows the same convention as ZeroVelocityConstraint:
  //   v_foot = input.segment(QUADRUPED_CONTACT_DIM * QUADRUPED_FOOT_NUM + index_ * QUADRUPED_CONTACT_DIM, QUADRUPED_CONTACT_DIM)
  // Here we only need the normal (z) component of that foot velocity.
  const size_t zIndex =
      QUADRUPED_CONTACT_DIM * QUADRUPED_FOOT_NUM + index_ * QUADRUPED_CONTACT_DIM + 2;  // +2 -> z component
  if (zIndex + 1 > input.size()) {
    throw std::runtime_error("[NormalVelocityConstraint] Invalid input indexing: zIndex out of bounds.");
  }

  const auto vel_z = input(zIndex);

  // SwingTrajectoryPlanner is single-robot and only has QUADRUPED_FOOT_NUM legs.
  // Use the local foot index (0..QUADRUPED_FOOT_NUM-1), not the global contact index.
  const size_t localFootIndex = contactPointIndex_ % QUADRUPED_FOOT_NUM;

  // Get target z-velocity from swing trajectory
  const scalar_t targetVelZ = swingTrajectoryPlannerPtr_->getZvelocityConstraint(localFootIndex, time);
  
  // Constraint: v_z + positionErrorGain * (z - z_target) = v_target
  // => v_z + positionErrorGain * z - positionErrorGain * z_target - v_target = 0
  // => v_z + positionErrorGain * z = v_target + positionErrorGain * z_target
  vector_t constraint(1);
  constraint(0) = vel_z - targetVelZ;
  
  if (positionErrorGain_ > 0.0) {
    // Get foot z position from state
    const size_t footZIndex = robotStateOffset_ + 12 + (localFootIndex) * 3 + 2;  // +2 for z component
    const scalar_t footZ = state(footZIndex);
    const scalar_t targetPosZ = swingTrajectoryPlannerPtr_->getZpositionConstraint(localFootIndex, time);
    
    // Add position error term: positionErrorGain * (z - z_target)
    constraint(0) += positionErrorGain_ * (footZ - targetPosZ);
  }
  
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation NormalVelocityConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(1, state.size());
  approx.dfdu = matrix_t::Zero(1, input.size());

  const size_t zIndex =
      QUADRUPED_CONTACT_DIM * QUADRUPED_FOOT_NUM + index_ * QUADRUPED_CONTACT_DIM + 2;  // +2 -> z component
  if (zIndex + 1 > input.size()) {
    throw std::runtime_error("[NormalVelocityConstraint::getLinearApproximation] Invalid input indexing: zIndex out of bounds.");
  }

  // df/d(vel_z) = 1
  approx.dfdu(0, zIndex) = 1.0;
  
  // If position error gain is active, add df/d(foot_z) = positionErrorGain
  if (positionErrorGain_ > 0.0) {
    const size_t localFootIndex = contactPointIndex_ % QUADRUPED_FOOT_NUM;
    const size_t footZIndex = robotStateOffset_ + 12 + (localFootIndex) * 3 + 2;  // +2 for z component
    approx.dfdx(0, footZIndex) = positionErrorGain_;
  }
  
  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
