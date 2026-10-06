#include "ocs2_multi_robot/constraint/distributed/CargoArmKinematicsBoxConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CargoArmKinematicsBoxConstraint::CargoArmKinematicsBoxConstraint(
    const SwitchedModelReferenceManagerWithTerrain& referenceManager,
    const std::vector<vector_t>& handlePositions,
    size_t numRobots)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      handlePositions_(handlePositions),
      numRobots_(numRobots) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool CargoArmKinematicsBoxConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CargoArmKinematicsBoxConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                    const PreComputation& preComp) const {
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
  
  // Extract cargo state (variable)
  const vector3_t cargo_world = state.segment(0, 3);
  const vector3_t cargo_euler = state.segment(6, 3);
  const auto w_R_b_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);
  
  vector_t constraint_violation(numRobots_ * 6);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    // Get robot state from precomputation (fixed)
    const vector_t& prevRobotState = preCompAlternating.getPrevRobotState(robot);
    const vector3_t robot_world = prevRobotState.segment(0, 3);
    const vector3_t robot_euler = prevRobotState.segment(6, 3);
    const auto w_R_b_robot = getRotationMatrixFromZyxEulerAngles(robot_euler);
    
    // Compute handle position in world frame from cargo state
    const auto handle_world = cargo_world + w_R_b_cargo * handlePositions_[robot].segment(0, 3);
    const size_t constraintOffset = robot * 6;
    
    // Position constraints (6): lower and upper bounds on handle position relative to robot base
    // Lower bound: R_robot^T * (handle - robot) - lower_bound >= 0
    constraint_violation.segment(constraintOffset, 3) = w_R_b_robot.transpose() * (handle_world - robot_world);
    constraint_violation.segment(constraintOffset, 3) += -arm_max_dev_from_base.segment(0, 3);
    // Upper bound: upper_bound - R_robot^T * (handle - robot) >= 0
    constraint_violation.segment(constraintOffset + 3, 3) = -w_R_b_robot.transpose() * (handle_world - robot_world);
    constraint_violation.segment(constraintOffset + 3, 3) += arm_max_dev_from_base.segment(3, 3);
  }
  
  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation CargoArmKinematicsBoxConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
  
  // Extract cargo state (variable)
  const vector3_t cargo_world = state.segment(0, 3);
  const vector3_t cargo_euler = state.segment(6, 3);
  const auto w_R_b_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);
  
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(numRobots_ * 6, state.size());
  approx.dfdu = matrix_t::Zero(numRobots_ * 6, input.size());
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    // Get robot state from precomputation (fixed)
    const vector_t& prevRobotState = preCompAlternating.getPrevRobotState(robot);
    const vector3_t robot_world = prevRobotState.segment(0, 3);
    const vector3_t robot_euler = prevRobotState.segment(6, 3);
    const auto w_R_b_robot = getRotationMatrixFromZyxEulerAngles(robot_euler);
    
    // Compute handle position in world frame from cargo state
    const auto handle_world = cargo_world + w_R_b_cargo * handlePositions_[robot].segment(0, 3);
    const size_t constraintOffset = robot * 6;
    
    // df/d(cargo_world) for position constraints
    approx.dfdx.block<3, 3>(constraintOffset, 0) = w_R_b_robot.transpose();
    approx.dfdx.block<3, 3>(constraintOffset + 3, 0) = -w_R_b_robot.transpose();
    
    // df/d(cargo_euler) for position constraints
    const vector3_t handle_pos = handlePositions_[robot].segment(0, 3);
    approx.dfdx.block<3, 3>(constraintOffset, 6) = 
        w_R_b_robot.transpose() * getRotMatrixJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, handle_pos);
    approx.dfdx.block<3, 3>(constraintOffset + 3, 6) = 
        -w_R_b_robot.transpose() * getRotMatrixJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, handle_pos);
    
    // Note: df/d(robot_world) and df/d(robot_euler) are zero because robots are fixed in precomputation
  }
  
  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
