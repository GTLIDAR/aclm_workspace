#include "ocs2_multi_robot/constraint/MultiRobotArmKinematicsBoxConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiRobotArmKinematicsBoxConstraint::MultiRobotArmKinematicsBoxConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
  const std::vector<vector_t>& robotHandles, size_t numRobots)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      robotHandles_(robotHandles),
      numRobots_(numRobots) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MultiRobotArmKinematicsBoxConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MultiRobotArmKinematicsBoxConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  const vector3_t cargo_world = state.segment(cargoStateOffset, 3);
  const vector3_t cargo_euler = state.segment(cargoStateOffset + 6, 3);
  const auto w_R_b_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);
  
  vector_t constraint_violation(numRobots_ * 6);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    const vector3_t robot_world = state.segment(robotStateOffset, 3);
    const vector3_t robot_euler = state.segment(robotStateOffset + 6, 3);
    const auto w_R_b_robot = getRotationMatrixFromZyxEulerAngles(robot_euler);
    
    const auto handle_world = cargo_world + w_R_b_cargo * robotHandles_[robot].segment(0, 3);
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
VectorFunctionLinearApproximation MultiRobotArmKinematicsBoxConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  const vector3_t cargo_world = state.segment(cargoStateOffset, 3);
  const vector3_t cargo_euler = state.segment(cargoStateOffset + 6, 3);
  const auto w_R_b_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);
  
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(numRobots_ * 6, state.size());
  approx.dfdu = matrix_t::Zero(numRobots_ * 6, input.size());
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    const vector3_t robot_world = state.segment(robotStateOffset, 3);
    const vector3_t robot_euler = state.segment(robotStateOffset + 6, 3);
    const auto w_R_b_robot = getRotationMatrixFromZyxEulerAngles(robot_euler);
    
    const auto handle_world = cargo_world + w_R_b_cargo * robotHandles_[robot].segment(0, 3);
    const size_t constraintOffset = robot * 6;
    
    // df/d(robot_world) for position constraints
    approx.dfdx.block<3, 3>(constraintOffset, robotStateOffset) = -w_R_b_robot.transpose();
    approx.dfdx.block<3, 3>(constraintOffset + 3, robotStateOffset) = w_R_b_robot.transpose();
    
    // df/d(robot_euler) for position constraints
    const vector3_t handle_rel = handle_world - robot_world;
    approx.dfdx.block<3, 3>(constraintOffset, robotStateOffset + 6) = 
        getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(robot_euler, handle_rel);
    approx.dfdx.block<3, 3>(constraintOffset + 3, robotStateOffset + 6) = 
        -getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(robot_euler, handle_rel);
    
    // df/d(cargo_world) for position constraints
    approx.dfdx.block<3, 3>(constraintOffset, cargoStateOffset) = w_R_b_robot.transpose();
    approx.dfdx.block<3, 3>(constraintOffset + 3, cargoStateOffset) = -w_R_b_robot.transpose();
    
    // df/d(cargo_euler) for position constraints
    const vector3_t handle_pos = robotHandles_[robot].segment(0, 3);
    approx.dfdx.block<3, 3>(constraintOffset, cargoStateOffset + 6) = 
        w_R_b_robot.transpose() * getRotMatrixJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, handle_pos);
    approx.dfdx.block<3, 3>(constraintOffset + 3, cargoStateOffset + 6) = 
        -w_R_b_robot.transpose() * getRotMatrixJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, handle_pos);
  }
  
  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2

