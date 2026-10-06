#include "ocs2_multi_robot/constraint/MultiRobotCargoFormationConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiRobotCargoFormationConstraint::MultiRobotCargoFormationConstraint(
    const SwitchedModelReferenceManagerWithTerrain& referenceManager,
    const std::vector<vector_t>& robotOffsets,
    const vector_t& position_tolerance,
    size_t numRobots)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      robotOffsets_(robotOffsets),
      position_tolerance_(position_tolerance),
      numRobots_(numRobots) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MultiRobotCargoFormationConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MultiRobotCargoFormationConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const PreComputation& preComp) const {
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  
  // Extract cargo state
  const vector3_t cargo_world = state.segment(cargoStateOffset, 3);
  const vector3_t cargo_euler = state.segment(cargoStateOffset + 6, 3);
  const auto w_R_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);

  // Compute constraint violations (4 per robot: 2 lower + 2 upper bounds for x and y)
  vector_t constraint_violation(numRobots_ * 4);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    const vector3_t robot_world = state.segment(robotStateOffset, 3);
    
    // Get nominal offset in cargo frame (just the position part)
    const vector3_t robot_offset_pos = robotOffsets_[robot].segment(0, 3);
    
    // Compute actual position in cargo frame
    const vector3_t robot_in_cargo_frame = w_R_cargo.transpose() * (robot_world - cargo_world);
    
    const size_t constraintOffset = robot * 4;
    
    // Lower bounds (constraints 0-1): robot_in_cargo(x,y) >= nominal - tolerance
    constraint_violation.segment(constraintOffset, 2) = robot_in_cargo_frame.head(2) - (robot_offset_pos.head(2) - position_tolerance_.head(2));
    
    // Upper bounds (constraints 2-3): robot_in_cargo(x,y) <= nominal + tolerance
    // Formulated as: nominal + tolerance - robot_in_cargo(x,y) >= 0
    constraint_violation.segment(constraintOffset + 2, 2) = -robot_in_cargo_frame.head(2) + (robot_offset_pos.head(2) + position_tolerance_.head(2));
  }

  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation MultiRobotCargoFormationConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  
  // Extract cargo state
  const vector3_t cargo_world = state.segment(cargoStateOffset, 3);
  const vector3_t cargo_euler = state.segment(cargoStateOffset + 6, 3);
  const auto w_R_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(numRobots_ * 4, state.size());
  approx.dfdu = matrix_t::Zero(numRobots_ * 4, input.size());

  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    const vector3_t robot_world = state.segment(robotStateOffset, 3);
    
    const size_t constraintOffset = robot * 4;
    
    // df/d(robot_world) for constraints (lower bounds, x and y)
    approx.dfdx.block<2, 3>(constraintOffset, robotStateOffset) = w_R_cargo.transpose().topRows(2);
    
    // df/d(robot_world) for constraints (upper bounds, x and y)
    approx.dfdx.block<2, 3>(constraintOffset + 2, robotStateOffset) = -w_R_cargo.transpose().topRows(2);
    
    // df/d(cargo_world) for constraints (lower bounds)
    approx.dfdx.block<2, 3>(constraintOffset, cargoStateOffset) = -w_R_cargo.transpose().topRows(2);
    
    // df/d(cargo_world) for constraints (upper bounds)
    approx.dfdx.block<2, 3>(constraintOffset + 2, cargoStateOffset) = w_R_cargo.transpose().topRows(2);

    // df/d(cargo_euler) for constraints (only x and y rows)
    const vector3_t robot_rel = robot_world - cargo_world;
    const matrix_t jac = getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, robot_rel);
    approx.dfdx.block<2, 3>(constraintOffset, cargoStateOffset + 6) = jac.topRows(2);
    approx.dfdx.block<2, 3>(constraintOffset + 2, cargoStateOffset + 6) = -jac.topRows(2);
  }

  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
