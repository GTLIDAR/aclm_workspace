#include "ocs2_multi_robot/constraint/RobotCargoFormationConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RobotCargoFormationConstraint::RobotCargoFormationConstraint(
    const SwitchedModelReferenceManagerWithTerrain& referenceManager,
    const vector_t& robot1_offset,
    const vector_t& robot2_offset,
    const vector_t& position_tolerance)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      robot1_offset_(robot1_offset),
      robot2_offset_(robot2_offset),
      position_tolerance_(position_tolerance) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool RobotCargoFormationConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t RobotCargoFormationConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                  const PreComputation& preComp) const {
  // Extract cargo state
  const vector3_t cargo_world = state.segment(48, 3);
  const vector3_t cargo_euler = state.segment(54, 3);
  const auto w_R_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);
  
  // Extract robot 1 state
  const vector3_t robot1_world = state.segment(0, 3);
  
  // Extract robot 2 state
  const vector3_t robot2_world = state.segment(24, 3);

  // Get nominal offsets in cargo frame (just the position part)
  const vector3_t robot1_offset_pos = robot1_offset_.segment(0, 3);
  const vector3_t robot2_offset_pos = robot2_offset_.segment(0, 3);

  // Compute actual positions in cargo frame
  const vector3_t robot1_in_cargo_frame = w_R_cargo.transpose() * (robot1_world - cargo_world);
  const vector3_t robot2_in_cargo_frame = w_R_cargo.transpose() * (robot2_world - cargo_world);

  // Compute constraint violations (8 total: 4 per robot, only x and y)
  vector_t constraint_violation(8);
  
  // Robot 1 constraints (indices 0-3)
  // Lower bounds (constraints 0-1): robot1_in_cargo(x,y) >= nominal - tolerance
  constraint_violation.segment(0, 2) = robot1_in_cargo_frame.head(2) - (robot1_offset_pos.head(2) - position_tolerance_.head(2));
  
  // Upper bounds (constraints 2-3): robot1_in_cargo(x,y) <= nominal + tolerance
  constraint_violation.segment(2, 2) = -robot1_in_cargo_frame.head(2) + (robot1_offset_pos.head(2) + position_tolerance_.head(2));

  // Robot 2 constraints (indices 4-7)
  // Lower bounds (constraints 4-5): robot2_in_cargo(x,y) >= nominal - tolerance
  constraint_violation.segment(4, 2) = robot2_in_cargo_frame.head(2) - (robot2_offset_pos.head(2) - position_tolerance_.head(2));
  
  // Upper bounds (constraints 6-7): robot2_in_cargo(x,y) <= nominal + tolerance
  constraint_violation.segment(6, 2) = -robot2_in_cargo_frame.head(2) + (robot2_offset_pos.head(2) + position_tolerance_.head(2));

  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation RobotCargoFormationConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  
  // Extract cargo state
  const vector3_t cargo_world = state.segment(48, 3);
  const vector3_t cargo_euler = state.segment(54, 3);
  const auto w_R_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);
  
  // Extract robot states
  const vector3_t robot1_world = state.segment(0, 3);
  const vector3_t robot2_world = state.segment(24, 3);

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(8, state.size());
  approx.dfdu = matrix_t::Zero(8, input.size());

  // Robot 1 constraints (indices 0-3, only x and y)
  // df/d(robot1_world) for constraints 0-1 (lower bounds, x and y)
  approx.dfdx.block<2, 3>(0, 0) = w_R_cargo.transpose().topRows(2);
  
  // df/d(robot1_world) for constraints 2-3 (upper bounds, x and y)
  approx.dfdx.block<2, 3>(2, 0) = -w_R_cargo.transpose().topRows(2);
  
  // df/d(cargo_world) for constraints 0-1
  approx.dfdx.block<2, 3>(0, 48) = -w_R_cargo.transpose().topRows(2);
  
  // df/d(cargo_world) for constraints 2-3
  approx.dfdx.block<2, 3>(2, 48) = w_R_cargo.transpose().topRows(2);

  // df/d(cargo_euler) for constraints 0-1 and 2-3 (only x and y rows)
  const vector3_t robot1_rel = robot1_world - cargo_world;
  const matrix_t jac1 = getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, robot1_rel);
  approx.dfdx.block<2, 3>(0, 54) = jac1.topRows(2);
  approx.dfdx.block<2, 3>(2, 54) = -jac1.topRows(2);

  // Robot 2 constraints (indices 4-7, only x and y)
  // df/d(robot2_world) for constraints 4-5 (lower bounds, x and y)
  approx.dfdx.block<2, 3>(4, 24) = w_R_cargo.transpose().topRows(2);
  
  // df/d(robot2_world) for constraints 6-7 (upper bounds, x and y)
  approx.dfdx.block<2, 3>(6, 24) = -w_R_cargo.transpose().topRows(2);
  
  // df/d(cargo_world) for constraints 4-5
  approx.dfdx.block<2, 3>(4, 48) = -w_R_cargo.transpose().topRows(2);
  
  // df/d(cargo_world) for constraints 6-7
  approx.dfdx.block<2, 3>(6, 48) = w_R_cargo.transpose().topRows(2);

  // df/d(cargo_euler) for constraints 4-5 and 6-7 (only x and y rows)
  const vector3_t robot2_rel = robot2_world - cargo_world;
  const matrix_t jac2 = getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, robot2_rel);
  approx.dfdx.block<2, 3>(4, 54) = jac2.topRows(2);
  approx.dfdx.block<2, 3>(6, 54) = -jac2.topRows(2);

  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
