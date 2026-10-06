#include "ocs2_multi_robot/constraint/distributed/CargoRobotFormationConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CargoRobotFormationConstraint::CargoRobotFormationConstraint(
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
bool CargoRobotFormationConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CargoRobotFormationConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                  const PreComputation& preComp) const {
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
  
  // Extract cargo state (variable)
  const vector3_t cargo_world = state.segment(0, 3);
  const vector3_t cargo_euler = state.segment(6, 3);
  const auto w_R_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);

  // Compute constraint violations (4 per robot: 2 lower + 2 upper bounds for x and y)
  vector_t constraint_violation(numRobots_ * 4);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    // Get robot state from precomputation (fixed)
    const vector_t& prevRobotState = preCompAlternating.getPrevRobotState(robot);
    const vector3_t robot_world = prevRobotState.segment(0, 3);
    
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
VectorFunctionLinearApproximation CargoRobotFormationConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
  
  // Extract cargo state (variable)
  const vector3_t cargo_world = state.segment(0, 3);
  const vector3_t cargo_euler = state.segment(6, 3);
  const auto w_R_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(numRobots_ * 4, state.size());
  approx.dfdu = matrix_t::Zero(numRobots_ * 4, input.size());

  for (size_t robot = 0; robot < numRobots_; ++robot) {
    // Get robot state from precomputation (fixed)
    const vector_t& prevRobotState = preCompAlternating.getPrevRobotState(robot);
    const vector3_t robot_world = prevRobotState.segment(0, 3);
    
    const size_t constraintOffset = robot * 4;
    
    // df/d(cargo_world) for constraints (lower bounds)
    approx.dfdx.block<2, 3>(constraintOffset, 0) = -w_R_cargo.transpose().topRows(2);
    
    // df/d(cargo_world) for constraints (upper bounds)
    approx.dfdx.block<2, 3>(constraintOffset + 2, 0) = w_R_cargo.transpose().topRows(2);

    // df/d(cargo_euler) for constraints (only x and y rows)
    const vector3_t robot_rel = robot_world - cargo_world;
    const matrix_t jac = getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, robot_rel);
    approx.dfdx.block<2, 3>(constraintOffset, 6) = jac.topRows(2);
    approx.dfdx.block<2, 3>(constraintOffset + 2, 6) = -jac.topRows(2);
    
    // Note: df/d(robot_world) is zero because robots are fixed in precomputation
  }

  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
