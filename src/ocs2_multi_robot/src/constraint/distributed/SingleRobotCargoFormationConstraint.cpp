#include "ocs2_multi_robot/constraint/distributed/SingleRobotCargoFormationConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SingleRobotCargoFormationConstraint::SingleRobotCargoFormationConstraint(
    const SwitchedModelReferenceManagerWithTerrain& referenceManager,
    const vector_t& robotOffset,
    const vector_t& position_tolerance,
    size_t robotIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      robotOffset_(robotOffset),
      position_tolerance_(position_tolerance),
      robotIndex_(robotIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool SingleRobotCargoFormationConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SingleRobotCargoFormationConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const PreComputation& preComp) const {
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
  const vector_t& prevCargoState = preCompAlternating.getPrevCargoState();
  
  // Extract cargo state from precomputation (fixed)
  const vector3_t cargo_world = prevCargoState.segment(0, 3);
  const vector3_t cargo_euler = prevCargoState.segment(6, 3);
  const auto w_R_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);

  // Extract robot state (variable)
  const vector3_t robot_world = state.segment(0, 3);
  
  // Get nominal offset in cargo frame (just the position part)
  const vector3_t robot_offset_pos = robotOffset_.segment(0, 3);
  
  // Compute actual position in cargo frame
  const vector3_t robot_in_cargo_frame = w_R_cargo.transpose() * (robot_world - cargo_world);
  
  vector_t constraint_violation(4);
  
  // Lower bounds (constraints 0-1): robot_in_cargo(x,y) >= nominal - tolerance
  constraint_violation.segment(0, 2) = robot_in_cargo_frame.head(2) - (robot_offset_pos.head(2) - position_tolerance_.head(2));
  
  // Upper bounds (constraints 2-3): robot_in_cargo(x,y) <= nominal + tolerance
  // Formulated as: nominal + tolerance - robot_in_cargo(x,y) >= 0
  constraint_violation.segment(2, 2) = -robot_in_cargo_frame.head(2) + (robot_offset_pos.head(2) + position_tolerance_.head(2));

  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SingleRobotCargoFormationConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
  const vector_t& prevCargoState = preCompAlternating.getPrevCargoState();
  
  // Extract cargo state from precomputation (fixed)
  const vector3_t cargo_world = prevCargoState.segment(0, 3);
  const vector3_t cargo_euler = prevCargoState.segment(6, 3);
  const auto w_R_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);

  // Extract robot state (variable)
  const vector3_t robot_world = state.segment(0, 3);

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(4, state.size());
  approx.dfdu = matrix_t::Zero(4, input.size());

  // df/d(robot_world) for constraints (lower bounds, x and y)
  approx.dfdx.block<2, 3>(0, 0) = w_R_cargo.transpose().topRows(2);
  
  // df/d(robot_world) for constraints (upper bounds, x and y)
  approx.dfdx.block<2, 3>(2, 0) = -w_R_cargo.transpose().topRows(2);
  
  // Note: df/d(cargo_world) and df/d(cargo_euler) are zero because cargo is fixed in precomputation

  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
