#include "ocs2_multi_robot/constraint/distributed/SingleRobotArmKinematicsBoxConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SingleRobotArmKinematicsBoxConstraint::SingleRobotArmKinematicsBoxConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                                                             size_t index)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      index_(index) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool SingleRobotArmKinematicsBoxConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SingleRobotArmKinematicsBoxConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
  const auto& prevHandlePositionInWorld = preCompAlternating.getPrevHandlePositionInWorld(index_);
  
  const vector3_t robot_world = state.segment(0, 3);
  const vector3_t robot_euler = state.segment(6, 3);
  const auto w_R_b = getRotationMatrixFromZyxEulerAngles(robot_euler);

  vector_t constraint_violation(6);
  constraint_violation.segment(0, 3) = w_R_b.transpose() * (prevHandlePositionInWorld - robot_world);
  constraint_violation.segment(0, 3) += -arm_max_dev_from_base.segment(0,3);
  constraint_violation.segment(3, 3) = -w_R_b.transpose() * (prevHandlePositionInWorld - robot_world);
  constraint_violation.segment(3, 3) += arm_max_dev_from_base.segment(3,3);

  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SingleRobotArmKinematicsBoxConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComp);
  const auto& prevHandlePositionInWorld = preCompAlternating.getPrevHandlePositionInWorld(index_);
  
  const vector3_t robot_world = state.segment(0, 3);
  const vector3_t robot_euler = state.segment(6, 3);
  const auto w_R_b = getRotationMatrixFromZyxEulerAngles(robot_euler);

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(6, state.size());
  approx.dfdu = matrix_t::Zero(6, input.size());

  // Constraint: w_R_b^T * (prevHandlePositionInWorld - robot_world)
  // For constraints 0-2 (lower bounds): w_R_b^T * (prevHandlePositionInWorld - robot_world) - arm_max_dev_from_base(0:3) >= 0
  // For constraints 3-5 (upper bounds): -w_R_b^T * (prevHandlePositionInWorld - robot_world) + arm_max_dev_from_base(3:6) >= 0
  
  const vector3_t handle_rel = prevHandlePositionInWorld - robot_world;
  
  // df/d(robot_world) for constraints 0-2 (lower bounds)
  approx.dfdx.block<3, 3>(0, 0) = -w_R_b.transpose();
  
  // df/d(robot_world) for constraints 3-5 (upper bounds)
  approx.dfdx.block<3, 3>(3, 0) = w_R_b.transpose();
  
  // df/d(robot_euler) for constraints 0-2 (lower bounds)
  // d/d(euler)[w_R_b^T * handle_rel]
  approx.dfdx.block<3, 3>(0, 6) = getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(robot_euler, handle_rel);
  
  // df/d(robot_euler) for constraints 3-5 (upper bounds)
  // d/d(euler)[-w_R_b^T * handle_rel]
  approx.dfdx.block<3, 3>(3, 6) = -getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(robot_euler, handle_rel);

  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
