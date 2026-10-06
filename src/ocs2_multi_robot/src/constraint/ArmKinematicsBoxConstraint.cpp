#include "ocs2_multi_robot/constraint/ArmKinematicsBoxConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ArmKinematicsBoxConstraint::ArmKinematicsBoxConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
  const vector_t& r1_handle, const vector_t& r2_handle)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      r1_handle_(r1_handle),
      r2_handle_(r2_handle) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ArmKinematicsBoxConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ArmKinematicsBoxConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  const vector3_t cargo_world = state.segment(48, 3);
  const vector3_t cargo_euler = state.segment(54, 3);
  const auto w_R_b_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);
  
  const vector3_t robot1_world = state.segment(0, 3);
  const vector3_t robot1_euler = state.segment(6, 3);
  const auto w_R_b_r1 = getRotationMatrixFromZyxEulerAngles(robot1_euler);

  const vector3_t robot2_world = state.segment(24, 3);
  const vector3_t robot2_euler = state.segment(30, 3);
  const auto w_R_b_r2 = getRotationMatrixFromZyxEulerAngles(robot2_euler);

  const auto r1_world = cargo_world + w_R_b_cargo * r1_handle_.segment(0,3);
  const auto r2_world = cargo_world + w_R_b_cargo * r2_handle_.segment(0,3);

  vector_t constraint_violation(12);
  constraint_violation.segment(0, 3) = w_R_b_r1.transpose() * (r1_world - robot1_world);
  constraint_violation.segment(0, 3) += -arm_max_dev_from_base.segment(0,3);
  constraint_violation.segment(3, 3) = -w_R_b_r1.transpose() * (r1_world - robot1_world);
  constraint_violation.segment(3, 3) += arm_max_dev_from_base.segment(3,3);

  constraint_violation.segment(6, 3) = w_R_b_r2.transpose() * (r2_world - robot2_world);
  constraint_violation.segment(6, 3) += -arm_max_dev_from_base.segment(0,3);
  constraint_violation.segment(9, 3) = -w_R_b_r2.transpose() * (r2_world - robot2_world);
  constraint_violation.segment(9, 3) += arm_max_dev_from_base.segment(3,3);
  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ArmKinematicsBoxConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  const vector3_t cargo_world = state.segment(48, 3);
  const vector3_t cargo_euler = state.segment(54, 3);
  const auto w_R_b_cargo = getRotationMatrixFromZyxEulerAngles(cargo_euler);
  
  const vector3_t robot1_world = state.segment(0, 3);
  const vector3_t robot1_euler = state.segment(6, 3);
  const auto w_R_b_r1 = getRotationMatrixFromZyxEulerAngles(robot1_euler);

  const vector3_t robot2_world = state.segment(24, 3);
  const vector3_t robot2_euler = state.segment(30, 3);
  const auto w_R_b_r2 = getRotationMatrixFromZyxEulerAngles(robot2_euler);

  const auto r1_world = cargo_world + w_R_b_cargo * r1_handle_.segment(0,3);
  const auto r2_world = cargo_world + w_R_b_cargo * r2_handle_.segment(0,3);

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(12, state.size());
  approx.dfdu = matrix_t::Zero(12, input.size());

  // Robot 1 constraints (indices 0-5)
  // df/d(robot1_world) for constraints 0-2 and 3-5
  approx.dfdx.block<3, 3>(0, 0) = -w_R_b_r1.transpose();
  approx.dfdx.block<3, 3>(3, 0) = w_R_b_r1.transpose();
  
  // df/d(robot1_euler) for constraints 0-2 and 3-5
  const vector3_t r1_rel = r1_world - robot1_world;
  approx.dfdx.block<3, 3>(0, 6) = getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(robot1_euler, r1_rel);
  approx.dfdx.block<3, 3>(3, 6) = -getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(robot1_euler, r1_rel);

  // df/d(cargo_world) for constraints 0-2 and 3-5
  approx.dfdx.block<3, 3>(0, 48) = w_R_b_r1.transpose();
  approx.dfdx.block<3, 3>(3, 48) = -w_R_b_r1.transpose();

  // df/d(cargo_euler) for constraints 0-2 and 3-5
  const vector3_t r1_handle_pos = r1_handle_.segment(0,3);
  const vector3_t r1_handle_world = w_R_b_cargo * r1_handle_pos;
  approx.dfdx.block<3, 3>(0, 54) = w_R_b_r1.transpose() * getRotMatrixJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, r1_handle_pos);
  approx.dfdx.block<3, 3>(3, 54) = -w_R_b_r1.transpose() * getRotMatrixJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, r1_handle_pos);

  // Robot 2 constraints (indices 6-11)
  // df/d(robot2_world) for constraints 6-8 and 9-11
  approx.dfdx.block<3, 3>(6, 24) = -w_R_b_r2.transpose();
  approx.dfdx.block<3, 3>(9, 24) = w_R_b_r2.transpose();
  
  // df/d(robot2_euler) for constraints 6-8 and 9-11
  const vector3_t r2_rel = r2_world - robot2_world;
  approx.dfdx.block<3, 3>(6, 30) = getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(robot2_euler, r2_rel);
  approx.dfdx.block<3, 3>(9, 30) = -getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(robot2_euler, r2_rel);

  // df/d(cargo_world) for constraints 6-8 and 9-11
  approx.dfdx.block<3, 3>(6, 48) = w_R_b_r2.transpose();
  approx.dfdx.block<3, 3>(9, 48) = -w_R_b_r2.transpose();

  // df/d(cargo_euler) for constraints 6-8 and 9-11
  const vector3_t r2_handle_pos = r2_handle_.segment(0,3);
  const vector3_t r2_handle_world = w_R_b_cargo * r2_handle_pos;
  approx.dfdx.block<3, 3>(6, 54) = w_R_b_r2.transpose() * getRotMatrixJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, r2_handle_pos);
  approx.dfdx.block<3, 3>(9, 54) = -w_R_b_r2.transpose() * getRotMatrixJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(cargo_euler, r2_handle_pos);

  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
