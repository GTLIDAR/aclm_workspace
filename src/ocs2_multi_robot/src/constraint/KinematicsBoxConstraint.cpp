#include "ocs2_multi_robot/constraint/KinematicsBoxConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
KinematicsBoxConstraint::KinematicsBoxConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                         size_t contactPointIndex, size_t index, size_t stateOffset)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      Index_(index),
      robotIDstateOffset_(stateOffset),
      contactPointIndex_(contactPointIndex){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool KinematicsBoxConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t KinematicsBoxConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  // Calculate local foot index within the robot's state block
  // contactPointIndex_ is the global foot index, we need the local foot index
  const size_t footInRobot = contactPointIndex_ % QUADRUPED_FOOT_NUM;
  const size_t footStateOffset = robotIDstateOffset_ + 12 + footInRobot * QUADRUPED_CONTACT_DIM;
  const vector3_t pos_world = state.segment(footStateOffset, QUADRUPED_CONTACT_DIM);
  const vector3_t com_world = state.segment(0 + robotIDstateOffset_, 3);
  const vector3_t orient_euler = state.segment(6 + robotIDstateOffset_, 3);
  const auto w_R_b = getRotationMatrixFromZyxEulerAngles(orient_euler);

  vector_t constraint_violation(6);
  const vector3_t nominal_pos = getNominalEePosWrtCom(footInRobot);
  constraint_violation.segment(0, 3) = w_R_b.transpose()*(pos_world - com_world);
  constraint_violation.segment(0, 3) += -(nominal_pos - max_dev_from_nominal);
  constraint_violation.segment(3, 3) = -w_R_b.transpose()*(pos_world - com_world);
  constraint_violation.segment(3, 3) += (nominal_pos + max_dev_from_nominal);
  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation KinematicsBoxConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  const size_t footInRobot = contactPointIndex_ % QUADRUPED_FOOT_NUM;
  const size_t footStateOffset = robotIDstateOffset_ + 12 + footInRobot * QUADRUPED_CONTACT_DIM;
  const vector3_t pos_world = state.segment(footStateOffset, QUADRUPED_CONTACT_DIM);
  const vector3_t com_world = state.segment(0 + robotIDstateOffset_, 3);
  const vector3_t orient_euler = state.segment(6 + robotIDstateOffset_, 3);
  const auto w_R_b = getRotationMatrixFromZyxEulerAngles(orient_euler);
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(6, state.size());
  approx.dfdx.block<3, 3>(0, 0 + robotIDstateOffset_) = -w_R_b.transpose()*matrix3_t::Identity();
  approx.dfdx.block<3, 3>(3, 0 + robotIDstateOffset_) = w_R_b.transpose()*matrix3_t::Identity();
  approx.dfdx.block<3, 3>(0, footStateOffset) = w_R_b.transpose()*matrix3_t::Identity();
  approx.dfdx.block<3, 3>(3, footStateOffset) = -w_R_b.transpose()*matrix3_t::Identity();

  // TODO: get the partial derivative to the orientation
  approx.dfdx.block<3, 3>(0, 6 + robotIDstateOffset_) = getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(orient_euler, pos_world - com_world)*matrix3_t::Identity();
  approx.dfdx.block<3, 3>(3, 6 + robotIDstateOffset_) = -getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec<scalar_t>(orient_euler, pos_world - com_world)*matrix3_t::Identity();

  approx.dfdu = matrix_t::Zero(6, input.size());
  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
