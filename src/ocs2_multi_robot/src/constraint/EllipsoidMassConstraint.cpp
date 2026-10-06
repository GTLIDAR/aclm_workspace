#include "ocs2_multi_robot/constraint/EllipsoidMassConstraint.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "ocs2_multi_robot/utils/OrientationTools.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EllipsoidMassConstraint::EllipsoidMassConstraint(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                                 const scalar_t& mass)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      mass_(mass) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EllipsoidMassConstraint::EllipsoidMassConstraint(const EllipsoidMassConstraint& rhs)
    : StateConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      mass_(rhs.mass_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool EllipsoidMassConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EllipsoidMassConstraint::getValue(scalar_t time, const vector_t& state,
                                                 const PreComputation& preComp) const {
  const vector_t e = state.segment(STATE_DIM_AE, 3);

  const TargetTrajectories& targetTrajectories =
      referenceManagerPtr_->getTargetTrajectories();
  const auto stateReference = targetTrajectories.getDesiredState(time);
  if (stateReference.size() != STATE_DIM_AE_WITH_ELLIPSOID + 6) {
    throw std::runtime_error("state size is wrong, need to be 30");
  }
  const vector_t I_w_vec = stateReference.tail(6);
  matrix3_t I_w;
  I_w << I_w_vec(0), I_w_vec(1), I_w_vec(3),
         I_w_vec(1), I_w_vec(2), I_w_vec(4),
         I_w_vec(3), I_w_vec(4), I_w_vec(5); 
  vector3_t I_wbd_ev = I_w.eigenvalues().real();
  scalar_t rho = (pow(mass_, 5.0/2.0)*pow(18.0/(125.0*(I_wbd_ev(0) + I_wbd_ev(1) - I_wbd_ev(2))*
                  (I_wbd_ev(0) - I_wbd_ev(1) + I_wbd_ev(2))*(I_wbd_ev(1) - I_wbd_ev(0) + I_wbd_ev(2))), 1.0/2.0))/(2.0*M_PI);
  // scalar_t rho = (pow(3.0/4.0*mass_, 5.0/2.0) * pow(8.0/15.0*M_PI, 3.0/2.0)) 
  //                / pow(M_PI, 5.0/2.0)*pow(I_wbd_ev(0)*I_wbd_ev(1)*I_wbd_ev(2), 0.5);
  // std::cout << "rho is: " << rho << std::endl;

  vector_t constraint_violation(1);
  constraint_violation << 4.0/3.0*M_PI*e(0)*e(1)*e(2)*rho - mass_;
  return constraint_violation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EllipsoidMassConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                  const PreComputation& preComp) const {
  const vector_t e = state.segment(STATE_DIM_AE, 3);

  const TargetTrajectories& targetTrajectories =
      referenceManagerPtr_->getTargetTrajectories();
  const auto stateReference = targetTrajectories.getDesiredState(time);
  if (stateReference.size() != STATE_DIM_AE_WITH_ELLIPSOID + 6) {
    throw std::runtime_error("state size is wrong, need to be 30");
  }
  const vector_t I_w_vec = stateReference.tail(6);
  matrix3_t I_w;
  I_w << I_w_vec(0), I_w_vec(1), I_w_vec(3),
         I_w_vec(1), I_w_vec(2), I_w_vec(4),
         I_w_vec(3), I_w_vec(4), I_w_vec(5); 
  vector3_t I_wbd_ev = I_w.eigenvalues().real();
  scalar_t rho = (pow(mass_, 5.0/2.0)*pow(18.0/(125.0*(I_wbd_ev(0) + I_wbd_ev(1) - I_wbd_ev(2))*
                  (I_wbd_ev(0) - I_wbd_ev(1) + I_wbd_ev(2))*(I_wbd_ev(1) - I_wbd_ev(0) + I_wbd_ev(2))), 1.0/2.0))/(2.0*M_PI);
  // scalar_t rho = (pow(3.0/4.0*mass_, 5.0/2.0) * pow(8.0/15.0*M_PI, 3.0/2.0)) 
  //                / pow(M_PI, 5.0/2.0)*pow(I_wbd_ev(0)*I_wbd_ev(1)*I_wbd_ev(2), 0.5);

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, preComp);
  approx.dfdx = matrix_t::Zero(1, state.size());
  approx.dfdx(0, STATE_DIM_AE) = 4.0/3.0*M_PI*e(1)*e(2)*rho;
  approx.dfdx(0, STATE_DIM_AE + 1) = 4.0/3.0*M_PI*e(0)*e(2)*rho;
  approx.dfdx(0, STATE_DIM_AE + 2) = 4.0/3.0*M_PI*e(0)*e(1)*rho;
  
  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
