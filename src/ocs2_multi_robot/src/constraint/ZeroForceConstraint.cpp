#include "ocs2_multi_robot/constraint/ZeroForceConstraint.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                         size_t contactPointIndex, size_t index)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      Index_(index),
      contactPointIndex_(contactPointIndex){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroForceConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  return input.segment(Index_*QUADRUPED_CONTACT_DIM, QUADRUPED_CONTACT_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(QUADRUPED_CONTACT_DIM, state.size());
  approx.dfdu = matrix_t::Zero(QUADRUPED_CONTACT_DIM, input.size());
  approx.dfdu.middleCols<QUADRUPED_CONTACT_DIM>(Index_*QUADRUPED_CONTACT_DIM).diagonal() = vector_t::Ones(QUADRUPED_CONTACT_DIM);
  return approx;
}

}  // namespace multi_robot
}  // namespace ocs2
