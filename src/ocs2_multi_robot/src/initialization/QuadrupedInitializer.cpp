#include "ocs2_multi_robot/initialization/QuadrupedInitializer.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadrupedInitializer::QuadrupedInitializer(const scalar_t& mass, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
    : mass_(mass), referenceManagerPtr_(&referenceManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadrupedInitializer* QuadrupedInitializer::clone() const {
  return new QuadrupedInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadrupedInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  size_t input_num;
  if (state.rows() == STATE_DIM_AE_WITH_ELLIPSOID) {
    input_num = INPUT_DIM_AE_WITH_ELLIPSOID;
  } else if (state.rows() == STATE_DIM_AE) {
    input_num = INPUT_DIM_AE;
  } else if (state.rows() == STATE_DIM_ELLIPSOID) {
    input_num = INPUT_DIM_ELLIPSOID;
  }
  input.setZero(input_num);
  vector3_t forceAverage(0.0, 0.0, mass_ * 9.81 / ROBOTS_FOOT_NUM);
  for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
    input.segment(ee*QUADRUPED_CONTACT_DIM, 3) = forceAverage;
  }
  for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
    input.segment(ee*QUADRUPED_CONTACT_DIM+24, 3) = forceAverage;
  }
  nextState = state;
}

}  // namespace multi_robot
}  // namespace ocs2
