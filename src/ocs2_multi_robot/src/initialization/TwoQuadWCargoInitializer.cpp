#include "ocs2_multi_robot/initialization/TwoQuadWCargoInitializer.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TwoQuadWCargoInitializer::TwoQuadWCargoInitializer(const scalar_t& robot_mass, const scalar_t& cargo_mass, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
    : robot_mass_(robot_mass), cargo_mass_(cargo_mass), referenceManagerPtr_(&referenceManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TwoQuadWCargoInitializer* TwoQuadWCargoInitializer::clone() const {
  return new TwoQuadWCargoInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TwoQuadWCargoInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  size_t input_num;
  if (state.rows() == STATE_DIM_AE_WITH_ELLIPSOID) {
    input_num = INPUT_DIM_AE_WITH_ELLIPSOID;
  } else if (state.rows() == STATE_DIM_AE) {
    input_num = INPUT_DIM_AE;
  } else if (state.rows() == STATE_DIM_ELLIPSOID) {
    input_num = INPUT_DIM_ELLIPSOID;
  } else if (state.rows() == STATE_DIM_TWO_QUAD_W_CARGO) {
    input_num = INPUT_DIM_TWO_QUAD_W_CARGO;
  }
  input.setZero(input_num);
  // This initializer is specifically for two quadrupeds (8 feet total), independent of the global NUM_ROBOTS.
  constexpr size_t TWO_QUAD_NUM_ROBOTS = 2;
  constexpr size_t TWO_QUAD_FOOT_NUM = TWO_QUAD_NUM_ROBOTS * QUADRUPED_FOOT_NUM;  // 8

  vector3_t forceAverage(0.0, 0.0, (robot_mass_ + cargo_mass_ / 2.0) * 9.81 / TWO_QUAD_FOOT_NUM);
  for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
    input.segment(ee*QUADRUPED_CONTACT_DIM, 3) = forceAverage;
  }
  for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
    input.segment(ee*QUADRUPED_CONTACT_DIM+24, 3) = forceAverage;
  }

  // Get the average manipulation force (two arms total)
  const vector3_t manipulationForceAverage(0.0, 0.0, cargo_mass_ * 9.81 / static_cast<scalar_t>(TWO_QUAD_NUM_ROBOTS));

  for (size_t arm = 0; arm < TWO_QUAD_NUM_ROBOTS; ++arm) {
    input.segment(SINGLE_ROBOT_INPUT_DIM * TWO_QUAD_NUM_ROBOTS + arm * ARM_CONTACT_DIM, 3) = manipulationForceAverage;
  }
  nextState = state;
}

}  // namespace multi_robot
}  // namespace ocs2
