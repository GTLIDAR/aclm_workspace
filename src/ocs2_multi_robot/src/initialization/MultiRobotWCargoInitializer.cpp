#include "ocs2_multi_robot/initialization/MultiRobotWCargoInitializer.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiRobotWCargoInitializer::MultiRobotWCargoInitializer(const scalar_t& robot_mass, const scalar_t& cargo_mass, 
                                                         size_t numRobots, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
    : robot_mass_(robot_mass), cargo_mass_(cargo_mass), numRobots_(numRobots), referenceManagerPtr_(&referenceManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiRobotWCargoInitializer* MultiRobotWCargoInitializer::clone() const {
  return new MultiRobotWCargoInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotWCargoInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  
  // Determine input dimension based on state dimension
  size_t input_num;
  const size_t stateDim = state.rows();
  const size_t expectedStateDim = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  
  if (stateDim == expectedStateDim) {
    input_num = numRobots_ * SINGLE_ROBOT_INPUT_DIM + numRobots_ * ARM_CONTACT_DIM;
  } else if (stateDim == STATE_DIM_AE_WITH_ELLIPSOID) {
    input_num = INPUT_DIM_AE_WITH_ELLIPSOID;
  } else if (stateDim == STATE_DIM_AE) {
    input_num = INPUT_DIM_AE;
  } else if (stateDim == STATE_DIM_ELLIPSOID) {
    input_num = INPUT_DIM_ELLIPSOID;
  } else {
    input_num = numRobots_ * SINGLE_ROBOT_INPUT_DIM + numRobots_ * ARM_CONTACT_DIM;
  }
  
  input.setZero(input_num);
  
  // Compute force average per stance leg
  const size_t totalStanceLegs = std::count(contactFlags.begin(), contactFlags.end(), true);
  // const size_t totalStanceLegs = NUM_ROBOTS * QUADRUPED_FOOT_NUM;
  vector3_t forceAverage(0.0, 0.0, (robot_mass_ * numRobots_ + cargo_mass_) * 9.81 / static_cast<scalar_t>(totalStanceLegs));
  
  // Set forces for each robot's feet
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      const size_t contactIndex = robot * QUADRUPED_FOOT_NUM + ee;
      if (contactFlags[contactIndex]) {
        input.segment(robot * SINGLE_ROBOT_INPUT_DIM + ee * QUADRUPED_CONTACT_DIM, 3) = forceAverage;
      }
    }
  }

  // Get the average manipulation force per robot
  const vector3_t manipulationForceAverage(0.0, 0.0, cargo_mass_ * 9.81 / static_cast<scalar_t>(numRobots_));

  const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
  for (size_t arm = 0; arm < numRobots_; ++arm) {
    input.segment(cargoInputOffset + arm * ARM_CONTACT_DIM, 3) = manipulationForceAverage;
  }
  
  nextState = state;
}

}  // namespace multi_robot
}  // namespace ocs2

