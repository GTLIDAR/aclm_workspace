#include "ocs2_multi_robot/initialization/CargoInitializer.h"

namespace ocs2 {
namespace multi_robot {

CargoInitializer::CargoInitializer(scalar_t cargoMass, size_t numArms) : cargoMass_(cargoMass), numArms_(numArms) {}

CargoInitializer* CargoInitializer::clone() const {
  return new CargoInitializer(*this);
}

void CargoInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  input.setZero(numArms_ * ARM_CONTACT_DIM);
  if (numArms_ > 0) {
    const vector3_t forceAverage(0.0, 0.0, cargoMass_ * 9.81 / static_cast<scalar_t>(numArms_));
    for (size_t arm = 0; arm < numArms_; ++arm) {
      input.segment(arm * ARM_CONTACT_DIM, 3) = forceAverage;
    }
  }
  nextState = state;
}

}  // namespace multi_robot
}  // namespace ocs2


