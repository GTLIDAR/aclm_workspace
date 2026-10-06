#include "ocs2_multi_robot/initialization/SingleRobotInitializer.h"

#include <ocs2_quadruped/common/utils.h>

namespace ocs2 {
namespace multi_robot {

SingleRobotInitializer::SingleRobotInitializer(scalar_t robotMass, size_t numFeet, scalar_t cargoMass, size_t robotId,
                                               size_t numRobots, const SwitchedModelReferenceManagerWithTerrain& referenceManager)
    : robotMass_(robotMass), numFeet_(numFeet), cargoMass_(cargoMass), robotId_(robotId), numRobots_(numRobots),
      referenceManagerPtr_(&referenceManager) {}

SingleRobotInitializer* SingleRobotInitializer::clone() const {
  return new SingleRobotInitializer(*this);
}

void SingleRobotInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  input.setZero(ALTERNATING_SINGLE_ROBOT_INPUT_DIM);
  if (numFeet_ > 0) {
    // Use per-robot contact flags to compute number of stance legs, similar to SingleRobotQuadraticTrackingCost
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const size_t offset = robotId_ * QUADRUPED_FOOT_NUM;
    const quadruped::contact_flag_t contactFlagsSingleRobot = quadruped::contact_flag_t{
        contactFlags[offset + 0], contactFlags[offset + 1], contactFlags[offset + 2], contactFlags[offset + 3]};
    const auto numStanceLegs = quadruped::numberOfClosedContacts(contactFlagsSingleRobot);

    if (numStanceLegs > 0) {
      // Leg forces: average force per stance leg, consistent with SingleRobotQuadraticTrackingCost
      const vector3_t legForceAverage(
          0.0, 0.0,
          (robotMass_ + cargoMass_ / static_cast<scalar_t>(numRobots_)) * 9.81 / static_cast<scalar_t>(numStanceLegs));

      for (size_t foot = 0; foot < numFeet_; ++foot) {
        if (contactFlags[offset + foot]) {
          input.segment(foot * QUADRUPED_CONTACT_DIM, 3) = legForceAverage;
        }
      }
    }
  }
  
  // Arm force: negative average force (force applied ON the robot, opposite to what robot applies)
  // The cargo weight is distributed among all arms (NUM_ROBOTS), so each arm carries cargoMass * g / NUM_ROBOTS
  // For a single robot, the force applied ON it is negative (opposite direction to what the robot applies)
  // Only set the force part (first 3 elements), not the torque part
  vector3_t armForceAverage(0.0, 0.0, -cargoMass_ * 9.81 / static_cast<scalar_t>(numRobots_));
  input.segment(SINGLE_ROBOT_INPUT_DIM, 3) = armForceAverage;
  
  nextState = state;
}

}  // namespace multi_robot
}  // namespace ocs2


