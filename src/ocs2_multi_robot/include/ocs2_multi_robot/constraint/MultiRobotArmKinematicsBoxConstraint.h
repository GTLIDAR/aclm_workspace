#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Enforce a kinematics box for each robot's arm EE
*/
class MultiRobotArmKinematicsBoxConstraint final : public StateInputConstraint {
public:
  /*
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager.
   * @param [in] robotHandles : Vector of 6D handle poses for each robot [x, y, z, yaw, pitch, roll]
   * @param [in] numRobots : Number of robots
   */
  MultiRobotArmKinematicsBoxConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager, 
    const std::vector<vector_t>& robotHandles, size_t numRobots);

  ~MultiRobotArmKinematicsBoxConstraint() override = default;
  MultiRobotArmKinematicsBoxConstraint* clone() const override { return new MultiRobotArmKinematicsBoxConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return numRobots_ * 6; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

private:
  MultiRobotArmKinematicsBoxConstraint(const MultiRobotArmKinematicsBoxConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  std::vector<vector_t> robotHandles_;
  size_t numRobots_;
};

}  // namespace multi_robot
}  // namespace ocs2

