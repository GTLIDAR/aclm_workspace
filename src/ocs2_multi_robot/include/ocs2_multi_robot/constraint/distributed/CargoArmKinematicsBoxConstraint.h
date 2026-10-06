#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/AlternatingPreComputation.h"

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Distributed constraint for cargo-side arm kinematics box constraint.
 * 
 * This is the cargo-side version of the arm kinematics constraint. It treats the robot states
 * as fixed (from precomputation) and constrains the handle positions relative to the robot bases.
 * 
 * For each robot, the constraint enforces:
 *   lower_bound <= R_robot^T * (handle_world - robot_world) <= upper_bound
 */
class CargoArmKinematicsBoxConstraint final : public StateInputConstraint {
 public:
  /**
   * @brief Constructor
   * @param referenceManager Reference to the switched model reference manager
   * @param handlePositions Vector of handle positions relative to cargo [x, y, z, yaw, pitch, roll] for each robot
   * @param numRobots Number of robots
   */
  CargoArmKinematicsBoxConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                   const std::vector<vector_t>& handlePositions,
                                   size_t numRobots);

  ~CargoArmKinematicsBoxConstraint() override = default;
  CargoArmKinematicsBoxConstraint* clone() const override { return new CargoArmKinematicsBoxConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return numRobots_ * 6; }  // 6 per robot (3 lower + 3 upper bounds)

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                   const PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  CargoArmKinematicsBoxConstraint(const CargoArmKinematicsBoxConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  std::vector<vector_t> handlePositions_;
  size_t numRobots_;
};

}  // namespace multi_robot
}  // namespace ocs2
