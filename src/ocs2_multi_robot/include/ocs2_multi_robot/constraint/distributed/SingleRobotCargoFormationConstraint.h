#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/AlternatingPreComputation.h"

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Distributed constraint to maintain the geometric relationship between a robot and cargo.
 * 
 * This is the robot-side version of the formation constraint. It treats the cargo state
 * as fixed (from precomputation) and constrains the robot's position relative to the cargo.
 * 
 * The constraint is formulated as a box constraint in the cargo's local frame (x and y only):
 *   robot_local = R_cargo^T * (robot_COM - cargo_COM)
 *   nominal_offset - tolerance <= robot_local(x,y) <= nominal_offset + tolerance
 */
class SingleRobotCargoFormationConstraint final : public StateInputConstraint {
 public:
  /**
   * @brief Constructor
   * @param referenceManager Reference to the switched model reference manager
   * @param robotOffset Nominal offset of robot's COM from cargo COM in cargo frame [x, y, z, yaw, pitch, roll]
   * @param position_tolerance Maximum deviation from nominal position [x, y]
   * @param robotIndex Index of the robot (0-indexed)
   */
  SingleRobotCargoFormationConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                      const vector_t& robotOffset,
                                      const vector_t& position_tolerance,
                                      size_t robotIndex);

  ~SingleRobotCargoFormationConstraint() override = default;
  SingleRobotCargoFormationConstraint* clone() const override { return new SingleRobotCargoFormationConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 4; }  // 4 constraints (2 lower + 2 upper bounds for x, y)

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                   const PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  SingleRobotCargoFormationConstraint(const SingleRobotCargoFormationConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  vector_t robotOffset_;  // [x, y, z, yaw, pitch, roll]
  vector_t position_tolerance_;  // [x, y]
  size_t robotIndex_;
};

}  // namespace multi_robot
}  // namespace ocs2
