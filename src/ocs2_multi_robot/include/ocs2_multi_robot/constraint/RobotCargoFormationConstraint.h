#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Constraint to maintain the geometric relationship between robots and cargo
 * 
 * This constraint ensures that each robot's COM stays close to its nominal position
 * relative to the cargo, accounting for the cargo's position and orientation.
 * 
 * The constraint is formulated as a box constraint in the cargo's local frame (x and y only):
 * For each robot i:
 *   robot_i_local = R_cargo^T * (robot_i_COM - cargo_COM)
 *   nominal_offset_i_min <= robot_i_local(x,y) <= nominal_offset_i_max
 */
class RobotCargoFormationConstraint final : public StateInputConstraint {
 public:
  /**
   * @brief Constructor
   * @param referenceManager Reference to the switched model reference manager
   * @param robot1_offset Nominal offset of robot 1 COM from cargo COM in cargo frame [x, y, z, yaw, pitch, roll]
   * @param robot2_offset Nominal offset of robot 2 COM from cargo COM in cargo frame [x, y, z, yaw, pitch, roll]
   * @param position_tolerance Maximum deviation from nominal position [x, y] (z is not constrained)
   */
  RobotCargoFormationConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                 const vector_t& robot1_offset,
                                 const vector_t& robot2_offset,
                                 const vector_t& position_tolerance);

  ~RobotCargoFormationConstraint() override = default;
  RobotCargoFormationConstraint* clone() const override { return new RobotCargoFormationConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 8; }  // 4 per robot (2 lower + 2 upper bounds for x, y)

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                   const PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  RobotCargoFormationConstraint(const RobotCargoFormationConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  vector_t robot1_offset_;  // [x, y, z, yaw, pitch, roll]
  vector_t robot2_offset_;  // [x, y, z, yaw, pitch, roll]
  vector_t position_tolerance_;  // [x, y, z] max deviation
};

}  // namespace multi_robot
}  // namespace ocs2
