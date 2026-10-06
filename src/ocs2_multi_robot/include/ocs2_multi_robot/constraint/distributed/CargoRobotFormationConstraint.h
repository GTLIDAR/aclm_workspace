#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/AlternatingPreComputation.h"

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Distributed constraint to maintain the geometric relationship between cargo and robots.
 * 
 * This is the cargo-side version of the formation constraint. It treats all robot states
 * as fixed (from precomputation) and constrains the cargo's position and orientation
 * relative to the robots.
 * 
 * The constraint is formulated as a box constraint in the cargo's local frame (x and y only):
 *   robot_i_local = R_cargo^T * (robot_i_COM - cargo_COM)
 *   nominal_offset_i - tolerance <= robot_i_local(x,y) <= nominal_offset_i + tolerance
 */
class CargoRobotFormationConstraint final : public StateInputConstraint {
 public:
  /**
   * @brief Constructor
   * @param referenceManager Reference to the switched model reference manager
   * @param robotOffsets Vector of nominal offsets of each robot's COM from cargo COM in cargo frame [x, y, z, yaw, pitch, roll]
   * @param position_tolerance Maximum deviation from nominal position [x, y]
   * @param numRobots Number of robots
   */
  CargoRobotFormationConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                const std::vector<vector_t>& robotOffsets,
                                const vector_t& position_tolerance,
                                size_t numRobots);

  ~CargoRobotFormationConstraint() override = default;
  CargoRobotFormationConstraint* clone() const override { return new CargoRobotFormationConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return numRobots_ * 4; }  // 4 per robot (2 lower + 2 upper bounds for x, y)

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                   const PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  CargoRobotFormationConstraint(const CargoRobotFormationConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  std::vector<vector_t> robotOffsets_;  // Vector of [x, y, z, yaw, pitch, roll] for each robot
  vector_t position_tolerance_;  // [x, y]
  size_t numRobots_;
};

}  // namespace multi_robot
}  // namespace ocs2
