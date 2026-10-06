//
// Multi-robot Foot Collision Constraint using Signed Distance Field
// More sophisticated 3D collision avoidance compared to height-map-only approach
//

#pragma once

#include <memory>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

#include "ocs2_multi_robot/reference_manager/PerceptiveMultiRobotReferenceManager.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Foot collision constraint using Signed Distance Field (SDF)
 * 
 * This constraint uses a 3D SDF computed from the elevation map to ensure
 * the foot maintains safe clearance from ALL obstacles, not just the ground surface.
 * 
 * Unlike height-map-based constraints that only check vertical distance to ground,
 * SDF-based constraint detects collisions with:
 * - Ground surface
 * - Walls and vertical obstacles  
 * - Overhangs and ceilings
 * - Any 3D obstacle encoded in the SDF
 * 
 * Constraint formulation (h >= 0):
 *   h = SDF(foot_position) - clearance
 * 
 * - SDF > 0: foot is in free space
 * - SDF = 0: foot is on obstacle surface
 * - SDF < 0: foot is inside obstacle (collision!)
 */
class MultiRobotFootCollisionConstraint final : public StateInputConstraint {
 public:
  /**
   * @brief Constructor
   * @param referenceManager Reference to the perceptive multi-robot reference manager
   * @param globalFootIndex Global foot index across all robots (0 to numRobots*4-1)
   * @param robotStateOffset Offset to this robot's state in the full state vector
   * @param clearance Minimum safe distance from obstacles (meters)
   */
  MultiRobotFootCollisionConstraint(
      PerceptiveMultiRobotReferenceManager& referenceManager,
      size_t globalFootIndex,
      size_t robotStateOffset,
      scalar_t clearance = 0.02);

  ~MultiRobotFootCollisionConstraint() override = default;
  
  MultiRobotFootCollisionConstraint* clone() const override { 
    return new MultiRobotFootCollisionConstraint(*this); 
  }

  /** Check if SDF is available (from reference manager) */
  bool hasSDF() const { return referenceManagerPtr_->getSDF() != nullptr; }

  /** 
   * Active during swing phase only, with buffer around phase transitions
   * to avoid numerical issues near touchdown/liftoff 
   */
  bool isActive(scalar_t time) const override;
  
  size_t getNumConstraints(scalar_t time) const override { return 1; }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                   const PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const PreComputation& preComp) const override;

 private:
  MultiRobotFootCollisionConstraint(const MultiRobotFootCollisionConstraint& rhs);
  
  /** Extract foot position from state vector */
  vector3_t getFootPositionFromState(const vector_t& state) const;

  const PerceptiveMultiRobotReferenceManager* referenceManagerPtr_;
  
  size_t globalFootIndex_;
  size_t robotStateOffset_;
  size_t localFootIndex_;
  scalar_t clearance_;
  
  // Time buffer around phase transitions to avoid activating too close to touchdown/liftoff
  static constexpr scalar_t phaseTransitionBuffer_ = 0.05;  // 50ms
};

}  // namespace multi_robot
}  // namespace ocs2
