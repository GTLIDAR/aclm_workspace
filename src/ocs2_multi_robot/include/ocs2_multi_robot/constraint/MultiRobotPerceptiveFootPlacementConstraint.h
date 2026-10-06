//
// Multi-robot PerceptiveFootPlacementConstraint for ocs2_multi_robot
// Supports multiple robots with terrain-aware foot placement constraints using convex regions
//

#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include "ocs2_multi_robot/precomputation/MultiRobotPerceptivePreComputation.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Perceptive foot placement constraint that enforces feet to be within convex safe regions
 * during stance phases. This uses the MultiRobotConvexRegionSelector to determine safe regions.
 * Adapted for multi-robot systems.
 * 
 * Uses MultiRobotPerceptivePreComputation for efficient caching of (A,b) parameters.
 */
class MultiRobotPerceptiveFootPlacementConstraint final : public StateInputConstraint {
 public:
  /**
   * Constructor
   * @param [in] referenceManager : Reference manager with terrain
   * @param [in] convexRegionSelector : Multi-robot convex region selector
   * @param [in] globalFootIndex : The global foot index (robotId * 4 + localFootIdx)
   * @param [in] valueIndex : Index for input mapping
   * @param [in] robotStateOffset : Offset to robot's state in the full state vector
   * @param [in] numVertices : Number of vertices for the convex polygon constraint
   */
  MultiRobotPerceptiveFootPlacementConstraint(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                               MultiRobotConvexRegionSelector& convexRegionSelector,
                                               size_t globalFootIndex,
                                               size_t valueIndex,
                                               size_t robotStateOffset,
                                               size_t numVertices = 8);

  ~MultiRobotPerceptiveFootPlacementConstraint() override = default;
  MultiRobotPerceptiveFootPlacementConstraint* clone() const override { 
    return new MultiRobotPerceptiveFootPlacementConstraint(*this); 
  }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return numVertices_; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, 
                    const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, 
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  MultiRobotPerceptiveFootPlacementConstraint(const MultiRobotPerceptiveFootPlacementConstraint& rhs);

  /**
   * Get foot position from state for multi-robot system
   */
  vector3_t getFootPositionFromState(const vector_t& state) const;

  /**
   * Try to get cached parameters from PreComputation, fallback to direct computation
   */
  const FootPlacementParameters& getParameters(scalar_t time, const PreComputation& preComp) const;

  /**
   * Compute parameters directly (fallback when PreComputation not available)
   */
  FootPlacementParameters computeParametersFallback(scalar_t time) const;
  
  std::pair<matrix_t, vector_t> getPolygonConstraint(
      const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  MultiRobotConvexRegionSelector* convexRegionSelectorPtr_;
  const size_t globalFootIndex_;       // Global foot index (robotId * 4 + localFootIdx)
  const size_t valueIndex_;            // Index for input mapping
  const size_t robotStateOffset_;      // Offset to robot's state segment
  const size_t localFootIndex_;        // Local foot index within robot (0-3)
  const size_t numVertices_;

  mutable FootPlacementParameters fallbackParam_;  // Used when PreComputation not available
};

}  // namespace multi_robot
}  // namespace ocs2
