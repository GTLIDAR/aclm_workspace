#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include "ocs2_multi_robot/precomputation/AlternatingPerceptivePreComputation.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Distributed perceptive foot placement constraint for single robot problems.
 * Constrains feet to convex safe regions during stance phases.
 * Uses AlternatingPerceptivePreComputation to access cached terrain data.
 */
class SingleRobotPerceptiveFootPlacementConstraint final : public StateInputConstraint {
 public:
  SingleRobotPerceptiveFootPlacementConstraint(
      SwitchedModelReferenceManagerWithTerrain& referenceManager,
      MultiRobotConvexRegionSelector& convexRegionSelector,
      size_t robotIndex,
      size_t localFootIndex,
      size_t numVertices = 8);

  ~SingleRobotPerceptiveFootPlacementConstraint() override = default;
  SingleRobotPerceptiveFootPlacementConstraint* clone() const override { 
    return new SingleRobotPerceptiveFootPlacementConstraint(*this); 
  }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return numVertices_; }
  
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, 
                    const PreComputation& preComp) const override;
  
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, 
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  SingleRobotPerceptiveFootPlacementConstraint(const SingleRobotPerceptiveFootPlacementConstraint& rhs) = default;

  vector3_t getFootPositionFromState(const vector_t& state) const;
  const FootPlacementParameters& getParameters(scalar_t time, const PreComputation& preComp) const;
  FootPlacementParameters computeParametersFallback(scalar_t time) const;
  std::pair<matrix_t, vector_t> getPolygonConstraint(
      const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  MultiRobotConvexRegionSelector* convexRegionSelectorPtr_;
  const size_t robotIndex_;
  const size_t localFootIndex_;
  const size_t globalFootIndex_;  // robotIndex * 4 + localFootIndex
  const size_t numVertices_;
  mutable FootPlacementParameters fallbackParam_;
};

}  // namespace multi_robot
}  // namespace ocs2
