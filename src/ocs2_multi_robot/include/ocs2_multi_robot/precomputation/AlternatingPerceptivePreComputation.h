#pragma once

#include "ocs2_multi_robot/AlternatingPreComputation.h"
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include "ocs2_multi_robot/precomputation/MultiRobotPerceptivePreComputation.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Perceptive precomputation for alternating interface.
 * Extends AlternatingPreComputation with terrain-aware foot placement caching.
 */
class AlternatingPerceptivePreComputation : public AlternatingPreComputation {
 public:
  AlternatingPerceptivePreComputation(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                      const std::vector<vector_t>& handlePositions,
                                      const AlternatingTargetTrajectories& previousSolution,
                                      size_t numDualVariables,
                                      std::shared_ptr<MultiRobotConvexRegionSelector> convexRegionSelectorPtr,
                                      size_t numVertices = 8);

  ~AlternatingPerceptivePreComputation() override = default;

  AlternatingPerceptivePreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  /**
   * Get cached foot placement parameters for a specific foot
   * @param globalFootIndex Global foot index (robotId * 4 + localFootIdx)
   */
  const FootPlacementParameters& getFootPlacementParameters(size_t globalFootIndex) const {
    return perceptivePreComp_->getFootPlacementParameters(globalFootIndex);
  }

 private:
  AlternatingPerceptivePreComputation(const AlternatingPerceptivePreComputation& other) = default;

  // Store constructor parameters for cloning
  std::vector<vector_t> handlePositions_;
  size_t numDualVariables_;
  size_t numVertices_;
  std::shared_ptr<MultiRobotConvexRegionSelector> convexRegionSelectorPtr_;
  std::unique_ptr<MultiRobotPerceptivePreComputation> perceptivePreComp_;
};

}  // namespace multi_robot
}  // namespace ocs2
