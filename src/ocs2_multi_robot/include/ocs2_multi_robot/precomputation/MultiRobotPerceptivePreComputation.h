//
// PreComputation for Multi-robot Perceptive Foot Placement Constraints
// Caches (A,b) halfspace parameters for all feet to avoid redundant computation
//

#pragma once

#include <ocs2_core/PreComputation.h>
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Cached parameters for foot placement constraint
 */
struct FootPlacementParameters {
  matrix_t a;      // Halfspace A matrix (numVertices x 3)
  vector_t b;      // Halfspace b vector (numVertices x 1)
  bool valid = false;
};

/**
 * PreComputation class for multi-robot perceptive foot placement.
 * Computes and caches (A,b) halfspace parameters for all feet once per (t,x,u) evaluation.
 */
class MultiRobotPerceptivePreComputation : public PreComputation {
 public:
  MultiRobotPerceptivePreComputation(MultiRobotConvexRegionSelector& convexRegionSelector,
                                      size_t numRobots,
                                      size_t numVertices = 8);

  ~MultiRobotPerceptivePreComputation() override = default;

  MultiRobotPerceptivePreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  /**
   * Get cached parameters for a specific foot
   * @param globalFootIndex Global foot index (robotId * 4 + localFootIdx)
   * @return Cached FootPlacementParameters
   */
  const FootPlacementParameters& getFootPlacementParameters(size_t globalFootIndex) const;

  /**
   * Get the last time for which parameters were computed
   */
  scalar_t getLastComputedTime() const { return lastComputedTime_; }

 protected:
  MultiRobotPerceptivePreComputation(const MultiRobotPerceptivePreComputation& other);

 private:
  void computeFootPlacementParameters(size_t globalFootIndex, scalar_t time);
  
  std::pair<matrix_t, vector_t> getPolygonConstraint(
      const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  MultiRobotConvexRegionSelector* convexRegionSelectorPtr_;
  size_t numRobots_;
  size_t numVertices_;
  size_t totalFeet_;
  
  std::vector<FootPlacementParameters> footPlacementParams_;
  scalar_t lastComputedTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
};

}  // namespace multi_robot
}  // namespace ocs2
