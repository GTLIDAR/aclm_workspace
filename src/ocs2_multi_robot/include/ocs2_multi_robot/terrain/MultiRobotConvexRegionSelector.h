//
// Simplified Multi-robot ConvexRegionSelector for ocs2_multi_robot
// Provides terrain-aware foot placement constraints for multiple robots
//

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/terrain/HeightMap.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Simplified ConvexRegionSelector for multi-robot foot placement constraints.
 * 
 * Key functions:
 * - computeNominalFoothold(): Offset from robot base target trajectory at standMiddleTime
 * - update(): Compute projected foothold with terrain awareness using penalty function + getBestPlanarRegionAtPositionInWorld
 * - extractContactFlags(): Extract contact flags from mode sequence
 * - getProjection(): Get planar terrain projection for a foot
 * - getConvexPolygon(): Get convex polygon for foot placement constraint
 * - getNominalFoothold(): Get nominal foothold position
 * - Visualization of nominal and projected footholds
 */
class MultiRobotConvexRegionSelector {
 public:
  MultiRobotConvexRegionSelector(size_t numRobots,
                                  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                  std::shared_ptr<HeightMap> heightMapPtr,
                                  size_t numVertices = 8,
                                  double comHeight = 0.5,
                                  double minPlanarHeight = -std::numeric_limits<double>::infinity(),
                                  double maxPlanarHeight = std::numeric_limits<double>::infinity());

  /**
   * Update convex regions based on mode schedule and state.
   * Computes nominal footholds and projected footholds for all feet.
   * Uses penalty function + getBestPlanarRegionAtPositionInWorld for terrain-aware placement.
   */
  void update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState, 
              TargetTrajectories& targetTrajectories);

  /**
   * Get the planar terrain projection for a specific foot at a given time
   * @param globalFootIndex Global foot index (robotId * 4 + localFootIdx)
   */
  convex_plane_decomposition::PlanarTerrainProjection getProjection(size_t globalFootIndex, scalar_t time) const;

  /**
   * Get the convex polygon for a specific foot at a given time
   */
  convex_plane_decomposition::CgalPolygon2d getConvexPolygon(size_t globalFootIndex, scalar_t time) const;

  /**
   * Get the nominal foothold position for a specific foot at a given time
   */
  vector3_t getNominalFoothold(size_t globalFootIndex, scalar_t time) const;

  /**
   * Check if foot placement constraint should be active for a foot at given time
   */
  bool isFootPlacementActive(size_t globalFootIndex, scalar_t time) const;

  /**
   * Extract contact flags from mode sequence for all robots
   */
  std::vector<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * Get the planar terrain pointer
   */
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() { return planarTerrainPtr_; }

  /** Get height map pointer */
  std::shared_ptr<HeightMap> getHeightMap() const { return heightMapPtr_; }

  /**
   * Initialize visualization publisher
   */
  void initializeVisualization(ros::NodeHandle& nh, const std::string& frameId = "odom");

  /**
   * Publish visualization of nominal and projected footholds
   */
  void publishVisualization(scalar_t currentTime) const;

  /** Get number of robots */
  size_t getNumRobots() const { return numRobots_; }

  /** Get total number of feet across all robots */
  size_t getTotalFeet() const { return numRobots_ * QUADRUPED_FOOT_NUM; }

  /** Get height bounds (used by swing trajectory planner) */
  double getMinPlanarHeight() const { return minPlanarHeight_; }
  double getMaxPlanarHeight() const { return maxPlanarHeight_; }

 private:
  /**
   * Compute nominal foothold: offset from robot base target trajectory at given time
   */
  vector3_t computeNominalFoothold(size_t globalFootIndex, scalar_t time, const vector_t& initState, 
                                    TargetTrajectories& targetTrajectories);

  /**
   * Project nominal foothold onto planar terrain using penalty function
   */
  vector3_t projectFootholdOntoTerrain(const vector3_t& nominalFoothold, scalar_t robotBaseZ);

  /**
   * Find stance phase start/end indices
   */
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  /**
   * Get foot position from state vector
   */
  vector3_t getFootPositionFromState(size_t globalFootIndex, const vector_t& state) const;

  /** Publish nominal and projected foothold markers */
  void publishFootholdVisualization(scalar_t currentTime) const;

  /** Publish constraint polygon markers on separate topic */
  void publishPolygonVisualization(scalar_t currentTime) const;

  // Per-foot data (indexed by global foot index: robotId * 4 + localFootIdx)
  std::vector<std::vector<convex_plane_decomposition::PlanarTerrainProjection>> feetProjections_;
  std::vector<std::vector<convex_plane_decomposition::CgalPolygon2d>> convexPolygons_;
  std::vector<std::vector<vector3_t>> nominalFootholds_;
  std::vector<std::vector<scalar_t>> middleTimes_;
  std::vector<std::vector<scalar_t>> timeEvents_;

  size_t numRobots_;
  size_t numVertices_;

  convex_plane_decomposition::PlanarTerrain planarTerrain_;
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;

  // Height filtering for planar regions
  double minPlanarHeight_;
  double maxPlanarHeight_;
  double comHeight_;

  // Height map for terrain queries
  std::shared_ptr<HeightMap> heightMapPtr_;

  // Visualization
  ros::Publisher footholdPublisher_;
  ros::Publisher polygonPublisher_;
  std::string visualizationFrameId_ = "odom";
  bool visualizationInitialized_ = false;
};

}  // namespace multi_robot
}  // namespace ocs2
