//
// Perceptive Reference Manager for Multi-Robot Centroidal MPC with Convex Region Selection
//

#pragma once

#include <ros/ros.h>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Reference manager for perceptive multi-robot centroidal MPC.
 * Extends SwitchedModelReferenceManagerWithTerrain to update MultiRobotConvexRegionSelector
 * during reference modification.
 * 
 * Visualization and projected foothold computation are delegated to MultiRobotConvexRegionSelector.
 */
class PerceptiveMultiRobotReferenceManager : public SwitchedModelReferenceManagerWithTerrain {
 public:
  PerceptiveMultiRobotReferenceManager(std::shared_ptr<quadruped::GaitSchedule> gaitSchedulePtr,
                                        std::vector<std::shared_ptr<quadruped::SwingTrajectoryPlanner>> swingTrajectoryPlannerPtrs,
                                        std::shared_ptr<HeightMap> heightMapPtr,
                                        std::shared_ptr<MultiRobotConvexRegionSelector> convexRegionSelectorPtr,
                                        size_t numRobots,
                                        scalar_t comHeight = 0.5,
                                        scalar_t cargoHeightOffset = 0.8,
                                        scalar_t baseSwingHeight = 0.10);

  ~PerceptiveMultiRobotReferenceManager() override = default;

  std::shared_ptr<MultiRobotConvexRegionSelector> getConvexRegionSelectorPtr() { return convexRegionSelectorPtr_; }

  /** Get swing trajectory planner for a specific robot */
  const std::shared_ptr<quadruped::SwingTrajectoryPlanner>& getSwingTrajectoryPlannerForRobot(size_t robotIndex) const {
    return robotSwingTrajectoryPlannerPtrs_.at(robotIndex);
  }

  /** Set robot base offsets relative to cargo for terrain-averaged cargo z computation */
  void setRobotBaseOffsets(const std::vector<vector_t>& offsets) {
    robotBaseOffsets_ = offsets;
  }

  /** Get the current SDF (may be nullptr if not available) */
  std::shared_ptr<grid_map::SignedDistanceField> getSDF() const { return sdfPtr_; }

  /** Update the SDF from terrain data */
  void updateSDF(std::shared_ptr<grid_map::SignedDistanceField> sdf) { sdfPtr_ = std::move(sdf); }

 private:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) override;

  /** Update swing trajectory for all robots using height map and target trajectories */
  void updateAllRobotsSwingTrajectory(scalar_t initTime, const vector_t& initState, 
                                       const TargetTrajectories& targetTrajectories,
                                       ModeSchedule& modeSchedule);

  /** 
   * Compute adaptive swing height to clear terrain obstacles along swing path
   */
  scalar_t computeAdaptiveSwingHeight(scalar_t liftoffX, scalar_t liftoffY, scalar_t liftoffZ,
                                       scalar_t touchdownX, scalar_t touchdownY, scalar_t touchdownZ,
                                       scalar_t baseSwingHeight, scalar_t clearance = 0.03) const;

  std::shared_ptr<MultiRobotConvexRegionSelector> convexRegionSelectorPtr_;
  std::vector<std::shared_ptr<quadruped::SwingTrajectoryPlanner>> robotSwingTrajectoryPlannerPtrs_;
  std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
  size_t numRobots_;
  scalar_t comHeight_;
  scalar_t baseSwingHeight_{0.10};
  std::vector<vector_t> robotBaseOffsets_;  ///< Robot base offsets relative to cargo [x,y,z,yaw,pitch,roll]
  scalar_t cargoHeightOffset_{0.8};        ///< Cargo COM height above terrain
};

}  // namespace multi_robot
}  // namespace ocs2
