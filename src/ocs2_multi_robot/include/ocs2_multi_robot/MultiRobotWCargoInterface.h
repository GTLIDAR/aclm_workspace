#pragma once

#include <pinocchio/fwd.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/PreComputation.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_quadruped/gait/ModeSequenceTemplate.h>
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/dynamics/CentroidalDynamicsAD.h"
#include "ocs2_multi_robot/dynamics/CentroidalDynamicsWithEllipsoidAD.h"
#include "ocs2_multi_robot/dynamics/MultiRobotWCargoSRBDAD.h"
#include "ocs2_multi_robot/constraint/FrictionConeConstraint.h"
#include "ocs2_multi_robot/constraint/ZeroForceConstraint.h"
#include "ocs2_multi_robot/constraint/ZeroVelocityConstraint.h"
#include "ocs2_multi_robot/constraint/NormalVelocityConstraint.h"
#include "ocs2_multi_robot/constraint/FootPlacementConstraint.h"
#include "ocs2_multi_robot/constraint/KinematicsBoxConstraint.h"
#include "ocs2_multi_robot/constraint/MultiRobotArmKinematicsBoxConstraint.h"
#include "ocs2_multi_robot/constraint/MultiRobotCargoFormationConstraint.h"
#include "ocs2_multi_robot/constraint/ManipulationFrictionConeConstraint.h"
#include "ocs2_multi_robot/constraint/ManipulationTorqueBoxConstraint.h"
#include "ocs2_multi_robot/constraint/ObjectCBFConstraint.h"
#include "ocs2_multi_robot/constraint/ObjectBoundConstraint.h"
#include "ocs2_multi_robot/constraint/RobotCBFConstraint.h"
#include "ocs2_multi_robot/constraint/MultiRobotPerceptiveFootPlacementConstraint.h"
#include "ocs2_multi_robot/terrain/HeightMap.h"
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include "ocs2_multi_robot/terrain/MultiRobotGridMapHeightMap.h"
#include "ocs2_multi_robot/initialization/MultiRobotWCargoInitializer.h"
#include "ocs2_multi_robot/cost/MultiRobotWCargoTrackingCost.h"
#include "ocs2_multi_robot/robot_interface/MultiRobotInterfaceAbstract.h"


namespace ocs2 {
namespace multi_robot {

class MultiRobotWCargoInterface : public MultiRobotInterfaceAbstract {
public:
  MultiRobotWCargoInterface(const std::string& taskFile, const std::string& libraryFolder, bool verbose, bool loadCostMatrices);

  ~MultiRobotWCargoInterface() override = default;
  matrix_t cargo_inertia_;

  // Perceptive mode getters
  bool isPerceptiveMode() const { return usePerceptive_; }
  std::shared_ptr<MultiRobotConvexRegionSelector> getConvexRegionSelectorPtr() const { return convexRegionSelectorPtr_; }
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }
  std::shared_ptr<MultiRobotGridMapHeightMap> getGridMapHeightMapPtr() const { return gridMapHeightMapPtr_; }
  
  // Obstacle avoidance getter
  bool isObstacleAvoidanceEnabled() const { return enableObstacleAvoidance_; }

private:

  void setupOptimalControlProblem();

  std::shared_ptr<quadruped::GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

  std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose) const;

  RelaxedBarrierPenalty::Config loadKinematicsBoxSettings(const std::string& taskFile, bool verbose) const;

  RelaxedBarrierPenalty::Config loadFootPlacementSettings(const std::string& taskFile, bool verbose) const;

  RelaxedBarrierPenalty::Config loadPerceptiveFootPlacementSettings(const std::string& taskFile, bool verbose) const;

  RelaxedBarrierPenalty::Config loadArmKinematicsBoxSettings(const std::string& taskFile, bool verbose) const;
  
  std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadManipulationFrictionConeSettings(const std::string& taskFile, bool verbose) const;
  
  std::pair<vector3_t, vector3_t> loadManipulationTorqueBounds(const std::string& taskFile, bool verbose) const;
  
  RelaxedBarrierPenalty::Config loadManipulationTorqueBoxSettings(const std::string& taskFile, bool verbose) const;
  
  RelaxedBarrierPenalty::Config loadOrientationConstraintSettings(const std::string& taskFile, bool verbose) const;
  
  RelaxedBarrierPenalty::Config loadFormationConstraintSettings(const std::string& taskFile, bool verbose) const;
  
  std::unique_ptr<StateInputCost> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                            const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);

  std::unique_ptr<StateInputCost> getKinematicsBoxConstraint(size_t contactPointIndex,
                                                             const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputCost> getFootPlacementConstraint(size_t contactPointIndex,
                                                             const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputCost> getPerceptiveFootPlacementConstraint(size_t contactPointIndex,
                                                                       const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputCost> getArmKinematicsBoxConstraint(const std::vector<vector_t>& robotHandles,
                                                               const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  
  size_t numRobots_;
  std::vector<std::string> robotFootNames_;
  bool loadCostMatrices_;
  bool usePerceptive_;
  bool enableObstacleAvoidance_;
  std::shared_ptr<MultiRobotConvexRegionSelector> convexRegionSelectorPtr_;
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<MultiRobotGridMapHeightMap> gridMapHeightMapPtr_;
  std::shared_ptr<Obstacles> obstaclesBox1Ptr_, obstaclesBox2Ptr_, obstaclesWall1Ptr_, obstaclesWall2Ptr_, obstaclesWall3Ptr_;
};
}
}

