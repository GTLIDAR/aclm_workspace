#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_ddp/DDP_Settings.h>

#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_quadruped/gait/ModeSequenceTemplate.h>

#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/common/AlternatingTargetTrajectories.h"
#include "ocs2_multi_robot/alternating_base/AlternatingOptSettings.h"
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/AlternatingPreComputation.h"
#include "ocs2_multi_robot/initialization/TwoQuadWCargoInitializer.h"

// Forward declarations for perceptive locomotion
namespace convex_plane_decomposition {
  struct PlanarTerrain;
}

namespace ocs2 {
namespace multi_robot {

// Forward declarations for perceptive locomotion
class MultiRobotGridMapHeightMap;
class MultiRobotConvexRegionSelector;

/**
 * Helper interface that constructs the decoupled optimal control problems
 * (payload and single-robot subsystems) used in the alternating / ADMM solver.
 *
 * For now, the payload subsystem is set up with dynamics and tracking cost.
 * Robot subsystems will be filled in subsequent steps.
 */
class AlternatingInterface {
 public:
  AlternatingInterface(const std::string& taskFile, const std::string& libraryFolder, bool verbose);

  ~AlternatingInterface() = default;

  std::shared_ptr<ReferenceManager> getReferenceManagerPtr() const { return referenceManagerPtr_; }

  std::shared_ptr<SwitchedModelReferenceManagerWithTerrain> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const std::shared_ptr<OptimalControlProblem>& getCargoOptimalControlProblem() const { return cargoOptimalControlProblem_; }

  const std::vector<std::shared_ptr<OptimalControlProblem>>& getSingleRobotOptimalControlProblems() const {
    return singleRobotOptimalControlProblems_;
  }

  const std::vector<std::shared_ptr<OptimalControlProblem>>& getAllOptimalControlProblems() const { return optimalControlProblems_; }

  const vector_t& getCargoInitialState() const { return cargoInitialState_; }
  const std::vector<vector_t>& getRobotInitialStates() const { return robotInitialStates_; }
  const std::vector<vector_t>& getHandlePositions() const { return handlePositions_; }

  const mpc::Settings& getMpcSettings() const { return mpcSettings_; }
  const sqp::Settings& getSqpSettings() const { return sqpSettings_; }
  const AlternatingOptSettings& getAlternatingSettings() const { return alternatingSettings_; }

  const RolloutBase& getCargoRollout() const { return *cargoRolloutPtr_; }
  const RolloutBase& getSingleRobotRollout(size_t robotId) const { return *singleRobotRolloutPtrs_.at(robotId); }
  const std::vector<std::unique_ptr<RolloutBase>>& getSingleRobotRollouts() const { return singleRobotRolloutPtrs_; }

  const Initializer& getCargoInitializer() const { return *cargoInitializerPtr_; }
  const Initializer& getSingleRobotInitializer(size_t robotId) const { return *singleRobotInitializerPtrs_.at(robotId); }
  const std::vector<std::unique_ptr<Initializer>>& getSingleRobotInitializers() const { return singleRobotInitializerPtrs_; }
  const Initializer& getTwoQuadWCargoInitializer() const { return *twoQuadWCargoInitializerPtr_; }

  AlternatingPreComputation& getCargoPreComputation() const { return *cargoPreComputation_; }
  AlternatingPreComputation& getRobotPreComputation(size_t robotId) const { return *robotPreComputationPtrs_.at(robotId); }
  const std::vector<AlternatingPreComputation*>& getRobotPreComputations() const { return robotPreComputationPtrs_; }

  void updatePreComputations(const AlternatingTargetTrajectories& trajectories);

  // Perceptive locomotion getters
  bool isPerceptiveMode() const { return usePerceptive_; }
  std::shared_ptr<ocs2::multi_robot::MultiRobotGridMapHeightMap> getGridMapHeightMapPtr() const { return gridMapHeightMapPtr_; }
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }
  std::shared_ptr<ocs2::multi_robot::MultiRobotConvexRegionSelector> getConvexRegionSelectorPtr() const { return convexRegionSelectorPtr_; }

 private:
  void initializeReferenceManager();
  void setupCargoOptimalControlProblem();
  void loadInitialConditions();
  void loadHandlePositions();
  void loadModelParameters();

  // TODO: add single-robot OCP assembly in subsequent steps.
  void setupSingleRobotOptimalControlProblems();

  // Helper loaders for ADMM settings.
  std::shared_ptr<quadruped::GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;
  std::string taskFile_;
  std::string libraryFolder_;
  bool verbose_;

  size_t numRobots_{0};
  size_t numArms_{0};

  std::shared_ptr<SwitchedModelReferenceManagerWithTerrain> referenceManagerPtr_;

  std::shared_ptr<OptimalControlProblem> cargoOptimalControlProblem_;
  std::vector<std::shared_ptr<OptimalControlProblem>> singleRobotOptimalControlProblems_;
  std::vector<std::shared_ptr<OptimalControlProblem>> optimalControlProblems_;

  std::vector<vector_t> handlePositions_;

  scalar_t cargoMass_{0.0};
  scalar_t robotMass_{0.0};
  matrix_t cargoInertia_;
  matrix_t robotInertia_;
  bool recompileLibraries_{false};
  vector_t rho_;

  vector_t cargoInitialState_;
  std::vector<vector_t> robotInitialStates_;

  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;
  AlternatingOptSettings alternatingSettings_;

  std::unique_ptr<RolloutBase> cargoRolloutPtr_;
  std::vector<std::unique_ptr<RolloutBase>> singleRobotRolloutPtrs_;

  std::unique_ptr<Initializer> cargoInitializerPtr_;
  std::vector<std::unique_ptr<Initializer>> singleRobotInitializerPtrs_;
  std::unique_ptr<Initializer> twoQuadWCargoInitializerPtr_;

  AlternatingPreComputation* cargoPreComputation_{nullptr};
  std::vector<AlternatingPreComputation*> robotPreComputationPtrs_;

  // Perceptive locomotion support
  bool usePerceptive_ = false;
  std::shared_ptr<ocs2::multi_robot::MultiRobotGridMapHeightMap> gridMapHeightMapPtr_;
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<ocs2::multi_robot::MultiRobotConvexRegionSelector> convexRegionSelectorPtr_;
};

}  // namespace multi_robot
}  // namespace ocs2


