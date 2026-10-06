//
// Created by zzhou387 on 9/9/22.
// Refer to ocs2_legged_robot
//

#ifndef OCS2_INTERFACE_LEGGEDROBOTINTERFACE_H
#define OCS2_INTERFACE_LEGGEDROBOTINTERFACE_H
// ocs2
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>
#include "ocs2_quadruped/common/ModelSettings.h"
#include "ocs2_quadruped/initialization/LeggedRobotInitializer.h"
#include "ocs2_quadruped/reference_manager/SwitchedModelReferenceManager.h"
#include <jsoncpp/json/json.h>

namespace ocs2 {
namespace quadruped {

class LeggedRobotInterface : public RobotInterface {
public:
  LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, 
                       const std::string& referenceFile, const std::string& rebarFile);

  ~LeggedRobotInterface() override = default;
  
  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, 
                                  const std::string& referenceFile, const std::string& rebarFile, bool verbose);

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& sqpSettings() { return sqpSettings_; }

  const vector_t& getInitialState() const { return initialState_; }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const LeggedRobotInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

private:
  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

  std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);
  matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

  std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose) const;
  std::unique_ptr<StateInputCost> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                            const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputConstraint> getZeroForceConstraint(size_t contactPointIndex);
  std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  size_t contactPointIndex, bool useAnalyticalGradients);
  std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                    size_t contactPointIndex, bool useAnalyticalGradients);
  std::unique_ptr<StateInputConstraint> getFootPlacementConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                   size_t contactPointIndex,
                                                                   std::vector<std::pair<vector3_t, vector3_t>> rebarFile);
  std::unique_ptr<StateInputCost> getFootPlacementConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                             size_t contactPointIndex,
                                                             const RelaxedBarrierPenalty::Config& barrierPenaltyConfig,
                                                             std::vector<std::pair<vector3_t, vector3_t>> rebarFile);
  std::unique_ptr<StateInputCost> getCollisionAvoidanceConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  size_t contactPointIndex, scalar_t minimumDistance,
                                                                  const RelaxedBarrierPenalty::Config& barrierPenaltyConfig,
                                                                  std::vector<std::pair<vector3_t, vector3_t>> rebarFile);

  ModelSettings modelSettings_;
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<LeggedRobotInitializer> initializerPtr_;

  vector_t initialState_;
};

} // namespace quadruped
} // namespace ocs2_interface

#endif //OCS2_INTERFACE_LEGGEDROBOTINTERFACE_H
