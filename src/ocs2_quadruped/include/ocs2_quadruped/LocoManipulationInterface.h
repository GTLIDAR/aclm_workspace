//
// Created by zzhou387 on 9/9/22.
// Refer to ocs2_legged_robot
//

#ifndef OCS2_INTERFACE_LOCOMANIPULATIONINTERFACE_H
#define OCS2_INTERFACE_LOCOMANIPULATIONINTERFACE_H
// ocs2
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>

#include "ocs2_quadruped/common/ModelSettings.h"
#include "ocs2_quadruped/initialization/LocoManipulationInitializer.h"
#include "ocs2_quadruped/LocoManipulationPreComputation.h"

namespace ocs2 {
namespace quadruped {

class LocoManipulationInterface : public RobotInterface {
public:
  LocoManipulationInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile);

  ~LocoManipulationInterface() override = default;
  
  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

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
  const std::string& getEEFrameName() const {return eeFrame_; }

  const LocoManipulationInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

private:
  std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);
  matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

  std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose) const;
  std::unique_ptr<StateInputCost> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                            const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  bool useAnalyticalGradients);
  std::unique_ptr<StateCost> getEndEffectorConstraint(const std::string& ee_name,
                                                      const std::string& taskFile,
                                                      const std::string& prefix,
                                                      bool useAnalyticalGradients);

  ModelSettings modelSettings_;
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;
  std::string eeFrame_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<LocoManipulationInitializer> initializerPtr_;

  vector_t initialState_;
};

} // namespace quadruped
} // namespace ocs2_interface

#endif //OCS2_INTERFACE_LOCOMANIPULATIONINTERFACE_H
