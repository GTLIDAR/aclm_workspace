#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Specializes the CppAd version of normal velocity constraint on an end-effector position and linear velocity.
 * Constructs the member EndEffectorLinearConstraint object with number of constraints of 1.
 *
 * This constraint enforces foot z-velocity to follow the swing trajectory planner during swing phase.
 * For multi-robot systems, each robot should have its own swing trajectory planner to account for
 * different terrain heights.
 *
 * See also EndEffectorLinearConstraint for the underlying computation.
 */
class NormalVelocityConstraint final : public StateInputConstraint {
public:
  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] swingTrajectoryPlannerPtr : The swing trajectory planner for this robot
   * @param [in] contactPointIndex : The global contact point index (across all robots)
   * @param [in] index : Local foot index within this robot (0-3 for quadrupeds)
   * @param [in] robotStateOffset : State offset for this robot in the multi-robot state vector
   * @param [in] positionErrorGain : Gain for position error feedback (default 0.0 means velocity-only)
   */
  NormalVelocityConstraint(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                           std::shared_ptr<quadruped::SwingTrajectoryPlanner> swingTrajectoryPlannerPtr,
                           size_t contactPointIndex, size_t index,
                           size_t robotStateOffset = 0, scalar_t positionErrorGain = 0.0);

  ~NormalVelocityConstraint() override = default;
  NormalVelocityConstraint* clone() const override { return new NormalVelocityConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

private:
  NormalVelocityConstraint(const NormalVelocityConstraint& rhs);

  SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const std::shared_ptr<quadruped::SwingTrajectoryPlanner> swingTrajectoryPlannerPtr_;
  const size_t contactPointIndex_;
  const size_t index_;  // Same indexing convention as ZeroVelocityConstraint / ZeroForceConstraint
  const size_t robotStateOffset_;  // State offset for this robot
  const scalar_t positionErrorGain_;  // Gain for position error feedback
};

}  // namespace multi_robot
}  // namespace ocs2
