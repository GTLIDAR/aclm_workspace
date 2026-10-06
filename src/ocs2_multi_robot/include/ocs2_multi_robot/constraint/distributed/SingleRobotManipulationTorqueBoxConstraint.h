#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Bounding box constraint for manipulation torques applied by a single robot.
 * Only active when ARM_CONTACT_DIM == 6.
 * Operates on the torque components (indices 3-5) of the arm segment in ALTERNATING_SINGLE_ROBOT_INPUT_DIM.
 */
class SingleRobotManipulationTorqueBoxConstraint final : public StateInputConstraint {
 public:
  SingleRobotManipulationTorqueBoxConstraint(size_t robotIndex, const vector3_t& lowerBound, const vector3_t& upperBound);
  ~SingleRobotManipulationTorqueBoxConstraint() override = default;
  SingleRobotManipulationTorqueBoxConstraint* clone() const override {
    return new SingleRobotManipulationTorqueBoxConstraint(*this);
  }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  SingleRobotManipulationTorqueBoxConstraint(const SingleRobotManipulationTorqueBoxConstraint& other) = default;

  vector3_t getTorqueInWorld(const vector_t& input) const;

  const size_t robotIndex_;
  const vector3_t lowerBound_;
  const vector3_t upperBound_;
};

}  // namespace multi_robot
}  // namespace ocs2

