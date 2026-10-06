#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Bounding box constraint for manipulation torques applied on the cargo handles in centralized formulation.
 * Only active when ARM_CONTACT_DIM == 6.
 * Operates on the torque components in the cargo input segment.
 */
class ManipulationTorqueBoxConstraint final : public StateInputConstraint {
 public:
  ManipulationTorqueBoxConstraint(size_t robotIndex, const vector3_t& lowerBound, const vector3_t& upperBound, size_t numRobots);
  ~ManipulationTorqueBoxConstraint() override = default;
  ManipulationTorqueBoxConstraint* clone() const override {
    return new ManipulationTorqueBoxConstraint(*this);
  }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  ManipulationTorqueBoxConstraint(const ManipulationTorqueBoxConstraint& other) = default;

  vector3_t getTorqueInWorld(const vector_t& input) const;

  const size_t robotIndex_;
  const vector3_t lowerBound_;
  const vector3_t upperBound_;
  const size_t numRobots_;
};

}  // namespace multi_robot
}  // namespace ocs2

