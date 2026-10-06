#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Bounding box constraint for manipulation torques applied on the cargo handles.
 * Only active when ARM_CONTACT_DIM == 6.
 * Operates on the torque components (indices 3-5) of each ARM_CONTACT_DIM block.
 */
class CargoManipulationTorqueBoxConstraint final : public StateInputConstraint {
 public:
  CargoManipulationTorqueBoxConstraint(size_t handleIndex, const vector3_t& lowerBound, const vector3_t& upperBound);
  ~CargoManipulationTorqueBoxConstraint() override = default;
  CargoManipulationTorqueBoxConstraint* clone() const override {
    return new CargoManipulationTorqueBoxConstraint(*this);
  }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  CargoManipulationTorqueBoxConstraint(const CargoManipulationTorqueBoxConstraint& other) = default;

  vector3_t getTorqueInWorld(const vector_t& input) const;

  const size_t handleIndex_;
  const vector3_t lowerBound_;
  const vector3_t upperBound_;
};

}  // namespace multi_robot
}  // namespace ocs2

