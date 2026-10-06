#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Enfore zero end effector velocity during stance phase
*/
class ZeroVelocityConstraint : public StateInputConstraint {
public:
  ZeroVelocityConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager, size_t contactPointIndex, size_t index);
  ~ZeroVelocityConstraint() override = default;

  ZeroVelocityConstraint* clone() const override { return new ZeroVelocityConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return QUADRUPED_CONTACT_DIM; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

private:
  ZeroVelocityConstraint(const ZeroVelocityConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const size_t contactPointIndex_;
  const size_t Index_;
};
}
}