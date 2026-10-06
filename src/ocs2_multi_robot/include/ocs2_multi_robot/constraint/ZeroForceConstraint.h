#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

class ZeroForceConstraint final : public StateInputConstraint {
public:
  /*
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   * @param [in] info : The centroidal model information.
   */
  ZeroForceConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager, size_t contactPointIndex, size_t index);

  ~ZeroForceConstraint() override = default;
  ZeroForceConstraint* clone() const override { return new ZeroForceConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return QUADRUPED_CONTACT_DIM; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

private:
  ZeroForceConstraint(const ZeroForceConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const size_t contactPointIndex_;
  const size_t Index_;
};

}  // namespace multi_robot
}  // namespace ocs2
