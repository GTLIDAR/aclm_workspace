#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Enforce the foot height is consistent with the terrain map during stance phase
 */
class FootPlacementConstraint final : public StateInputConstraint {
public:
  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] contactPointIndex : The 3 DoF contact index.
   */
  FootPlacementConstraint(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                           size_t contactPointIndex, size_t index, size_t stateOffset);

  ~FootPlacementConstraint() override = default;
  FootPlacementConstraint* clone() const override { return new FootPlacementConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 2; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

private:
  FootPlacementConstraint(const FootPlacementConstraint& rhs);

  SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  std::shared_ptr<HeightMap> heightMapPtr_;
  const size_t contactPointIndex_;
  const size_t Index_;
  const size_t robotStateOffset_;
};

}  // namespace multi_robot
}  // namespace ocs2
