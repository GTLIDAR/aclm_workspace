#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Enforce a kinematics box for each EE
*/
class KinematicsBoxConstraint final : public StateInputConstraint {
public:
  /*
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   */
  KinematicsBoxConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager, size_t contactPointIndex, size_t index, size_t stateOffset);

  ~KinematicsBoxConstraint() override = default;
  KinematicsBoxConstraint* clone() const override { return new KinematicsBoxConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 6; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

private:
  KinematicsBoxConstraint(const KinematicsBoxConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const size_t contactPointIndex_;
  const size_t Index_;
  const size_t robotIDstateOffset_;

};

}  // namespace multi_robot
}  // namespace ocs2
