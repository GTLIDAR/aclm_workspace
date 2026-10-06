#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Enforce a kinematics box for each EE
*/
class ArmKinematicsBoxConstraint final : public StateInputConstraint {
public:
  /*
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   */
  ArmKinematicsBoxConstraint(const SwitchedModelReferenceManagerWithTerrain& referenceManager, 
    const vector_t& r1_handle, const vector_t& r2_handle);

  ~ArmKinematicsBoxConstraint() override = default;
  ArmKinematicsBoxConstraint* clone() const override { return new ArmKinematicsBoxConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 12; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

private:
  ArmKinematicsBoxConstraint(const ArmKinematicsBoxConstraint& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  vector_t r1_handle_;
  vector_t r2_handle_;

};

}  // namespace multi_robot
}  // namespace ocs2
