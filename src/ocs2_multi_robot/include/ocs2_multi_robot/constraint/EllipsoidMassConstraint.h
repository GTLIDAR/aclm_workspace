#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Enforce the ellipsoid mass constraint
 * m = \frac{4}{3}*pi*e_x*e_y*e_z*\rho
 * \rho = (pow(3.0/4.0*mass_, 5.0/2.0) * pow(8.0/15.0*M_PI, 3.0/2.0)) 
 *         / pow(M_PI, 5.0/2.0)*pow(\sigma(0)*\sigma(1)*\sigma(2), 0.5)
 * where \sigma are the eigen values of CCRB MoI
 */
class EllipsoidMassConstraint final : public StateConstraint {
public:
  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] contactPointIndex : The 6 DoF contact index.
   */
  EllipsoidMassConstraint(SwitchedModelReferenceManagerWithTerrain& referenceManager,
                          const scalar_t& mass);

  ~EllipsoidMassConstraint() override = default;
  EllipsoidMassConstraint* clone() const override { return new EllipsoidMassConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

private:
  EllipsoidMassConstraint(const EllipsoidMassConstraint& rhs);

  SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  scalar_t mass_;
};

}  // namespace multi_robot
}  // namespace ocs2
