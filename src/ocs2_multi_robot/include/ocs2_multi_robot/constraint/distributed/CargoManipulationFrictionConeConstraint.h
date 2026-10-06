#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_multi_robot/constraint/FrictionConeConstraint.h"
#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Soft friction cone constraint for manipulation forces applied on the cargo handles.
 * Operates on the first three entries (force) of each ARM_CONTACT_DIM block and ignores torque components.
 */
class CargoManipulationFrictionConeConstraint final : public StateInputConstraint {
 public:
  CargoManipulationFrictionConeConstraint(FrictionConeConstraint::Config config, size_t handleIndex,
                                          const vector_t& handlePoseInCargoFrame);
  ~CargoManipulationFrictionConeConstraint() override = default;
  CargoManipulationFrictionConeConstraint* clone() const override { return new CargoManipulationFrictionConeConstraint(*this); }

  bool isActive(scalar_t time) const override { return true; }
  size_t getNumConstraints(scalar_t time) const override { return 1; }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

 private:
  struct LocalForceDerivatives {
    matrix3_t dF_du;
  };

  struct ConeLocalDerivatives {
    vector3_t dCone_dF;
    matrix3_t d2Cone_dF2;
  };

  struct ConeDerivatives {
    vector3_t dCone_du;
    matrix3_t d2Cone_du2;
  };

  CargoManipulationFrictionConeConstraint(const CargoManipulationFrictionConeConstraint& other) = default;

  vector3_t getForceInWorld(const vector_t& input) const;
  matrix3_t computeHandleWorldRotation(const vector_t& state) const;
  vector_t coneConstraint(const vector3_t& localForces) const;
  LocalForceDerivatives computeLocalForceDerivatives(const matrix3_t& h_R_w) const;
  ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t& localForces) const;
  ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives,
                                                   const LocalForceDerivatives& localForceDerivatives) const;

  matrix_t frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeSecondDerivativeState(size_t stateDim) const;

  const FrictionConeConstraint::Config config_;
  const size_t handleIndex_;
  matrix3_t cargo_R_handle_;
};

}  // namespace multi_robot
}  // namespace ocs2


