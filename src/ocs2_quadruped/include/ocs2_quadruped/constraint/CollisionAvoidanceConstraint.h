#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "ocs2_quadruped/common/Types.h"
#include "ocs2_quadruped/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace quadruped {

class CollisionAvoidanceConstraint final : public StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionAvoidanceConstraint(
      const SwitchedModelReferenceManager& referenceManager,
      const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
      size_t contactPointIndex,
      scalar_t minimumDistance,
      std::vector<std::pair<vector3_t, vector3_t>> rebarSet);

  ~CollisionAvoidanceConstraint() override = default;
  CollisionAvoidanceConstraint* clone() const override {
    return new CollisionAvoidanceConstraint(*this);
  }

  bool isActive(scalar_t time) const override;

  size_t getNumConstraints(scalar_t time) const override { return 4; }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const PreComputation& preComp) const override;
  // VectorFunctionQuadraticApproximation getQuadraticApproximation(
  //     scalar_t time, const vector_t& state, const vector_t& input,
  //     const PreComputation& preComp) const override;

 private:
  CollisionAvoidanceConstraint(const CollisionAvoidanceConstraint& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t contactPointIndex_;

  const scalar_t minimumDistance_ = 0.03;
  std::vector<std::pair<vector3_t, vector3_t>> rebarSetEndPoints_;
  std::vector<std::pair<vector3_t, vector3_t>> rebarSet_;
  scalar_t x_max = -1e10;
  scalar_t y_max = -1e10;
  scalar_t x_min = 1e10;
  scalar_t y_min = 1e10;
};
}  // namespace quadruped
}  // namespace ocs2