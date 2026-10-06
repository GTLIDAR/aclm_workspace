#include "ocs2_multi_robot/constraint/distributed/CargoManipulationTorqueBoxConstraint.h"

#include <stdexcept>

namespace ocs2 {
namespace multi_robot {

CargoManipulationTorqueBoxConstraint::CargoManipulationTorqueBoxConstraint(size_t handleIndex,
                                                                           const vector3_t& lowerBound,
                                                                           const vector3_t& upperBound)
    : StateInputConstraint(ConstraintOrder::Linear),
      handleIndex_(handleIndex),
      lowerBound_(lowerBound),
      upperBound_(upperBound) {
  // Verify bounds are valid
  if ((upperBound_.array() < lowerBound_.array()).any()) {
    throw std::runtime_error("[CargoManipulationTorqueBoxConstraint] Upper bound must be >= lower bound for all components.");
  }
}

bool CargoManipulationTorqueBoxConstraint::isActive(scalar_t time) const {
  // Only active when ARM_CONTACT_DIM == 6 (includes torque)
  return (ARM_CONTACT_DIM == 6);
}

size_t CargoManipulationTorqueBoxConstraint::getNumConstraints(scalar_t time) const {
  if (isActive(time)) {
    return 6;  // 3 lower bounds + 3 upper bounds
  }
  return 0;
}

vector3_t CargoManipulationTorqueBoxConstraint::getTorqueInWorld(const vector_t& input) const {
  const size_t offset = handleIndex_ * ARM_CONTACT_DIM;
  if (offset + 6 > input.size()) {
    throw std::runtime_error("[CargoManipulationTorqueBoxConstraint] Invalid arm torque indexing.");
  }
  // Extract torque components (indices 3-5)
  return input.segment(offset + 3, 3);
}

vector_t CargoManipulationTorqueBoxConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                         const PreComputation& preComp) const {
  if (!isActive(time)) {
    return vector_t::Zero(0);
  }

  const vector3_t torque = getTorqueInWorld(input);
  
  // Constraint format: [torque - lowerBound, upperBound - torque] >= 0
  // This means: lowerBound <= torque <= upperBound
  vector_t constraint(6);
  constraint.segment(0, 3) = torque - lowerBound_;  // Lower bound: torque >= lowerBound
  constraint.segment(3, 3) = upperBound_ - torque;   // Upper bound: torque <= upperBound
  
  return constraint;
}

VectorFunctionLinearApproximation CargoManipulationTorqueBoxConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation;
  
  if (!isActive(time)) {
    linearApproximation.f = vector_t::Zero(0);
    linearApproximation.dfdx = matrix_t::Zero(0, state.size());
    linearApproximation.dfdu = matrix_t::Zero(0, input.size());
    return linearApproximation;
  }

  linearApproximation.f = getValue(time, state, input, preComp);
  linearApproximation.dfdx = matrix_t::Zero(6, state.size());
  
  // df/du: derivative w.r.t. input (torque components)
  const size_t offset = handleIndex_ * ARM_CONTACT_DIM;
  linearApproximation.dfdu = matrix_t::Zero(6, input.size());
  // For lower bounds: d/d(torque)[torque - lowerBound] = I
  linearApproximation.dfdu.block<3, 3>(0, offset + 3) = matrix3_t::Identity();
  // For upper bounds: d/d(torque)[upperBound - torque] = -I
  linearApproximation.dfdu.block<3, 3>(3, offset + 3) = -matrix3_t::Identity();
  
  return linearApproximation;
}

}  // namespace multi_robot
}  // namespace ocs2

