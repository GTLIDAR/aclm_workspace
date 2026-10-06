#include "ocs2_multi_robot/constraint/ManipulationTorqueBoxConstraint.h"

#include <stdexcept>

namespace ocs2 {
namespace multi_robot {

ManipulationTorqueBoxConstraint::ManipulationTorqueBoxConstraint(size_t robotIndex,
                                                                 const vector3_t& lowerBound,
                                                                 const vector3_t& upperBound,
                                                                 size_t numRobots)
    : StateInputConstraint(ConstraintOrder::Linear),
      robotIndex_(robotIndex),
      lowerBound_(lowerBound),
      upperBound_(upperBound),
      numRobots_(numRobots) {
  if (robotIndex_ >= numRobots_) {
    throw std::runtime_error("[ManipulationTorqueBoxConstraint] robotIndex must be < numRobots.");
  }
  // Verify bounds are valid
  if ((upperBound_.array() < lowerBound_.array()).any()) {
    throw std::runtime_error(
        "[ManipulationTorqueBoxConstraint] Upper bound must be >= lower bound for all components.");
  }
}

bool ManipulationTorqueBoxConstraint::isActive(scalar_t time) const {
  // Only active when ARM_CONTACT_DIM == 6 (includes torque)
  return (ARM_CONTACT_DIM == 6);
}

size_t ManipulationTorqueBoxConstraint::getNumConstraints(scalar_t time) const {
  if (isActive(time)) {
    return 6;  // 3 lower bounds + 3 upper bounds
  }
  return 0;
}

vector3_t ManipulationTorqueBoxConstraint::getTorqueInWorld(const vector_t& input) const {
  // In centralized formulation, cargo input is at the end
  const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
  const size_t torqueOffset = cargoInputOffset + robotIndex_ * 6 + 3;
  if (torqueOffset + 3 > input.size()) {
    throw std::runtime_error("[ManipulationTorqueBoxConstraint] Invalid arm torque indexing.");
  }
  // Extract torque components (indices 3-5 of the 6D contact)
  return input.segment(torqueOffset, 3);
}

vector_t ManipulationTorqueBoxConstraint::getValue(scalar_t time, const vector_t& state,
                                                                     const vector_t& input,
                                                                     const PreComputation& preComp) const {
  if (!isActive(time)) {
    return vector_t::Zero(0);
  }

  const vector3_t torque = getTorqueInWorld(input);

  // Constraint format: [torque - lowerBound, upperBound - torque] >= 0
  // This means: lowerBound <= torque <= upperBound
  vector_t constraint(6);
  constraint.segment(0, 3) = torque - lowerBound_;  // Lower bound: torque >= lowerBound
  constraint.segment(3, 3) = upperBound_ - torque;  // Upper bound: torque <= upperBound

  return constraint;
}

VectorFunctionLinearApproximation ManipulationTorqueBoxConstraint::getLinearApproximation(
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
  const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
  const size_t torqueOffset = cargoInputOffset + robotIndex_ * 6 + 3;
  linearApproximation.dfdu = matrix_t::Zero(6, input.size());
  // For lower bounds: d/d(torque)[torque - lowerBound] = I
  linearApproximation.dfdu.block<3, 3>(0, torqueOffset) = matrix3_t::Identity();
  // For upper bounds: d/d(torque)[upperBound - torque] = -I
  linearApproximation.dfdu.block<3, 3>(3, torqueOffset) = -matrix3_t::Identity();

  return linearApproximation;
}

}  // namespace multi_robot
}  // namespace ocs2

