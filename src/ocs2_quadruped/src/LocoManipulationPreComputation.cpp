#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_quadruped/LocoManipulationPreComputation.h"

namespace ocs2 {
namespace quadruped {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LocoManipulationPreComputation::LocoManipulationPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info)
    : pinocchioInterface_(std::move(pinocchioInterface)), pinocchioMapping_(info) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LocoManipulationPreComputation* LocoManipulationPreComputation::clone() const {
  return new LocoManipulationPreComputation(pinocchioInterface_, pinocchioMapping_.getCentroidalModelInfo());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LocoManipulationPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

  if (request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateGlobalPlacements(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LocoManipulationPreComputation::requestFinal(RequestSet request, scalar_t t, const vector_t& x) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

  if (request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace quadruped
}  // namespace ocs2