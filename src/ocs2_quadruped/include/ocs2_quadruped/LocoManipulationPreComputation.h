#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

namespace ocs2 {
namespace quadruped {

/** Callback for caching and reference update */
class LocoManipulationPreComputation : public PreComputation {
 public:
  LocoManipulationPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info);

  ~LocoManipulationPreComputation() override = default;

  LocoManipulationPreComputation(const LocoManipulationPreComputation& rhs) = delete;
  LocoManipulationPreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;
  void requestFinal(RequestSet request, scalar_t t, const vector_t& x) override;

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  PinocchioInterface pinocchioInterface_;
  CentroidalModelPinocchioMapping pinocchioMapping_;
};

}  // namespace quadruped
}  // namespace ocs2