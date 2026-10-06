#pragma once

#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

class TwoQuadWCargoInitializer final : public Initializer {
public:
  /*
   * Constructor
   * @param [in] mass : robot mass.
   * @param [in] referenceManager : Switched system reference manager.
   */
  TwoQuadWCargoInitializer(const scalar_t& robot_mass, const scalar_t& cargo_mass, const SwitchedModelReferenceManagerWithTerrain& referenceManager);

  ~TwoQuadWCargoInitializer() override = default;
  TwoQuadWCargoInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

private:
  TwoQuadWCargoInitializer(const TwoQuadWCargoInitializer& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t robot_mass_;
  const scalar_t cargo_mass_;
};

}  // namespace multi_robot
}  // namespace ocs2
