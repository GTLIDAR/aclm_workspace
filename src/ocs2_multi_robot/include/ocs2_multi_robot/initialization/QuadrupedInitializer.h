#pragma once

#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

class QuadrupedInitializer final : public Initializer {
public:
  /*
   * Constructor
   * @param [in] mass : robot mass.
   * @param [in] referenceManager : Switched system reference manager.
   */
  QuadrupedInitializer(const scalar_t& mass, const SwitchedModelReferenceManagerWithTerrain& referenceManager);

  ~QuadrupedInitializer() override = default;
  QuadrupedInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

private:
  QuadrupedInitializer(const QuadrupedInitializer& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t mass_;
};

}  // namespace multi_robot
}  // namespace ocs2
