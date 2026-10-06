#pragma once

#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

class CargoInitializer final : public Initializer {
 public:
  CargoInitializer(scalar_t cargoMass, size_t numArms);
  ~CargoInitializer() override = default;
  CargoInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  CargoInitializer(const CargoInitializer& other) = default;

  scalar_t cargoMass_;
  size_t numArms_;
};

}  // namespace multi_robot
}  // namespace ocs2


