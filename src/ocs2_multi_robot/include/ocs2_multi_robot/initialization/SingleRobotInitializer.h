#pragma once

#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

class SingleRobotInitializer final : public Initializer {
 public:
  SingleRobotInitializer(scalar_t robotMass, size_t numFeet, scalar_t cargoMass, size_t robotId,
                         size_t numRobots, const SwitchedModelReferenceManagerWithTerrain& referenceManager);
  ~SingleRobotInitializer() override = default;
  SingleRobotInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  SingleRobotInitializer(const SingleRobotInitializer& other) = default;

  scalar_t robotMass_;
  size_t numFeet_;
  scalar_t cargoMass_;
  size_t robotId_;
  size_t numRobots_;
  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
};

}  // namespace multi_robot
}  // namespace ocs2


