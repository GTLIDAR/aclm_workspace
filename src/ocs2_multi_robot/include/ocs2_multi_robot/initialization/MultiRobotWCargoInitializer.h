#pragma once

#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

class MultiRobotWCargoInitializer final : public Initializer {
public:
  /*
   * Constructor
   * @param [in] robot_mass : robot mass.
   * @param [in] cargo_mass : cargo mass.
   * @param [in] numRobots : number of robots.
   * @param [in] referenceManager : Switched system reference manager.
   */
  MultiRobotWCargoInitializer(const scalar_t& robot_mass, const scalar_t& cargo_mass, 
                             size_t numRobots, const SwitchedModelReferenceManagerWithTerrain& referenceManager);

  ~MultiRobotWCargoInitializer() override = default;
  MultiRobotWCargoInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

private:
  MultiRobotWCargoInitializer(const MultiRobotWCargoInitializer& other) = default;

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t robot_mass_;
  const scalar_t cargo_mass_;
  const size_t numRobots_;
};

}  // namespace multi_robot
}  // namespace ocs2

