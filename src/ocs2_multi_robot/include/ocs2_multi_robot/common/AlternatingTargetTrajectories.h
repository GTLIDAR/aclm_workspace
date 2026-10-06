#pragma once

#include <ostream>

#include "ocs2_core/Types.h"
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {
namespace multi_robot {

/**
 * This class is an interface class for the user defined target trajectories.
 */
struct AlternatingTargetTrajectories {
  explicit AlternatingTargetTrajectories(size_t size = 0);
  AlternatingTargetTrajectories(scalar_array_t desiredTimeTrajectory, vector_array_t desiredStateTrajectory,
                                vector_array_t desiredInputTrajectory,
                                vector_array_t desiredDualVariableTrajectory = vector_array_t());
  AlternatingTargetTrajectories(const TargetTrajectories& targetTrajectories,
                                size_t fullDualVariableDim);
  void clear();
  bool empty() const { return timeTrajectory.empty() || stateTrajectory.empty(); }
  size_t size() const { return timeTrajectory.size(); }
  void resetDualVariableTrajectory();

  bool operator==(const AlternatingTargetTrajectories& other);
  bool operator!=(const AlternatingTargetTrajectories& other) { return !(*this == other); }

  vector_t getDesiredState(scalar_t time) const;
  vector_t getDesiredInput(scalar_t time) const;
  vector_t getDesiredDualVariable(scalar_t time) const;

  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
  vector_array_t dualVariableTrajectory;
};

void swap(AlternatingTargetTrajectories& lh, AlternatingTargetTrajectories& rh);
std::ostream& operator<<(std::ostream& out, const AlternatingTargetTrajectories& targetTrajectories);

}  // namespace multi_robot
}  // namespace ocs2
