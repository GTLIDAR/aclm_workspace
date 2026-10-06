#include "ocs2_multi_robot/common/AlternatingTargetTrajectories.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
AlternatingTargetTrajectories::AlternatingTargetTrajectories(size_t size)
    : timeTrajectory(size), stateTrajectory(size), inputTrajectory(size), dualVariableTrajectory(size) {}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
AlternatingTargetTrajectories::AlternatingTargetTrajectories(scalar_array_t desiredTimeTrajectory,
                                                           vector_array_t desiredStateTrajectory,
                                                           vector_array_t desiredInputTrajectory,
                                                           vector_array_t desiredDualVariableTrajectory)
    : timeTrajectory(std::move(desiredTimeTrajectory)),
      stateTrajectory(std::move(desiredStateTrajectory)),
      inputTrajectory(std::move(desiredInputTrajectory)),
      dualVariableTrajectory(std::move(desiredDualVariableTrajectory)) {
  assert(stateTrajectory.size() == timeTrajectory.size());
  if (!inputTrajectory.empty()) {
    assert(inputTrajectory.size() == timeTrajectory.size());
  }

  if (dualVariableTrajectory.size() != inputTrajectory.size()) {
    throw std::runtime_error("[AlternatingTargetTrajectories] dualVariableTrajectory size mismatch with inputTrajectory size!");
  }
}

AlternatingTargetTrajectories::AlternatingTargetTrajectories(const TargetTrajectories& targetTrajectories,
                                                             size_t fullDualVariableDim)
    : timeTrajectory(targetTrajectories.timeTrajectory),
      stateTrajectory(targetTrajectories.stateTrajectory),
      inputTrajectory(targetTrajectories.inputTrajectory) {
  assert(stateTrajectory.size() == timeTrajectory.size());
  if (!inputTrajectory.empty()) {
    assert(inputTrajectory.size() == timeTrajectory.size());
  }
  dualVariableTrajectory = vector_array_t(timeTrajectory.size(), vector_t::Zero(fullDualVariableDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void AlternatingTargetTrajectories::clear() {
  timeTrajectory.clear();
  stateTrajectory.clear();
  inputTrajectory.clear();
  dualVariableTrajectory.clear();
}

void AlternatingTargetTrajectories::resetDualVariableTrajectory() {
  dualVariableTrajectory = vector_array_t(timeTrajectory.size(), vector_t::Zero(dualVariableTrajectory.size()));
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
bool AlternatingTargetTrajectories::operator==(const AlternatingTargetTrajectories& other) {
  return this->timeTrajectory == other.timeTrajectory && this->stateTrajectory == other.stateTrajectory &&
         this->inputTrajectory == other.inputTrajectory && this->dualVariableTrajectory == other.dualVariableTrajectory;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t AlternatingTargetTrajectories::getDesiredState(scalar_t time) const {
  if (this->empty()) {
    throw std::runtime_error("[AlternatingTargetTrajectories] AlternatingTargetTrajectories is empty!");
  } else {
    return LinearInterpolation::interpolate(time, timeTrajectory, stateTrajectory);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t AlternatingTargetTrajectories::getDesiredInput(scalar_t time) const {
  if (this->empty()) {
    throw std::runtime_error("[AlternatingTargetTrajectories] AlternatingTargetTrajectories is empty!");
  } else if (inputTrajectory.empty()) {
    throw std::runtime_error("[AlternatingTargetTrajectories] AlternatingTargetTrajectories does not have inputTrajectory!");
  } else {
    return LinearInterpolation::interpolate(time, timeTrajectory, inputTrajectory);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t AlternatingTargetTrajectories::getDesiredDualVariable(scalar_t time) const {
  if (this->empty()) {
    throw std::runtime_error("[AlternatingTargetTrajectories] AlternatingTargetTrajectories is empty!");
  } else if (dualVariableTrajectory.empty()) {
    throw std::runtime_error(
        "[AlternatingTargetTrajectories] AlternatingTargetTrajectories does not have dualVariableTrajectory!");
  } else {
    return LinearInterpolation::interpolate(time, timeTrajectory, dualVariableTrajectory);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void swap(AlternatingTargetTrajectories& lh, AlternatingTargetTrajectories& rh) {
  lh.timeTrajectory.swap(rh.timeTrajectory);
  lh.stateTrajectory.swap(rh.stateTrajectory);
  lh.inputTrajectory.swap(rh.inputTrajectory);
  lh.dualVariableTrajectory.swap(rh.dualVariableTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
std::ostream& operator<<(std::ostream& out, const AlternatingTargetTrajectories& targetTrajectories) {
  for (size_t i = 0; i < targetTrajectories.size(); i++) {
    out << "time: " << targetTrajectories.timeTrajectory[i] << "\n";
    out << "state: [" << toDelimitedString(targetTrajectories.stateTrajectory[i]) << "]\n";
    out << "input: [" << toDelimitedString(targetTrajectories.inputTrajectory[i]) << "]\n";
    out << "dual variable: [" << toDelimitedString(targetTrajectories.dualVariableTrajectory[i]) << "]\n";
  }  // end of i loop

  return out;
}

}  // namespace multi_robot
}  // namespace ocs2
