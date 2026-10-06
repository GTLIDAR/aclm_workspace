/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManagerWithTerrain::SwitchedModelReferenceManagerWithTerrain(std::shared_ptr<quadruped::GaitSchedule> gaitSchedulePtr,
                                          std::shared_ptr<quadruped::SwingTrajectoryPlanner> swingTrajectoryPtr,
                                          std::shared_ptr<HeightMap> heightMapPtr, size_t numRobots)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      heightMapPtr_(std::move(heightMapPtr)),
      numRobots_(numRobots),
      currentTime_(0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManagerWithTerrain::getContactFlags(scalar_t time) const {
  auto srcf = quadruped::modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
  contact_flag_t contactFlags(numRobots_ * QUADRUPED_FOOT_NUM);
  
  // Fill contact flags for all robots (each robot uses the same gait pattern)
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    for (size_t foot = 0; foot < QUADRUPED_FOOT_NUM; ++foot) {
      const size_t contactIndex = robot * QUADRUPED_FOOT_NUM + foot;
      contactFlags[contactIndex] = srcf[foot];
    }
  }
  
  return contactFlags;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManagerWithTerrain::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  setCurrentTime(initTime);
  
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  // Use height map to get terrain height at robot position for swing trajectory
  // This ensures swing trajectories adapt to terrain height even in non-perceptive mode
  scalar_t terrainHeight = 0.0;
  if (heightMapPtr_ && initState.size() >= 3) {
    // Use first robot's base position (indices 0, 1) to query terrain height
    terrainHeight = heightMapPtr_->GetHeight(initState(0), initState(1));
  }
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

}  // namespace quadruped
}  // namespace ocs2
