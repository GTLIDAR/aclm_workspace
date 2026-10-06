/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/Numerics.h>

#include <ocs2_quadruped/LeggedRobotPreComputation.h>

namespace ocs2 {
namespace quadruped {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(SwitchedModelReferenceManager& referenceManager,
                                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics, CentroidalModelInfo info,
                                                     ModelSettings settings,
                                                     std::vector<std::pair<vector3_t, vector3_t>> rebarSet)
    : referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(referenceManagerPtr_->getSwingTrajectoryPlanner()),
      settings_(std::move(settings)),
      rebarSetEndPoints_(rebarSet) {
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  closestRebarSet_.resize(info_.numThreeDofContacts);
  desiredClosestRebarSet_.resize(info_.numThreeDofContacts);
  footPlacementConConfigs_.resize(info_.numThreeDofContacts);
  // Step 1: Translate each pair into a linear expression (n, a)
  // n is a direction vector while a is a point on that line
  for(auto rebar : rebarSet){
    auto n = rebar.first - rebar.second;
    auto a = rebar.first;
    rebarSet_.push_back(std::pair<vector3_t, vector3_t>(n, a));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation* LeggedRobotPreComputation::clone() const {
  return new LeggedRobotPreComputation(*referenceManagerPtr_, *endEffectorKinematicsPtr_, info_, settings_, rebarSetEndPoints_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
      config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
    }
    return config;
  };

  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
  }

  // Step 2: compute and sort the distances to all the rebars
  // Get the closest four rebars
  // The CA needs: n1-n4, a1-a4, evaluated at the current time
  // for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
  //   vector3_t pEE = endEffectorKinematicsPtr_->getPosition(x)[i];
  //   std::vector<scalar_t> distance_set;
  //   std::unordered_map<scalar_t, std::pair<vector3_t, vector3_t>> rebar_map;
  //   for (auto rebar : rebarSet_) {
  //     auto n = rebar.first;
  //     auto a = rebar.second;
  //     scalar_t distance = ((pEE - a).cross(n)).norm() / n.norm(); 
  //     rebar_map[distance] = rebar;
  //     distance_set.push_back(distance);
  //   }

  //   std::sort(distance_set.begin(), distance_set.end());
  //   std::vector<std::pair<vector3_t, vector3_t>> closest_four_rebar(4);
  //   closest_four_rebar[0] = rebar_map.find(distance_set[0])->second;
  //   closest_four_rebar[1] = rebar_map.find(distance_set[1])->second;
  //   closest_four_rebar[2] = rebar_map.find(distance_set[2])->second;
  //   closest_four_rebar[3] = rebar_map.find(distance_set[3])->second; 
  //   closestRebarSet_[i] = closest_four_rebar;
  // }
    
  // // Repeat the above process but using the desired robot state at the middle of last stance time
  // scalar_t middle =
  //     referenceManagerPtr_->getGaitSchedule()->getMiddleOfLastStance(t);
  // const TargetTrajectories& targetTrajectories =
  //     referenceManagerPtr_->getTargetTrajectories();
  // const vector_t stateReference = targetTrajectories.getDesiredState(middle);

  // for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
  //   vector3_t pEE = endEffectorKinematicsPtr_->getPosition(stateReference)[i];
  //   std::vector<scalar_t> distance_set;
  //   std::unordered_map<scalar_t, std::pair<vector3_t, vector3_t>> rebar_map;
  //   for (auto rebar : rebarSet_) {
  //     auto n = rebar.first;
  //     auto a = rebar.second;
  //     scalar_t distance = ((pEE - a).cross(n)).norm() / n.norm(); 
  //     rebar_map[distance] = rebar;
  //     distance_set.push_back(distance);
  //   }

  //   std::sort(distance_set.begin(), distance_set.end());
  //   std::vector<std::pair<vector3_t, vector3_t>> closest_four_rebar(4);
  //   closest_four_rebar[0] = rebar_map.find(distance_set[0])->second;
  //   closest_four_rebar[1] = rebar_map.find(distance_set[1])->second;
  //   closest_four_rebar[2] = rebar_map.find(distance_set[2])->second;
  //   closest_four_rebar[3] = rebar_map.find(distance_set[3])->second; 
  //   desiredClosestRebarSet_[i] = closest_four_rebar;
  // }

  // // Step 3: compute the shrinked convex polygon
  // // The Footplacement needs: Ax, b, evaluated at the middle of last stance time
  // // s is a slack variable
  // scalar_t swingTimeLeft =
  //     referenceManagerPtr_->getGaitSchedule()->getSwingTimeLeft();

  // for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
  //   vector3_t pEE = endEffectorKinematicsPtr_->getPosition(stateReference)[i];
  //   EndEffectorLinearConstraint::Config config;
  //   config.Ax.setZero(4, 3);
  //   config.Av.setZero(4, 3);
  //   config.b.setZero(4);
  //   for (size_t j = 0; j < desiredClosestRebarSet_[i].size(); j++){
  //     auto rebar = desiredClosestRebarSet_[i][j];
  //     // write straight line as d = ax + by + c, points on the line have d value of zero
  //     scalar_t a, b, c; 
  //     if (rebar.first[0] == 0){
  //       a = 1;
  //       b = 0;
  //       c = -rebar.second[0];
  //     }
  //     else {
  //       a = rebar.first[1] / rebar.first[0];
  //       b = -1;
  //       c = - a * rebar.second[0] + rebar.second[1];
  //     }
  //     scalar_t d_ee = a * pEE[0] + b * pEE[1] + c;

  //     if (d_ee > 0) { // d = 0 is a lower bound for points within quadrilateral
  //       config.Ax(j, 0) = a;
  //       config.Ax(j, 1) = b;
  //       config.Ax(j, 2) = 0;
  //       config.b(j) = c; // safety buffer 5cm
  //     } else { // d = 0 is an upper bound for points within quadrilateral
  //       config.Ax(j, 0) = -a;
  //       config.Ax(j, 1) = -b;
  //       config.Ax(j, 2) = 0;
  //       config.b(j) = -c;
  //     }
  //     // config.b(j) += swingTimeLeft;
  //   }
  //   // std::cout << "debug Ax here" << std::endl << config.Ax << std::endl;
  //   footPlacementConConfigs_[i] = config;
  // }
  
}

}  // namespace quadruped
}  // namespace ocs2
