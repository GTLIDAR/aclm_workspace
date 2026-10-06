#include "ocs2_quadruped/constraint/FootPlacementConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_quadruped/LeggedRobotPreComputation.h"

namespace ocs2 {
namespace quadruped {

FootPlacementConstraint::FootPlacementConstraint(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    size_t contactPointIndex, std::vector<std::pair<vector3_t, vector3_t>> rebarSet)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 4)),
      rebarSetEndPoints_(rebarSet) {
  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error(
        "[FootPlacementConstraint] this class only accepts a single "
        "end-effector!");
  }
  for(auto rebar : rebarSet){
    auto n = rebar.first - rebar.second;
    auto a = rebar.first;
    rebarSet_.push_back(std::pair<vector3_t, vector3_t>(n, a));

    // Find the maximum and minimum values (ignore the orientation of rebars for now)
    x_max = rebar.first[0] > x_max ? rebar.first[0] : x_max;
    y_max = rebar.first[1] > y_max ? rebar.first[1] : y_max;
    x_max = rebar.second[0] > x_max ? rebar.second[0] : x_max;
    y_max = rebar.second[1] > y_max ? rebar.second[1] : y_max; 
    x_min = rebar.first[0] < x_min ? rebar.first[0] : x_min;
    y_min = rebar.first[1] < y_min ? rebar.first[1] : y_min;
    x_min = rebar.second[0] < x_min ? rebar.second[0] : x_min;
    y_min = rebar.second[1] < y_min ? rebar.second[1] : y_min; 
    
  }
}

bool FootPlacementConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];  // only consider one cycle
}

FootPlacementConstraint::FootPlacementConstraint(
    const FootPlacementConstraint& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      rebarSetEndPoints_(rhs.rebarSetEndPoints_) {
  for(auto rebar : rebarSetEndPoints_){
    auto n = rebar.first - rebar.second;
    auto a = rebar.first;
    rebarSet_.push_back(std::pair<vector3_t, vector3_t>(n, a));

    // Find the maximum and minimum values (ignore the orientation of rebars for now)
    x_max = rebar.first[0] > x_max ? rebar.first[0] : x_max;
    y_max = rebar.first[1] > y_max ? rebar.first[1] : y_max;
    x_max = rebar.second[0] > x_max ? rebar.second[0] : x_max;
    y_max = rebar.second[1] > y_max ? rebar.second[1] : y_max; 
    x_min = rebar.first[0] < x_min ? rebar.first[0] : x_min;
    y_min = rebar.first[1] < y_min ? rebar.first[1] : y_min;
    x_min = rebar.second[0] < x_min ? rebar.second[0] : x_min;
    y_min = rebar.second[1] < y_min ? rebar.second[1] : y_min; 
  }
}

vector_t FootPlacementConstraint::getValue(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  
  scalar_t middle =
      referenceManagerPtr_->getGaitSchedule()->getMiddleOfLastStance(time);

  const TargetTrajectories& targetTrajectories =
      referenceManagerPtr_->getTargetTrajectories();
  const vector_t stateReference = targetTrajectories.getDesiredState(middle);
  // std::cout << "ref traj for leg " << contactPointIndex_ << " is: " << std::endl << stateReference.transpose() << std::endl << std::endl;

  vector3_t pEE = endEffectorKinematicsPtr_->getPosition(stateReference).front();
  std::vector<scalar_t> distance_set;
  std::unordered_map<scalar_t, std::pair<vector3_t, vector3_t>> rebar_map;
  for (auto rebar : rebarSet_) {
    auto n = rebar.first;
    auto a = rebar.second;
    scalar_t distance = ((pEE - a).cross(n)).norm() / n.norm(); 
    rebar_map[distance] = rebar;
    distance_set.push_back(distance);
  }

  std::sort(distance_set.begin(), distance_set.end());
  std::vector<std::pair<vector3_t, vector3_t>> closest_four_rebar(4);
  closest_four_rebar[0] = rebar_map.find(distance_set[0])->second;
  closest_four_rebar[1] = rebar_map.find(distance_set[1])->second;
  closest_four_rebar[2] = rebar_map.find(distance_set[2])->second;
  closest_four_rebar[3] = rebar_map.find(distance_set[3])->second; 

  // Step 3: compute the shrinked convex polygon
  // The Footplacement needs: Ax, b, evaluated at the middle of last stance time
  // s is a slack variable
  scalar_t swingTimeLeft =
      referenceManagerPtr_->getGaitSchedule()->getSwingTimeLeft();

  EndEffectorLinearConstraint::Config config;
  config.Ax.setZero(4, 3);
  config.Av.setZero(4, 3);
  config.b.setZero(4);
  for (size_t j = 0; j < closest_four_rebar.size(); j++){
    auto rebar = closest_four_rebar[j];
    // write straight line as d = ax + by + c, points on the line have d value of zero
    scalar_t a, b, c; 
    if (rebar.first[0] == 0){
      a = 1;
      b = 0;
      c = -rebar.second[0];
    }
    else {
      a = rebar.first[1] / rebar.first[0];
      b = -1;
      c = - a * rebar.second[0] + rebar.second[1];
    }
    scalar_t d_ee = a * pEE[0] + b * pEE[1] + c;

    if (d_ee >= 0) { // d = 0 is a lower bound for points within quadrilateral
      config.Ax(j, 0) = a;
      config.Ax(j, 1) = b;
      config.Ax(j, 2) = 0;
      config.b(j) = c - 0.05; // safety buffer 5cm
    } else if (d_ee < 0) { // d = 0 is an upper bound for points within quadrilateral
      config.Ax(j, 0) = -a;
      config.Ax(j, 1) = -b;
      config.Ax(j, 2) = 0;
      config.b(j) = -c - 0.05;
    }
    config.b(j) += swingTimeLeft;
  }

  if (pEE[0] > x_max || pEE[1] > y_max || pEE[0] < x_min || pEE[1] < y_min){
    // std::cout << "debug here" << std::endl;
    config.b += Eigen::Vector4d::Ones() * 100; // large number to disable the constraint
  }

  eeLinearConstraintPtr_->configure(config);
  
  // const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  // eeLinearConstraintPtr_->configure(preCompLegged.getFootPlacementConstraintConfigs()[contactPointIndex_]);

  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

VectorFunctionLinearApproximation
FootPlacementConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  scalar_t middle =
      referenceManagerPtr_->getGaitSchedule()->getMiddleOfLastStance(time);

  const TargetTrajectories& targetTrajectories =
      referenceManagerPtr_->getTargetTrajectories();
  const vector_t stateReference = targetTrajectories.getDesiredState(middle);

  vector3_t pEE = endEffectorKinematicsPtr_->getPosition(stateReference).front();
  std::vector<scalar_t> distance_set;
  std::unordered_map<scalar_t, std::pair<vector3_t, vector3_t>> rebar_map;
  for (auto rebar : rebarSet_) {
    auto n = rebar.first;
    auto a = rebar.second;
    scalar_t distance = ((pEE - a).cross(n)).norm() / n.norm(); 
    rebar_map[distance] = rebar;
    distance_set.push_back(distance);
  }

  std::sort(distance_set.begin(), distance_set.end());
  std::vector<std::pair<vector3_t, vector3_t>> closest_four_rebar(4);
  closest_four_rebar[0] = rebar_map.find(distance_set[0])->second;
  closest_four_rebar[1] = rebar_map.find(distance_set[1])->second;
  closest_four_rebar[2] = rebar_map.find(distance_set[2])->second;
  closest_four_rebar[3] = rebar_map.find(distance_set[3])->second; 

  // Step 3: compute the shrinked convex polygon
  // The Footplacement needs: Ax, b, evaluated at the middle of last stance time
  // s is a slack variable
  scalar_t swingTimeLeft =
      referenceManagerPtr_->getGaitSchedule()->getSwingTimeLeft();

  EndEffectorLinearConstraint::Config config;
  config.Ax.setZero(4, 3);
  config.Av.setZero(4, 3);
  config.b.setZero(4);
  for (size_t j = 0; j < closest_four_rebar.size(); j++){
    auto rebar = closest_four_rebar[j];
    // write straight line as d = ax + by + c, points on the line have d value of zero
    scalar_t a, b, c; 
    if (rebar.first[0] == 0){
      a = 1;
      b = 0;
      c = -rebar.second[0];
    }
    else {
      a = rebar.first[1] / rebar.first[0];
      b = -1;
      c = - a * rebar.second[0] + rebar.second[1];
    }
    scalar_t d_ee = a * pEE[0] + b * pEE[1] + c;

    if (d_ee >= 0) { // d = 0 is a lower bound for points within quadrilateral
      config.Ax(j, 0) = a;
      config.Ax(j, 1) = b;
      config.Ax(j, 2) = 0;
      config.b(j) = c - 0.05; // safety buffer 5cm
    } else if (d_ee < 0) { // d = 0 is an upper bound for points within quadrilateral
      config.Ax(j, 0) = -a;
      config.Ax(j, 1) = -b;
      config.Ax(j, 2) = 0;
      config.b(j) = -c - 0.05;
    }
    config.b(j) += swingTimeLeft;
  }

  if (pEE[0] > x_max || pEE[1] > y_max || pEE[0] < x_min || pEE[1] < y_min){
    config.b += Eigen::Vector4d::Ones() * 100; // large number to disable the constraint
  }

  eeLinearConstraintPtr_->configure(config);

  // const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  // auto config = preCompLegged.getFootPlacementConstraintConfigs()[contactPointIndex_];
  // config.b += Eigen::Vector4d::Ones() * swingTimeLeft;
  // eeLinearConstraintPtr_->configure(config);
  // eeLinearConstraintPtr_->configure(preCompLegged.getFootPlacementConstraintConfigs()[contactPointIndex_]);

  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

}  // namespace quadruped
}  // namespace ocs2