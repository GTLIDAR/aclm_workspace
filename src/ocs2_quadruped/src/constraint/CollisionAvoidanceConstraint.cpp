#include "ocs2_quadruped/constraint/CollisionAvoidanceConstraint.h"
#include "ocs2_quadruped/LeggedRobotPreComputation.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2 {
namespace quadruped {

CollisionAvoidanceConstraint::CollisionAvoidanceConstraint(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    size_t contactPointIndex,
    scalar_t minimumDistance,
    std::vector<std::pair<vector3_t, vector3_t>> rebarSet)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      minimumDistance_(minimumDistance),
      rebarSetEndPoints_(rebarSet) {
  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error(
        "[CollisionAvoidanceConstraint] this class only accepts a single "
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

bool CollisionAvoidanceConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];  // only consider one cycle
}

CollisionAvoidanceConstraint::CollisionAvoidanceConstraint(
    const CollisionAvoidanceConstraint& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
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

vector_t CollisionAvoidanceConstraint::getValue(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
    // Step 0 define the rebar configuration (hardcoded here temporarily)
    const scalar_t rebar_radius = 0.0;
    const scalar_t rebar_radius_lower = 0.0;
    const scalar_t foot_radius = 0.0;

    // Compute and sort the distances to all the rebars
    // Get the closest four rebars
    // The CA needs: n1-n4, a1-a4, evaluated at the current time
    // scalar_t middle =
    // referenceManagerPtr_->getGaitSchedule()->getMiddleOfLastStance(time);

    // const TargetTrajectories& targetTrajectories =
    //     referenceManagerPtr_->getTargetTrajectories();
    // const vector_t stateReference = targetTrajectories.getDesiredState(middle);

    vector3_t pEE = endEffectorKinematicsPtr_->getPosition(state).front();
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

    vector_t d(getNumConstraints(time));
    for (size_t i = 0; i < getNumConstraints(time); i++) {
        vector3_t n = closest_four_rebar[i].first;
        vector3_t a = closest_four_rebar[i].second;
        d[i] = ((pEE - a).cross(n)).norm() / n.norm() - rebar_radius - foot_radius - minimumDistance_;
    }

    if (pEE[0] > x_max-0.05 || pEE[1] > y_max-0.05 || pEE[0] < x_min+0.05 || pEE[1] < y_min+0.05){
        d += Eigen::Vector4d::Ones() * 100; // large number to disable the constraint
    }

    // vector3_t pEE = endEffectorKinematicsPtr_->getPosition(state).front();
    // const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
    // vector_t d(getNumConstraints(time));
    // auto closest_rebar_set = preCompLegged.getClosestRebarSet()[contactPointIndex_];
    // for (size_t i = 0; i < getNumConstraints(time); i++) {
    //     vector3_t n = closest_rebar_set[i].first;
    //     vector3_t a = closest_rebar_set[i].second;
    //     d[i] = ((pEE - a).cross(n)).norm() / n.norm() - rebar_radius - foot_radius - minimumDistance_;
    // }

    return d;
}

VectorFunctionLinearApproximation
CollisionAvoidanceConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
    // Step 0 define the rebar configuration (hardcoded here temporarily)
    const scalar_t rebar_radius = 0.0;
    const scalar_t rebar_radius_lower = 0.0;
    const scalar_t foot_radius = 0.0;

    vector3_t pEE = endEffectorKinematicsPtr_->getPosition(state).front();
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
    
    VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
    
    const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

    vector_t d(getNumConstraints(time));
    for (size_t i = 0; i < getNumConstraints(time); i++) {
        vector3_t n = closest_four_rebar[i].first;
        vector3_t a = closest_four_rebar[i].second;
        d[i] = ((pEE - a).cross(n)).norm() / n.norm() - rebar_radius - foot_radius - minimumDistance_;
        linearApproximation.dfdx.middleRows(i, 1).noalias() += 1.0/n.norm() * ((pEE - a).cross(n)).normalized().transpose() 
                                        * (-skewSymmetricMatrix(n) * positionApprox.dfdx);
        
    }
    if (pEE[0] > x_max-0.05 || pEE[1] > y_max-0.05 || pEE[0] < x_min+0.05 || pEE[1] < y_min+0.05){
      d += Eigen::Vector4d::Ones() * 100; // large number to disable the constraint
    }

    linearApproximation.f.noalias() = d;

    // VectorFunctionLinearApproximation linearApproximation =
    //   VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
    
    // const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

    // vector3_t pEE = endEffectorKinematicsPtr_->getPosition(state).front();
    // const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
    // vector_t d(getNumConstraints(time));
    // auto closest_rebar_set = preCompLegged.getClosestRebarSet()[contactPointIndex_];
    // for (size_t i = 0; i < getNumConstraints(time); i++) {
    //     vector3_t n = closest_rebar_set[i].first;
    //     vector3_t a = closest_rebar_set[i].second;
    //     d[i] = ((pEE - a).cross(n)).norm() / n.norm() - rebar_radius - foot_radius - minimumDistance_;
    //     linearApproximation.dfdx.middleRows(i, 1).noalias() += 1.0/n.norm() * ((pEE - a).cross(n)).normalized().transpose() 
    //                                     * (-skewSymmetricMatrix(n) * positionApprox.dfdx);
        
    // }
    // linearApproximation.f.noalias() = d;
    
    return linearApproximation;
}

}  // namespace quadruped
}  // namespace ocs2