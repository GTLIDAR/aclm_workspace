#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include <ocs2_quadruped/common/utils.h>
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

namespace ocs2 {
namespace multi_robot {

/**
 * State-input tracking cost used for intermediate times
 */
class TwoQuadWCargoTrackingCost final : public QuadraticStateInputCost {
public:
  TwoQuadWCargoTrackingCost(matrix_t Q, matrix_t R, const scalar_t& robot_mass, const scalar_t& cargo_mass, 
                          const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                          const vector_t& robot1Offset, const vector_t& robot2Offset)
          : QuadraticStateInputCost(std::move(Q), std::move(R)), robot_mass_(robot_mass), cargo_mass_(cargo_mass), 
            referenceManagerPtr_(&referenceManager), robot1Offset_(robot1Offset), robot2Offset_(robot2Offset) {}

  ~TwoQuadWCargoTrackingCost() override = default;
  TwoQuadWCargoTrackingCost* clone() const override { return new TwoQuadWCargoTrackingCost(*this); }

private:
  TwoQuadWCargoTrackingCost(const TwoQuadWCargoTrackingCost& rhs) = default;

  // Here derivation just means the difference between desired value and actual value, not 'dy/dx'
  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const quadruped::contact_flag_t contactFlagsSingleRobot = quadruped::contact_flag_t{contactFlags[0], contactFlags[1], contactFlags[2], contactFlags[3]};
    const auto numStanceLegs = quadruped::numberOfClosedContacts(contactFlagsSingleRobot);
    
    // Get cargo state from target trajectories (last 12 elements)
    const vector_t xNominalDefault = targetTrajectories.getDesiredState(time).head(STATE_DIM_TWO_QUAD_W_CARGO);
    const vector_t cargoState = targetTrajectories.getDesiredState(time).segment(48, 12);
    
    // Compute robot states using fixed transformation from initialStateOffset
    vector_t xNominal(STATE_DIM_TWO_QUAD_W_CARGO);
    xNominal.setZero();
    
    // Set cargo state (last 12 elements)
    xNominal.segment(48, 12) = cargoState;
    
    // Compute robot 1 state using robot1Offset (same logic as initialization)
    xNominal.segment(0, 3) = cargoState.segment(0, 3) + robot1Offset_.segment(0, 3);
    xNominal(2) = xNominalDefault(2);  // Fixed COM height
    xNominal.segment(3, 3) = xNominalDefault.segment(3, 3);  // COM velocity
    
    // Orientation composition (ZYX): R_robot1 = R_cargo * R_offset
    vector3_t cargo_euler = cargoState.segment(6, 3);
    vector3_t robot1_euler_offset = robot1Offset_.segment(3, 3);
    Eigen::AngleAxis<scalar_t> yaw_c1(cargo_euler(0), Eigen::Matrix<scalar_t,3,1>(0,0,1));
    Eigen::AngleAxis<scalar_t> pit_c1(cargo_euler(1), Eigen::Matrix<scalar_t,3,1>(0,1,0));
    Eigen::AngleAxis<scalar_t> rol_c1(cargo_euler(2), Eigen::Matrix<scalar_t,3,1>(1,0,0));
    Eigen::Quaternion<scalar_t> q_cargo_zyx = yaw_c1 * pit_c1 * rol_c1;
    Eigen::AngleAxis<scalar_t> yaw_o1(robot1_euler_offset(0), Eigen::Matrix<scalar_t,3,1>(0,0,1));
    Eigen::AngleAxis<scalar_t> pit_o1(robot1_euler_offset(1), Eigen::Matrix<scalar_t,3,1>(0,1,0));
    Eigen::AngleAxis<scalar_t> rol_o1(robot1_euler_offset(2), Eigen::Matrix<scalar_t,3,1>(1,0,0));
    Eigen::Quaternion<scalar_t> q_offset_zyx_1 = yaw_o1 * pit_o1 * rol_o1;
    Eigen::Quaternion<scalar_t> q_robot1 = q_cargo_zyx * q_offset_zyx_1;
    matrix3_t R_robot1 = q_robot1.toRotationMatrix();
    Eigen::Matrix<scalar_t,3,1> eulZyx_r1 = R_robot1.eulerAngles(2,1,0);
    vector3_t robot1_euler;
    robot1_euler << eulZyx_r1(0), eulZyx_r1(1), eulZyx_r1(2);
    xNominal.segment(6, 3) = robot1_euler;
    xNominal.segment(9, 3) = xNominalDefault.segment(9, 3);  // Angular momentum
    
    // Foot positions using composed orientation
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      const vector3_t p_in_default_frame = getNominalEePosWrtCom(ee);
      const vector3_t p_world_new = xNominal.segment(0, 3) + R_robot1 * p_in_default_frame;
      xNominal.segment(12 + ee * QUADRUPED_CONTACT_DIM, 3) = p_world_new;
    }
    
    // Compute robot 2 state using robot2Offset (same logic as initialization)
    xNominal.segment(24, 3) = cargoState.segment(0, 3) + robot2Offset_.segment(0, 3);
    xNominal(26) = xNominalDefault(26);  // Fixed COM height
    xNominal.segment(27, 3) = xNominalDefault.segment(27, 3);  // COM velocity
    
    // Orientation composition (ZYX): R_robot2 = R_cargo * R_offset
    vector3_t robot2_euler_offset = robot2Offset_.segment(3, 3);
    Eigen::AngleAxis<scalar_t> yaw_o2(robot2_euler_offset(0), Eigen::Matrix<scalar_t,3,1>(0,0,1));
    Eigen::AngleAxis<scalar_t> pit_o2(robot2_euler_offset(1), Eigen::Matrix<scalar_t,3,1>(0,1,0));
    Eigen::AngleAxis<scalar_t> rol_o2(robot2_euler_offset(2), Eigen::Matrix<scalar_t,3,1>(1,0,0));
    Eigen::Quaternion<scalar_t> q_offset_zyx_2 = yaw_o2 * pit_o2 * rol_o2;
    Eigen::Quaternion<scalar_t> q_robot2 = q_cargo_zyx * q_offset_zyx_2;
    matrix3_t R_robot2 = q_robot2.toRotationMatrix();
    Eigen::Matrix<scalar_t,3,1> eulZyx_r2 = R_robot2.eulerAngles(2,1,0);
    vector3_t robot2_euler;
    robot2_euler << eulZyx_r2(0), eulZyx_r2(1), eulZyx_r2(2);
    xNominal.segment(30, 3) = robot2_euler;
    xNominal.segment(33, 3) = xNominalDefault.segment(33, 3);  // Angular momentum
    
    // Foot positions using composed orientation
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      const vector3_t p_in_default_frame = getNominalEePosWrtCom(ee);
      const vector3_t p_world_new = xNominal.segment(24, 3) + R_robot2 * p_in_default_frame;
      xNominal.segment(36 + ee * QUADRUPED_CONTACT_DIM, 3) = p_world_new;
    }

    vector_t uNominal(input.rows());
    uNominal.setZero();
    vector3_t forceAverage(0.0, 0.0, (robot_mass_ + cargo_mass_ / 2.0) * 9.81 / numStanceLegs);
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      if (contactFlags[ee])
        uNominal.segment(ee*QUADRUPED_CONTACT_DIM, 3) = forceAverage;
    }
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      if (contactFlags[ee])
        uNominal.segment(ee*QUADRUPED_CONTACT_DIM+24, 3) = forceAverage;
    }
    return {state.head(STATE_DIM_TWO_QUAD_W_CARGO) - xNominal, input - uNominal};
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const scalar_t robot_mass_;
  const scalar_t cargo_mass_;
  const vector_t robot1Offset_;
  const vector_t robot2Offset_;
};

/**
 * State tracking cost used for the final time
 */
class TwoQuadWCargoStateTrackingCostAE final : public QuadraticStateCost {
public:
  TwoQuadWCargoStateTrackingCostAE(matrix_t Q, const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                   const vector_t& robot1Offset, const vector_t& robot2Offset)
          : QuadraticStateCost(std::move(Q)), referenceManagerPtr_(&referenceManager), 
            robot1Offset_(robot1Offset), robot2Offset_(robot2Offset) {}

  ~TwoQuadWCargoStateTrackingCostAE() override = default;
  TwoQuadWCargoStateTrackingCostAE* clone() const override { return new TwoQuadWCargoStateTrackingCostAE(*this); }

private:
  TwoQuadWCargoStateTrackingCostAE(const TwoQuadWCargoStateTrackingCostAE& rhs) = default;

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    
    // Get cargo state from target trajectories (last 12 elements)
    const vector_t cargoState = targetTrajectories.getDesiredState(time).segment(48, 12);
    
    // Compute robot states using fixed transformation from initialStateOffset
    vector_t xNominal(STATE_DIM_TWO_QUAD_W_CARGO);
    xNominal.setZero();
    
    // Set cargo state (last 12 elements)
    xNominal.segment(48, 12) = cargoState;
    
    // Compute robot 1 state using robot1Offset (same logic as initialization)
    xNominal.segment(0, 3) = cargoState.segment(0, 3) + robot1Offset_.segment(0, 3);
    xNominal(2) = 0.487695;  // Fixed COM height
    xNominal.segment(3, 3) = vector_t::Zero(3);  // COM velocity
    
    // Orientation composition (ZYX): R_robot1 = R_cargo * R_offset
    vector3_t cargo_euler = cargoState.segment(6, 3);
    vector3_t robot1_euler_offset = robot1Offset_.segment(3, 3);
    Eigen::AngleAxis<scalar_t> yaw_c1(cargo_euler(0), Eigen::Matrix<scalar_t,3,1>(0,0,1));
    Eigen::AngleAxis<scalar_t> pit_c1(cargo_euler(1), Eigen::Matrix<scalar_t,3,1>(0,1,0));
    Eigen::AngleAxis<scalar_t> rol_c1(cargo_euler(2), Eigen::Matrix<scalar_t,3,1>(1,0,0));
    Eigen::Quaternion<scalar_t> q_cargo_zyx = yaw_c1 * pit_c1 * rol_c1;
    Eigen::AngleAxis<scalar_t> yaw_o1(robot1_euler_offset(0), Eigen::Matrix<scalar_t,3,1>(0,0,1));
    Eigen::AngleAxis<scalar_t> pit_o1(robot1_euler_offset(1), Eigen::Matrix<scalar_t,3,1>(0,1,0));
    Eigen::AngleAxis<scalar_t> rol_o1(robot1_euler_offset(2), Eigen::Matrix<scalar_t,3,1>(1,0,0));
    Eigen::Quaternion<scalar_t> q_offset_zyx_1 = yaw_o1 * pit_o1 * rol_o1;
    Eigen::Quaternion<scalar_t> q_robot1 = q_cargo_zyx * q_offset_zyx_1;
    matrix3_t R_robot1 = q_robot1.toRotationMatrix();
    Eigen::Matrix<scalar_t,3,1> eulZyx_r1 = R_robot1.eulerAngles(2,1,0);
    vector3_t robot1_euler;
    robot1_euler << eulZyx_r1(0), eulZyx_r1(1), eulZyx_r1(2);
    xNominal.segment(6, 3) = robot1_euler;
    xNominal.segment(9, 3) = vector_t::Zero(3);  // Angular momentum
    
    // Foot positions using composed orientation
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      const vector3_t p_in_default_frame = getNominalEePosWrtCom(ee);
      const vector3_t p_world_new = xNominal.segment(0, 3) + R_robot1 * p_in_default_frame;
      xNominal.segment(12 + ee * QUADRUPED_CONTACT_DIM, 3) = p_world_new;
    }
    
    // Compute robot 2 state using robot2Offset (same logic as initialization)
    xNominal.segment(24, 3) = cargoState.segment(0, 3) + robot2Offset_.segment(0, 3);
    xNominal(26) = 0.487695;  // Fixed COM height
    xNominal.segment(27, 3) = vector_t::Zero(3);  // COM velocity
    
    // Orientation composition (ZYX): R_robot2 = R_cargo * R_offset
    vector3_t robot2_euler_offset = robot2Offset_.segment(3, 3);
    Eigen::AngleAxis<scalar_t> yaw_o2(robot2_euler_offset(0), Eigen::Matrix<scalar_t,3,1>(0,0,1));
    Eigen::AngleAxis<scalar_t> pit_o2(robot2_euler_offset(1), Eigen::Matrix<scalar_t,3,1>(0,1,0));
    Eigen::AngleAxis<scalar_t> rol_o2(robot2_euler_offset(2), Eigen::Matrix<scalar_t,3,1>(1,0,0));
    Eigen::Quaternion<scalar_t> q_offset_zyx_2 = yaw_o2 * pit_o2 * rol_o2;
    Eigen::Quaternion<scalar_t> q_robot2 = q_cargo_zyx * q_offset_zyx_2;
    matrix3_t R_robot2 = q_robot2.toRotationMatrix();
    Eigen::Matrix<scalar_t,3,1> eulZyx_r2 = R_robot2.eulerAngles(2,1,0);
    vector3_t robot2_euler;
    robot2_euler << eulZyx_r2(0), eulZyx_r2(1), eulZyx_r2(2);
    xNominal.segment(30, 3) = robot2_euler;
    xNominal.segment(33, 3) = vector_t::Zero(3);  // Angular momentum
    
    // Foot positions using composed orientation
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      const vector3_t p_in_default_frame = getNominalEePosWrtCom(ee);
      const vector3_t p_world_new = xNominal.segment(24, 3) + R_robot2 * p_in_default_frame;
      xNominal.segment(36 + ee * QUADRUPED_CONTACT_DIM, 3) = p_world_new;
    }
    
    return state.head(STATE_DIM_TWO_QUAD_W_CARGO) - xNominal;
  }

  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  const vector_t robot1Offset_;
  const vector_t robot2Offset_;
};

}  // namespace multi_robot
}  // namespace ocs2