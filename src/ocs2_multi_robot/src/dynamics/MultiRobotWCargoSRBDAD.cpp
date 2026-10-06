#include <pinocchio/fwd.hpp>

#include "ocs2_multi_robot/dynamics/MultiRobotWCargoSRBDAD.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace multi_robot {

MultiRobotWCargoSRBDAD::MultiRobotWCargoSRBDAD(const scalar_t& robot_mass, const scalar_t& cargo_mass, 
  const matrix3_t& robot_inertia, const matrix3_t& cargo_inertia, 
  const std::vector<vector_t>& robotHandles, size_t numRobots,
  const std::string& libraryFolder, bool recompileLibraries) {
  robot_mass_ = robot_mass;
  cargo_mass_ = cargo_mass;
  robotHandles_ = robotHandles;
  numRobots_ = numRobots;
  
  // couldn't directly cast to ad_matrix for some reason
  robot_I_b_.setZero(3, 3);
  for (size_t i=0; i<3; i++) {
    for (size_t j=0; j<3; j++) {
      robot_I_b_(i, j) = static_cast<ad_scalar_t>(robot_inertia(i, j));
    }
  }
  cargo_I_b_.setZero(3, 3);
  for (size_t i=0; i<3; i++) {
    for (size_t j=0; j<3; j++) {
      cargo_I_b_(i, j) = static_cast<ad_scalar_t>(cargo_inertia(i, j));
    }
  }
  
  // Compute state and input dimensions dynamically
  const size_t stateDim = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  const size_t inputDim = numRobots_ * SINGLE_ROBOT_INPUT_DIM + numRobots_ * ARM_CONTACT_DIM;
  
  this->initialize(stateDim, inputDim, "multi_robot_" + std::to_string(numRobots_) + "_cargo_srbd_dynamics", libraryFolder,
                   recompileLibraries, true);
}

ad_vector_t MultiRobotWCargoSRBDAD::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                 const ad_vector_t& parameters) const {
  const size_t stateDim = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  ad_vector_t stateDerivative(stateDim);
  
  // Extract cargo state (at the end)
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  const ad_vector_t rc = state.segment(cargoStateOffset, 3);
  const ad_vector_t drc = state.segment(cargoStateOffset + 3, 3);
  const ad_vector_t thetac = state.segment(cargoStateOffset + 6, 3);
  const ad_vector_t lc = state.segment(cargoStateOffset + 9, 3);
  
  ad_matrix_t w_R_cargo = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(thetac);
  
  // Process each robot
  std::vector<Eigen::Matrix<ad_scalar_t, 3, 1>> robot_handle_positions_world(numRobots_);
  std::vector<ad_matrix_t> robot_handle_rotations(numRobots_);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    // Extract 6D transformation: x, y, z, yaw, pitch, roll
    Eigen::Matrix<ad_scalar_t, 3, 1> handle_pos_cargo;
    handle_pos_cargo << static_cast<ad_scalar_t>(robotHandles_[robot](0)), 
                        static_cast<ad_scalar_t>(robotHandles_[robot](1)), 
                        static_cast<ad_scalar_t>(robotHandles_[robot](2));
    
    ad_vector_t handle_euler(3);
    handle_euler << static_cast<ad_scalar_t>(robotHandles_[robot](3)), 
                   static_cast<ad_scalar_t>(robotHandles_[robot](4)), 
                   static_cast<ad_scalar_t>(robotHandles_[robot](5));
    
    ad_matrix_t cargo_R_handle = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(handle_euler);
    robot_handle_rotations[robot] = w_R_cargo * cargo_R_handle;
    robot_handle_positions_world[robot] = rc + w_R_cargo * handle_pos_cargo;
  }
  
  // Process each robot's dynamics
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    const size_t robotInputOffset = robot * SINGLE_ROBOT_INPUT_DIM;
    const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
    
    const ad_vector_t r = state.segment(robotStateOffset, 3);
    const ad_vector_t dr = state.segment(robotStateOffset + 3, 3);
    const ad_vector_t theta = state.segment(robotStateOffset + 6, 3);
    const ad_vector_t l = state.segment(robotStateOffset + 9, 3);
    const ad_vector_t p_ee = state.segment(robotStateOffset + 12, 12);
    const ad_vector_t f = input.segment(robotInputOffset, 12);
    const ad_vector_t v_ee = input.segment(robotInputOffset + 12, 12);
    
    const ad_vector_t fr = input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM, 3);
    ad_vector_t taur(3);
    taur.setZero(3);
    // Optionally include the torque part of the 6D arm contact in the dynamics.
    if (USE_ARM_TORQUE_IN_DYNAMICS && ARM_CONTACT_DIM >= 6) {
      taur = input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM + 3, 3);
    }
    
    // m * \ddot r = \sum f_j + mg, linear acceleration
    stateDerivative.segment(robotStateOffset, 3) = dr;
    ad_vector_t f_sum;
    f_sum.setZero(3);
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      f_sum += f.segment(ee*3, 3);
    }
    f_sum -= fr;
    ad_vector_t mg(3);
    mg << static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(-robot_mass_*g_);
    stateDerivative.segment(robotStateOffset + 3, 3) = (static_cast<ad_scalar_t>(1.0/robot_mass_))*(f_sum + mg);
    
    // \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
    ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(theta);
    Eigen::Matrix<ad_scalar_t, 3, 3> I_w;
    I_w = w_R_b * robot_I_b_ * w_R_b.transpose();
    ad_vector_t theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(theta, I_w.inverse() * l);
    stateDerivative.segment(robotStateOffset + 6, 3) = theta_dot;
    
    // \dot l = \sum (p_ee - r) \cross f
    ad_vector_t tau_sum;
    tau_sum.setZero(3);
    
    Eigen::Matrix<ad_scalar_t, 3, 1> r_fixed = r;
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      Eigen::Matrix<ad_scalar_t, 3, 1> f_ee = f.segment(ee*3, 3);
      Eigen::Matrix<ad_scalar_t, 3, 1> ee_pos = p_ee.segment(ee*3, 3);
      tau_sum += f_ee.cross(r_fixed - ee_pos);
    }
    
    Eigen::Matrix<ad_scalar_t, 3, 1> handle_pos_world = robot_handle_positions_world[robot];
    Eigen::Matrix<ad_scalar_t, 3, 1> fr_vec = fr;
    Eigen::Matrix<ad_scalar_t, 3, 1> handle_diff = r_fixed - handle_pos_world;
    tau_sum += -fr_vec.cross(handle_diff);
    if (USE_ARM_TORQUE_IN_DYNAMICS && ARM_CONTACT_DIM >= 6) {
      Eigen::Matrix<ad_scalar_t, 3, 1> taur_vec = taur;
      tau_sum -= taur_vec;
    }
    
    stateDerivative.segment(robotStateOffset + 9, 3) = tau_sum;
    stateDerivative.segment(robotStateOffset + 12, 12) = v_ee;
  }
  
  /***************************************** Cargo ****************************************/
  ad_vector_t mg_cargo(3);
  mg_cargo << static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(-cargo_mass_*g_);
  
  stateDerivative.segment(cargoStateOffset, 3) = drc;
  ad_vector_t f_sum;
  f_sum.setZero(3);
  
  const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    f_sum += input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM, 3);
  }
  
  stateDerivative.segment(cargoStateOffset + 3, 3) = (static_cast<ad_scalar_t>(1.0/cargo_mass_))*(f_sum + mg_cargo);
  
  ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(thetac);
  Eigen::Matrix<ad_scalar_t, 3, 3> I_w;
  I_w = w_R_b * cargo_I_b_ * w_R_b.transpose();
  ad_vector_t theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(thetac, I_w.inverse() * lc);
  stateDerivative.segment(cargoStateOffset + 6, 3) = theta_dot;
  
  ad_vector_t cargo_tau_sum;
  cargo_tau_sum.setZero(3);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    Eigen::Matrix<ad_scalar_t, 3, 1> handle_pos_cargo;
    handle_pos_cargo << static_cast<ad_scalar_t>(robotHandles_[robot](0)),
                        static_cast<ad_scalar_t>(robotHandles_[robot](1)),
                        static_cast<ad_scalar_t>(robotHandles_[robot](2));
    Eigen::Matrix<ad_scalar_t, 3, 1> handle_rel_world = w_R_cargo * handle_pos_cargo;
    const ad_vector_t fr = input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM, 3);
    Eigen::Matrix<ad_scalar_t, 3, 1> fr_vec = fr;
    cargo_tau_sum += handle_rel_world.cross(fr_vec);
    if (USE_ARM_TORQUE_IN_DYNAMICS && ARM_CONTACT_DIM >= 6) {
      const ad_vector_t taur = input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM + 3, 3);
      Eigen::Matrix<ad_scalar_t, 3, 1> taur_vec = taur;
      cargo_tau_sum += taur_vec;
    }
  }
  stateDerivative.segment(cargoStateOffset + 9, 3) = cargo_tau_sum;
  
  return stateDerivative;
}

} // namespace multi_robot
} // namespace ocs2

