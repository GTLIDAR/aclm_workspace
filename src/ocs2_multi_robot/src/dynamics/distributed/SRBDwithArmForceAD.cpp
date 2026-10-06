#include "ocs2_multi_robot/dynamics/distributed/SRBDwithArmForceAD.h"
#include <iostream>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace multi_robot {

SRBDwithArmForceAD::SRBDwithArmForceAD(int robot_id, const scalar_t& mass, const matrix3_t& inertia, 
                                       int footNum, int armNum,
                                       const std::string& libraryFolder, bool recompileLibraries,
                                       AlternatingPreComputation& preComputation) 
    : SystemDynamicsBaseAD(preComputation) {
  robotId_ = robot_id;
  mass_ = mass;
  footNum_ = footNum;
  armNum_ = armNum;

  stateSize_ = 12 + footNum_ * 3;
  inputSize_ = footNum_ * QUADRUPED_CONTACT_DIM + 
               footNum_ * 3 + armNum_ * ARM_CONTACT_DIM; // f_j + v_j + f_i + \tau_i
  armPosSize_ = armNum_ * 3;

  // couldn't directly cast to ad_matrix for some reason
  I_b_.setZero(3, 3);
  for (size_t i=0; i<3; i++) {
    for (size_t j=0; j<3; j++) {
      I_b_(i, j) = static_cast<ad_scalar_t>(inertia(i, j));
    }
  }

  this->initialize(stateSize_, inputSize_, "single_rigid_body_dynamics_w_arm" + std::to_string(robotId_), libraryFolder, recompileLibraries, true);
}

ad_vector_t SRBDwithArmForceAD::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                              const ad_vector_t& parameters) const {
  const ad_vector_t r = state.head(3);
  const ad_vector_t dr = state.segment(3, 3);
  const ad_vector_t theta = state.segment(6, 3);
  const ad_vector_t l = state.segment(9, 3);
  // const ad_vector_t p_ee = state.tail(12);
  // const ad_vector_t f = input.head(12);  // leg contact forces
  // const ad_vector_t v_ee = input.segment(12, 12);  // leg end-effector velocities

  ad_vector_t stateDerivative(stateSize_);
  // m * \ddot r = \sum f_j + \sum f_i + mg
  stateDerivative.head(3) = dr;
  ad_vector_t f_sum;
  f_sum.setZero(3);
  // Sum leg contact forces
  for (size_t ee=0; ee < footNum_; ee++) {
    f_sum += input.segment(ee*3, 3);
  }
  // Sum arm forces
  const int armInputStart = footNum_ * QUADRUPED_CONTACT_DIM + 
                            footNum_ * 3;  // after leg forces and velocities
  for (size_t arm=0; arm < armNum_; arm++) {
    f_sum += input.segment(armInputStart + arm*ARM_CONTACT_DIM, 3);  // first 3 dimensions are forces
  }
  ad_vector_t mg(3);
  mg << static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(-mass_*g_);
  stateDerivative.segment(3, 3) = (static_cast<ad_scalar_t>(1.0/mass_))*(f_sum + mg);

  // \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
  // Get rotation matrix for transforming body-frame positions to world frame
  const ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(theta);
  Eigen::Matrix<ad_scalar_t, 3, 3> I_w;
  I_w = w_R_b * I_b_ * w_R_b.transpose();
  ad_vector_t theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(theta, I_w.inverse() * l);
  stateDerivative.segment(6, 3) = theta_dot;

  // \dot l = \sum_j (p_j - r) \cross f_j + \sum_i (p_i - r) \cross f_i + \tau_i
  ad_vector_t tau_sum;
  tau_sum.setZero(3);

  Eigen::Matrix<ad_scalar_t, 3, 1> r_fixed = r;
  // Add torques from leg contact forces
  const ad_vector_t p_ee = state.tail(footNum_ * 3);
  for (size_t ee=0; ee < footNum_; ee++) {
    Eigen::Matrix<ad_scalar_t, 3, 1> f_ee = input.segment(ee*3, 3);
    Eigen::Matrix<ad_scalar_t, 3, 1> ee_pos = p_ee.segment(ee*3, 3);
    tau_sum += f_ee.cross(r_fixed - ee_pos); // has to be fixed size during instantiation
  }
  // Add torques from arm forces and (optionally) explicit arm torques
  for (size_t arm=0; arm < armNum_; arm++) {
    // Get arm force and position
    // Get arm force and torque from input
    Eigen::Matrix<ad_scalar_t, 3, 1> f_arm_ee = input.segment(armInputStart + arm*ARM_CONTACT_DIM, 3);
    
    // Convert arm position in world frame into cppAD type
    Eigen::Matrix<ad_scalar_t, 3, 1> arm_pos_world;
    arm_pos_world << static_cast<ad_scalar_t>(parameters(arm*3 + 0)),
                     static_cast<ad_scalar_t>(parameters(arm*3 + 1)),
                     static_cast<ad_scalar_t>(parameters(arm*3 + 2));
    
    // Add torque from arm force
    tau_sum += f_arm_ee.cross(r_fixed - arm_pos_world);
    
    // Optionally add explicit arm torque if ARM_CONTACT_DIM includes torque (> 3)
    if (USE_ARM_TORQUE_IN_DYNAMICS && ARM_CONTACT_DIM > 3) {
      tau_sum += input.segment(armInputStart + arm*ARM_CONTACT_DIM + 3, 3);
    }
  }
  stateDerivative.segment(9, 3) = tau_sum;

  stateDerivative.segment(12, footNum_ * 3) = input.segment(footNum_ * QUADRUPED_CONTACT_DIM, footNum_ * 3); // leg end-effector velocities

  return stateDerivative;
}

vector_t SRBDwithArmForceAD::getFlowMapParameters(scalar_t time, const PreComputation& preComputation) const {
  const auto& preCompAlternating = cast<AlternatingPreComputation>(preComputation);
  const auto& prevHandlePosition = preCompAlternating.getPrevHandlePositionInWorld(robotId_);
  // if (time < 0.01 && robotId_ == 1) {
  //   std::cerr << "[SRBDwithArmForceAD] Robot " << robotId_ << " time=" << time 
  //             << " prevHandlePosition=" << prevHandlePosition.transpose() 
  //             << " handleCount=" << preCompAlternating.getHandleCount() << std::endl;
  // }
  return prevHandlePosition;
}

} // namespace multi_robot
} // namespace ocs2