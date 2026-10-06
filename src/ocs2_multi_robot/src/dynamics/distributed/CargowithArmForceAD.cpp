#include "ocs2_multi_robot/dynamics/distributed/CargowithArmForceAD.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace multi_robot {

CargowithArmForceAD::CargowithArmForceAD(const scalar_t& mass, const matrix3_t& inertia, 
                                         int armNum, std::vector<vector_t> armHandlePositions,
                                         const std::string& libraryFolder, bool recompileLibraries) {
  mass_ = mass;
  armNum_ = armNum;
  armHandlePositions_ = std::move(armHandlePositions);

  stateSize_ = CARGO_STATE_DIM;
  inputSize_ = armNum_ * ARM_CONTACT_DIM; // f_j + v_j + f_i + \tau_i

  // couldn't directly cast to ad_matrix for some reason
  I_b_.setZero(3, 3);
  for (size_t i=0; i<3; i++) {
    for (size_t j=0; j<3; j++) {
      I_b_(i, j) = static_cast<ad_scalar_t>(inertia(i, j));
    }
  }
  
  this->initialize(stateSize_, inputSize_, "cargo_dynamics_w_arm" + std::to_string(armNum_), libraryFolder,
                   recompileLibraries, true);
}

ad_vector_t CargowithArmForceAD::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                               const ad_vector_t& parameters) const {
  const ad_vector_t r = state.head(3);
  const ad_vector_t dr = state.segment(3, 3);
  const ad_vector_t theta = state.segment(6, 3);
  const ad_vector_t l = state.segment(9, 3);

  ad_vector_t stateDerivative(stateSize_);
  // m * \ddot r = \sum f_j + \sum f_i + mg
  stateDerivative.head(3) = dr;
  ad_vector_t f_sum;
  f_sum.setZero(3);

  // Sum arm forces
  for (size_t arm=0; arm < armNum_; arm++) {
    f_sum += input.segment(arm*ARM_CONTACT_DIM, 3);  // first 3 dimensions are forces
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

  // \dot l = \sum_i (p_i - r) \cross f_i + \tau_i
  ad_vector_t tau_sum;
  tau_sum.setZero(3);

  Eigen::Matrix<ad_scalar_t, 3, 1> r_fixed = r;
  // Add torques from arm forces and explicit arm torques
  for (size_t arm=0; arm < armNum_; arm++) {
    // Get arm force and position
    // Get arm force and torque from input
    Eigen::Matrix<ad_scalar_t, 3, 1> f_arm_ee = input.segment(arm*ARM_CONTACT_DIM, 3);
    
    // Convert arm position from body frame to world frame
    Eigen::Matrix<ad_scalar_t, 3, 1> arm_pos_body;
    arm_pos_body << static_cast<ad_scalar_t>(armHandlePositions_[arm](0)),
                    static_cast<ad_scalar_t>(armHandlePositions_[arm](1)),
                    static_cast<ad_scalar_t>(armHandlePositions_[arm](2));
    Eigen::Matrix<ad_scalar_t, 3, 1> arm_pos_world = w_R_b * arm_pos_body;

    // Add torque from arm force
    tau_sum += f_arm_ee.cross(-arm_pos_world);
    
    // Optionally add explicit arm torque if ARM_CONTACT_DIM includes torque (> 3)
    if (USE_ARM_TORQUE_IN_DYNAMICS && ARM_CONTACT_DIM > 3) {
      tau_sum += input.segment(arm*ARM_CONTACT_DIM + 3, 3);
    }
  }
  stateDerivative.segment(9, 3) = tau_sum;

  return stateDerivative;
}

} // namespace multi_robot
} // namespace ocs2