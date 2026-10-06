#include <pinocchio/fwd.hpp>

#include "ocs2_multi_robot/dynamics/TwoQuadwCargoSRBDAD.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace multi_robot {

TwoQuadwCargoSRBDAD::TwoQuadwCargoSRBDAD(const scalar_t& robot_mass, const scalar_t& cargo_mass, 
  const matrix3_t& robot_inertia, const matrix3_t& cargo_inertia, 
  const vector_t& r1_handle, const vector_t& r2_handle, const std::string& libraryFolder, bool recompileLibraries) {
  robot_mass_ = robot_mass;
  cargo_mass_ = cargo_mass;
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
  r1_handle_ = r1_handle;
  r2_handle_ = r2_handle;
  this->initialize(STATE_DIM_TWO_QUAD_W_CARGO, INPUT_DIM_TWO_QUAD_W_CARGO, "single_rigid_body_dynamics_ae_only", libraryFolder, recompileLibraries, true);
}

ad_vector_t TwoQuadwCargoSRBDAD::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                 const ad_vector_t& parameters) const {
  const ad_vector_t r1 = state.segment(0, 3); // position
  const ad_vector_t dr1 = state.segment(3, 3); // velocity
  const ad_vector_t theta1 = state.segment(6, 3); // euler angles
  const ad_vector_t l1 = state.segment(9, 3); // angular momentum
  const ad_vector_t p_ee1 = state.segment(12,12); // foot end-effector positions
  const ad_vector_t f1 = input.segment(0, 12); // foot end-effector forces
  const ad_vector_t v_ee1 = input.segment(12,12); // foot end-effector velocities

  const ad_vector_t r2 = state.segment(24, 3);
  const ad_vector_t dr2 = state.segment(27, 3);
  const ad_vector_t theta2 = state.segment(30, 3);
  const ad_vector_t l2 = state.segment(33, 3);
  const ad_vector_t p_ee2 = state.segment(36,12);
  const ad_vector_t f2 = input.segment(24, 12);
  const ad_vector_t v_ee2 = input.segment(36,12);

  const ad_vector_t rc = state.segment(48, 3);
  const ad_vector_t drc = state.segment(51, 3);
  const ad_vector_t thetac = state.segment(54, 3);
  const ad_vector_t lc = state.segment(57, 3);
  const ad_vector_t fr1 = input.segment(48, 3);
  const ad_vector_t taur1 = input.segment(51, 3);
  const ad_vector_t fr2 = input.segment(54, 3);
  const ad_vector_t taur2 = input.segment(57, 3);

  Eigen::Matrix<ad_scalar_t, 3, 1> left_handle_offset_, right_handle_offset_, fr1_, fr2_;
  ad_matrix_t r1_R_, r2_R_;
  fr1_ = fr1;
  fr2_ = fr2;

  // Extract 6D transformation: x, y, z, yaw, pitch, roll
  // Handle position components
  left_handle_offset_ << static_cast<ad_scalar_t>(r1_handle_(0)), static_cast<ad_scalar_t>(r1_handle_(1)), static_cast<ad_scalar_t>(r1_handle_(2));
  right_handle_offset_ << static_cast<ad_scalar_t>(r2_handle_(0)), static_cast<ad_scalar_t>(r2_handle_(1)), static_cast<ad_scalar_t>(r2_handle_(2));
  
  // Orientation components (yaw, pitch, roll)
  ad_vector_t r1_euler(3), r2_euler(3);
  r1_euler << static_cast<ad_scalar_t>(r1_handle_(3)), static_cast<ad_scalar_t>(r1_handle_(4)), static_cast<ad_scalar_t>(r1_handle_(5));
  r2_euler << static_cast<ad_scalar_t>(r2_handle_(3)), static_cast<ad_scalar_t>(r2_handle_(4)), static_cast<ad_scalar_t>(r2_handle_(5));
  
  // Create rotation matrices from Euler angles (ZYX order)
  r1_R_ = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(r1_euler);
  r2_R_ = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(r2_euler);

  /************************************** Robot1 *******************************************/
  ad_vector_t stateDerivative(STATE_DIM_TWO_QUAD_W_CARGO);
  // m * \ddot r = \sum f_j + mg, linear acceleration
  stateDerivative.segment(0, 3) = dr1;
  ad_vector_t f_sum;
  f_sum.setZero(3);
  for (size_t ee=0; ee < QUADRUPED_FOOT_NUM; ee++) {
    f_sum += f1.segment(ee*3, 3);
  }
  f_sum -= fr1;
  ad_vector_t mg(3);
  mg << static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(-robot_mass_*g_);
  stateDerivative.segment(3, 3) = (static_cast<ad_scalar_t>(1.0/robot_mass_))*(f_sum + mg);

  // \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
  ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(theta1);
  Eigen::Matrix<ad_scalar_t, 3, 3> I_w;
  I_w = w_R_b * robot_I_b_ * w_R_b.transpose();
  ad_vector_t theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(theta1, I_w.inverse() * l1);
  stateDerivative.segment(6, 3) = theta_dot;

  // \dot l = \sum (p_ee - r) \cross f moumentum rate
  ad_vector_t tau_sum;
  tau_sum.setZero(3);

  Eigen::Matrix<ad_scalar_t, 3, 1> r_fixed = r1;
  for (size_t ee=0; ee < QUADRUPED_FOOT_NUM; ee++) {
    Eigen::Matrix<ad_scalar_t, 3, 1> f_ee = f1.segment(ee*3, 3);
    Eigen::Matrix<ad_scalar_t, 3, 1> ee_pos = p_ee1.segment(ee*3, 3);
    // tau_sum += f_ee.cross(r_fixed - ee_pos); // has to be fixed size during instaniation
    Eigen::Matrix<ad_scalar_t, 3, 1> ee_rel_pos = ee_pos - r_fixed;
    tau_sum += ee_rel_pos.cross(f_ee);
  }

  ad_matrix_t w_R_cargo = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(thetac);
  Eigen::Matrix<ad_scalar_t, 3, 1> r1_pos_world = rc + w_R_cargo * left_handle_offset_;
  // tau_sum += fr1_.cross(r_fixed - r1_pos_world);
  Eigen::Matrix<ad_scalar_t, 3, 1> r1_rel_pos = r1_pos_world - r_fixed;
  tau_sum += -r1_rel_pos.cross(fr1_);
  tau_sum -= taur1;

  stateDerivative.segment(9, 3) = tau_sum;

  stateDerivative.segment(12, 12) = v_ee1;

  /************************************** Robot2 *******************************************/
  
  // m * \ddot r = \sum f_j + mg
  stateDerivative.segment(24, 3) = dr2;
  f_sum.setZero(3);
  for (size_t ee=0; ee < QUADRUPED_FOOT_NUM; ee++) {
    f_sum += f2.segment(ee*3, 3);
  }
  f_sum -= fr2;

  stateDerivative.segment(27, 3) = (static_cast<ad_scalar_t>(1.0/robot_mass_))*(f_sum + mg);

  // \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
  w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(theta2);

  I_w = w_R_b * robot_I_b_ * w_R_b.transpose();
  theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(theta2, I_w.inverse() * l2);
  stateDerivative.segment(30, 3) = theta_dot;

  // \dot l = \sum (p_ee - r) \cross f
  tau_sum.setZero(3);

  r_fixed = r2;
  for (size_t ee=0; ee < QUADRUPED_FOOT_NUM; ee++) {
    Eigen::Matrix<ad_scalar_t, 3, 1> f_ee = f2.segment(ee*3, 3);
    Eigen::Matrix<ad_scalar_t, 3, 1> ee_pos = p_ee2.segment(ee*3, 3);
    // tau_sum += f_ee.cross(r_fixed - ee_pos); // has to be fixed size during instaniation
    Eigen::Matrix<ad_scalar_t, 3, 1> ee_rel_pos = ee_pos - r_fixed;
    tau_sum += ee_rel_pos.cross(f_ee);
  }
  Eigen::Matrix<ad_scalar_t, 3, 1> r2_pos_world = rc + w_R_cargo * right_handle_offset_;
  // tau_sum += fr2_.cross(r_fixed - r2_pos_world);
  Eigen::Matrix<ad_scalar_t, 3, 1> r2_rel_pos = r2_pos_world - r_fixed;
  tau_sum += -r2_rel_pos.cross(fr2_);
  tau_sum -= taur2;

  stateDerivative.segment(33, 3) = tau_sum;

  stateDerivative.segment(36, 12) = v_ee2;

  /***************************************** Cargo ****************************************/
  ad_vector_t mg_cargo(3);
  mg_cargo << static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(-cargo_mass_*g_);

  stateDerivative.segment(48, 3) = drc;
  f_sum.setZero(3);
  f_sum = fr1 + fr2;

  stateDerivative.segment(51, 3) = (static_cast<ad_scalar_t>(1.0/cargo_mass_))*(f_sum + mg_cargo);

  Eigen::Matrix<ad_scalar_t, 3, 3> I_w_cargo;
  I_w_cargo = w_R_cargo * cargo_I_b_ * w_R_cargo.transpose();
  theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(thetac, I_w_cargo.inverse() * lc);
  stateDerivative.segment(54, 3) = theta_dot;

  tau_sum.setZero(3);
  Eigen::Matrix<ad_scalar_t, 3, 1> r1_rel_world = w_R_cargo * left_handle_offset_;
  Eigen::Matrix<ad_scalar_t, 3, 1> r2_rel_world = w_R_cargo * right_handle_offset_;
  // tau_sum += (fr1_.cross(r1_rel_world)) + (fr2_.cross(r2_rel_world));
  tau_sum += r1_rel_world.cross(fr1_) + r2_rel_world.cross(fr2_);
  tau_sum += taur1 + taur2;
  stateDerivative.segment(57, 3) = tau_sum;

  return stateDerivative;
}

} // namespace multi_robot
} // namespace ocs2