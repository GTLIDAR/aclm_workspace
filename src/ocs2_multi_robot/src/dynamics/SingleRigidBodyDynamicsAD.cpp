#include "ocs2_multi_robot/dynamics/SingleRigidBodyDynamicsAD.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace multi_robot {

SingleRigidBodyDynamicsAD::SingleRigidBodyDynamicsAD(const scalar_t& mass, const matrix3_t& inertia, const std::string& libraryFolder, bool recompileLibraries) {
  mass_ = mass;
  // couldn't directly cast to ad_matrix for some reason
  I_b_.setZero(3, 3);
  for (size_t i=0; i<3; i++) {
    for (size_t j=0; j<3; j++) {
      I_b_(i, j) = static_cast<ad_scalar_t>(inertia(i, j));
    }
  }
  
  this->initialize(STATE_DIM_AE, INPUT_DIM_AE, "single_rigid_body_dynamics_ae_only", libraryFolder, recompileLibraries, true);
}

ad_vector_t SingleRigidBodyDynamicsAD::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                 const ad_vector_t& parameters) const {
  const ad_vector_t r = state.head(3);
  const ad_vector_t dr = state.segment(3, 3);
  const ad_vector_t theta = state.segment(6, 3);
  const ad_vector_t l = state.segment(9, 3);
  const ad_vector_t p_ee = state.tail(12);
  const ad_vector_t f = input.head(12);
  const ad_vector_t v_ee = input.tail(12);

  ad_vector_t stateDerivative(STATE_DIM_AE);
  // m * \ddot r = \sum f_j + mg
  stateDerivative.head(3) = dr;
  ad_vector_t f_sum;
  f_sum.setZero(3);
  for (size_t ee=0; ee < ROBOTS_FOOT_NUM; ee++) {
    f_sum += f.segment(ee*3, 3);
  }
  ad_vector_t mg(3);
  mg << static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(-mass_*g_);
  stateDerivative.segment(3, 3) = (static_cast<ad_scalar_t>(1.0/mass_))*(f_sum + mg);

  // \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
  ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(theta);
  Eigen::Matrix<ad_scalar_t, 3, 3> I_w;
  I_w = w_R_b * I_b_ * w_R_b.transpose();
  ad_vector_t theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(theta, I_w.inverse() * l);
  stateDerivative.segment(6, 3) = theta_dot;

  // \dot l = \sum (p_ee - r) \cross f
  ad_vector_t tau_sum;
  tau_sum.setZero(3);

  Eigen::Matrix<ad_scalar_t, 3, 1> r_fixed = r;
  for (size_t ee=0; ee < ROBOTS_FOOT_NUM; ee++) {
    Eigen::Matrix<ad_scalar_t, 3, 1> f_ee = f.segment(ee*3, 3);
    Eigen::Matrix<ad_scalar_t, 3, 1> ee_pos = p_ee.segment(ee*3, 3);
    tau_sum += f_ee.cross(r_fixed - ee_pos); // has to be fixed size during instaniation
  }
  stateDerivative.segment(9, 3) = tau_sum;

  stateDerivative.segment(12, 12) = v_ee;
  
  return stateDerivative;
}

} // namespace multi_robot
} // namespace ocs2