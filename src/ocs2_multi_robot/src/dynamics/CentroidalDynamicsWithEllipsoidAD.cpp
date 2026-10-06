#include "ocs2_multi_robot/dynamics/CentroidalDynamicsWithEllipsoidAD.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace multi_robot {

CentroidalDynamicsWithEllipsoidAD::CentroidalDynamicsWithEllipsoidAD(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                           const scalar_t& mass, const std::string& libraryFolder, bool recompileLibraries) 
  : mass_(mass),
    referenceManagerPtr_(&referenceManager){
  // I_b_.resize(3, 3);
  // I_b_.setIdentity();
  this->initialize(STATE_DIM_AE_WITH_ELLIPSOID, INPUT_DIM_AE_WITH_ELLIPSOID, "centroidal_dynamics_ae_with_ellipsoid", libraryFolder, recompileLibraries, true);
}

ad_vector_t CentroidalDynamicsWithEllipsoidAD::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                 const ad_vector_t& parameters) const {
  const ad_vector_t r = state.head(3);
  const ad_vector_t dr = state.segment(3, 3);
  const ad_vector_t theta = state.segment(6, 3);
  const ad_vector_t l = state.segment(9, 3);
  const ad_vector_t p_ee = state.segment(12, 12);
  const ad_vector_t e = state.segment(24, 3);
  const ad_vector_t gamma = state.segment(27, 3);

  const ad_vector_t f = input.head(12);
  const ad_vector_t v_ee = input.segment(12, 12);
  const ad_vector_t e_dot = input.segment(24, 3);
  const ad_vector_t omega_gamma = input.segment(27, 3);

  ad_vector_t stateDerivative(STATE_DIM_AE_WITH_ELLIPSOID);
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

  // \dot theta = W(I_ellip^(-1)*l); W(): map from angular velocity to EA dot
  ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(gamma);
  Eigen::Matrix<ad_scalar_t, 3, 3> I_ellip_w, I_ellip_b;
  ad_scalar_t I_xx = static_cast<ad_scalar_t>(1.0/5.0*mass_)*(pow(e(1), 2) + pow(e(2), 2));
  ad_scalar_t I_yy = static_cast<ad_scalar_t>(1.0/5.0*mass_)*(pow(e(2), 2) + pow(e(0), 2));
  ad_scalar_t I_zz = static_cast<ad_scalar_t>(1.0/5.0*mass_)*(pow(e(0), 2) + pow(e(1), 2));
  I_ellip_b.setIdentity();
  I_ellip_b(0, 0) = I_xx;
  I_ellip_b(1, 1) = I_yy;
  I_ellip_b(2, 2) = I_zz;
  I_ellip_w = w_R_b * I_ellip_b * w_R_b.transpose();
  
  // ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(theta);
  // Eigen::Matrix<ad_scalar_t, 3, 3> I_w;
  // I_w = w_R_b * I_b * w_R_b.transpose();
  ad_vector_t theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(theta, I_ellip_w.inverse() * l);
  stateDerivative.segment(6, 3) = theta_dot;

  // \dot l = \sum (p_ee - r) \cross f
  ad_vector_t tau_sum;
  tau_sum.setZero(3);

  Eigen::Matrix<ad_scalar_t, 3, 1> r_fixed = r;
  for (size_t ee=0; ee < ROBOTS_FOOT_NUM; ee++) {
    Eigen::Matrix<ad_scalar_t, 3, 1> f_ee = f.segment(ee*3, 3);
    Eigen::Matrix<ad_scalar_t, 3, 1> ee_pos = p_ee.segment(ee*3, 3);
    tau_sum += f_ee.cross(r_fixed - ee_pos); // has to be fixed size during instantiation
  }
  stateDerivative.segment(9, 3) = tau_sum;

  stateDerivative.segment(12, 12) = v_ee;

  stateDerivative.segment(24, 3) = e_dot;

  ad_vector_t gamma_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(gamma, omega_gamma);
  stateDerivative.segment(27, 3) = gamma_dot;
  
  return stateDerivative;
}

// vector_t CentroidalDynamicsWithEllipsoidAD::getFlowMapParameters(scalar_t time, const PreComputation& preComputation) const {
//   const TargetTrajectories& targetTrajectories =
//       referenceManagerPtr_->getTargetTrajectories();
//   const vector_t stateReference = targetTrajectories.getDesiredState(time);
//   return stateReference;
// }

} // namespace multi_robot
} // namespace ocs2