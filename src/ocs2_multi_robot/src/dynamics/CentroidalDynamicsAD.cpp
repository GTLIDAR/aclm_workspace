#include "ocs2_multi_robot/dynamics/CentroidalDynamicsAD.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace multi_robot {

CentroidalDynamicsAD::CentroidalDynamicsAD(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                                           const scalar_t& mass, const std::string& libraryFolder, bool recompileLibraries) 
  : mass_(mass),
    referenceManagerPtr_(&referenceManager){
  // I_b_.resize(3, 3);
  // I_b_.setIdentity();
  this->initialize(STATE_DIM_AE, INPUT_DIM_AE, "centroidal_dynamics_ae_only", libraryFolder, recompileLibraries, true);
}

ad_vector_t CentroidalDynamicsAD::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
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
  Eigen::Matrix<ad_scalar_t, 3, 3> I_w;
  const ad_vector_t I_w_ref = parameters.tail(6);
  I_w << I_w_ref(0), I_w_ref(1), I_w_ref(3),
         I_w_ref(1), I_w_ref(2), I_w_ref(4),
         I_w_ref(3), I_w_ref(4), I_w_ref(5);
  
  // ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(theta);
  // Eigen::Matrix<ad_scalar_t, 3, 3> I_w;
  // I_w = w_R_b * I_b * w_R_b.transpose();
  ad_vector_t theta_dot = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ad_scalar_t>(theta, I_w.inverse() * l);
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
  
  return stateDerivative;
}

vector_t CentroidalDynamicsAD::getFlowMapParameters(scalar_t time, const PreComputation& preComputation) const {
  const TargetTrajectories& targetTrajectories =
      referenceManagerPtr_->getTargetTrajectories();
  const vector_t stateReference = targetTrajectories.getDesiredState(time);
  return stateReference;
}

} // namespace multi_robot
} // namespace ocs2