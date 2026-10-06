#pragma once

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/AlternatingPreComputation.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Single rigid body dynamics class for 
 * x = [r, \dot r, \theta, l, p_j]
 * u = [f_j, v_j, f_i, \tau_i]
 * 
 * m * \ddot r = \sum_j f_j + \sum_i f_i + mg
 * \dot l = \sum_j (p_j - r) \cross f_j + \sum_i (p_i - r) \cross f_i + \tau_i
 * \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
*/
class SRBDwithArmForceAD : public SystemDynamicsBaseAD {
public:
  /**
   * Constructor
   * @param [in] mass : Robot total mass.
   * @param [in] inertia : The Robot inertia given a fixed joint configuration
   * @param [in] footNum : Number of foot end-effectors.
   * @param [in] armNum : Number of arm end-effectors. 
   * @param [in] libraryFolder : Folder to store cppad files.
   * @param [in] recompileLibraries : Flag on whether to recompile the cppad.
   */
  SRBDwithArmForceAD(int robotId, const scalar_t& mass, const matrix3_t& inertia,
                     int footNum, int armNum,
                     const std::string& libraryFolder, bool recompileLibraries,
                     AlternatingPreComputation& preComputation);

  ~SRBDwithArmForceAD() override = default;

  SRBDwithArmForceAD(const SRBDwithArmForceAD& rhs) = default;

  SRBDwithArmForceAD* clone() const override { return new SRBDwithArmForceAD(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, const ad_vector_t& parameters) const override;

  /** Get the flow map parameters 
   * The returned vector represents the arm end-effector positions 
   * in the world frame at the current time:
   * [p_ee1_x, p_ee1_y, p_ee1_z, ..., p_eeN_x, p_eeN_y, p_eeN_z]
  */
  vector_t getFlowMapParameters(scalar_t time, const PreComputation& preComputation) const override;

  size_t getNumFlowMapParameters() const override { return armPosSize_; }

  
private:
  scalar_t mass_;
  scalar_t g_ = 9.81;
  ad_matrix_t I_b_;
  int footNum_;
  int armNum_;
  int stateSize_;
  int inputSize_;
  int armPosSize_;
  int robotId_;
};

} // namespace multi_robot
} // namespace ocs2