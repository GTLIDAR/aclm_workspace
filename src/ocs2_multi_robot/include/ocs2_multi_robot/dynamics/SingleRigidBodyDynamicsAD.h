#pragma once

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Single rigid body dynamics class for 
 * x = [r, \dot r, \theta, l, p_ee]
 * u = [f, v_ee]
 * 
 * m * \ddot r = \sum f_j + mg
 * \dot l = \sum (p_ee - r) \cross f
 * \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
*/
class SingleRigidBodyDynamicsAD : public SystemDynamicsBaseAD {
public:
  /**
   * Constructor
   * @param [in] mass : Robot total mass.
   * @param [in] inertia : The Robot inertia given a fixed joint configuration 
   * @param [in] libraryFolder : Folder to store cppad files.
   * @param [in] recompileLibraries : Flag on whether to recompile the cppad.
   */
  SingleRigidBodyDynamicsAD(const scalar_t& mass, const matrix3_t& inertia, const std::string& libraryFolder, bool recompileLibraries);

  ~SingleRigidBodyDynamicsAD() override = default;

  SingleRigidBodyDynamicsAD(const SingleRigidBodyDynamicsAD& rhs) = default;

  SingleRigidBodyDynamicsAD* clone() const override { return new SingleRigidBodyDynamicsAD(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, const ad_vector_t& parameters) const override;

private:
  scalar_t mass_;
  scalar_t g_ = 9.81;
  ad_matrix_t I_b_;
};

} // namespace multi_robot
} // namespace ocs2