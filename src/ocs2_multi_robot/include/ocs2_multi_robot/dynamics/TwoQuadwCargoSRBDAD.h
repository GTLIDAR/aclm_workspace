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
class TwoQuadwCargoSRBDAD : public SystemDynamicsBaseAD {
public:
  /**
   * Constructor
   * @param [in] robot_mass : Robot total mass.
   * @param [in] cargo_mass : Cargo total mass.
   * @param [in] robot_inertia : The Robot inertia given a fixed joint configuration 
   * @param [in] cargo_inertia : The Cargo inertia
   * @param [in] r1_handle : 6D transformation for robot1 handle [x, y, z, yaw, pitch, roll]
   * @param [in] r2_handle : 6D transformation for robot2 handle [x, y, z, yaw, pitch, roll]
   * @param [in] libraryFolder : Folder to store cppad files.
   * @param [in] recompileLibraries : Flag on whether to recompile the cppad.
   */
  TwoQuadwCargoSRBDAD(const scalar_t& robot_mass, const scalar_t& cargo_mass, 
    const matrix3_t& robot_inertia, const matrix3_t& cargo_inertia, 
    const vector_t& r1_handle, const vector_t& r2_handle, const std::string& libraryFolder, bool recompileLibraries);

  ~TwoQuadwCargoSRBDAD() override = default;

  TwoQuadwCargoSRBDAD(const TwoQuadwCargoSRBDAD& rhs) = default;

  TwoQuadwCargoSRBDAD* clone() const override { return new TwoQuadwCargoSRBDAD(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, const ad_vector_t& parameters) const override;

private:
  scalar_t robot_mass_;
  scalar_t cargo_mass_;
  scalar_t g_ = 9.81;
  ad_matrix_t robot_I_b_;
  ad_matrix_t cargo_I_b_;
  vector_t r1_handle_;
  vector_t r2_handle_;
};

} // namespace multi_robot
} // namespace ocs2