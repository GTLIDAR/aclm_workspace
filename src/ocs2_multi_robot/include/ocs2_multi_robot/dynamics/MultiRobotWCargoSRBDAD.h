#pragma once

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Single rigid body dynamics class for N robots with cargo
 * x = [r_1, \dot r_1, \theta_1, l_1, p_ee_1, ..., r_N, \dot r_N, \theta_N, l_N, p_ee_N, r_c, \dot r_c, \theta_c, l_c]
 * u = [f_1, v_ee_1, ..., f_N, v_ee_N, fr_1, taur_1, ..., fr_N, taur_N]
 * 
 * m * \ddot r = \sum f_j + mg
 * \dot l = \sum (p_ee - r) \cross f
 * \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
*/
class MultiRobotWCargoSRBDAD : public SystemDynamicsBaseAD {
public:
  /**
   * Constructor
   * @param [in] robot_mass : Robot total mass.
   * @param [in] cargo_mass : Cargo total mass.
   * @param [in] robot_inertia : The Robot inertia given a fixed joint configuration 
   * @param [in] cargo_inertia : The Cargo inertia
   * @param [in] robotHandles : Vector of 6D transformations for each robot handle [x, y, z, yaw, pitch, roll]
   * @param [in] numRobots : Number of robots
   * @param [in] libraryFolder : Folder to store cppad files.
   * @param [in] recompileLibraries : Flag on whether to recompile the cppad.
   */
  MultiRobotWCargoSRBDAD(const scalar_t& robot_mass, const scalar_t& cargo_mass, 
    const matrix3_t& robot_inertia, const matrix3_t& cargo_inertia, 
    const std::vector<vector_t>& robotHandles, size_t numRobots,
    const std::string& libraryFolder, bool recompileLibraries);

  ~MultiRobotWCargoSRBDAD() override = default;

  MultiRobotWCargoSRBDAD(const MultiRobotWCargoSRBDAD& rhs) = default;

  MultiRobotWCargoSRBDAD* clone() const override { return new MultiRobotWCargoSRBDAD(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, const ad_vector_t& parameters) const override;

private:
  scalar_t robot_mass_;
  scalar_t cargo_mass_;
  scalar_t g_ = 9.81;
  ad_matrix_t robot_I_b_;
  ad_matrix_t cargo_I_b_;
  std::vector<vector_t> robotHandles_;
  size_t numRobots_;
};

} // namespace multi_robot
} // namespace ocs2

