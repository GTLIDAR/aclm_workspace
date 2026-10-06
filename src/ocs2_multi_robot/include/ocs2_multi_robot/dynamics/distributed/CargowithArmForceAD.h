#pragma once

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/AlternatingPreComputation.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Single rigid body dynamics class for the cargo with arm forces
 * x = [r, \dot r, \theta, l]
 * u = [f_i, \tau_i]
 * 
 * m * \ddot r = \sum_i f_i + mg
 * \dot l = \sum_i (p_i - r) \cross f_i + \tau_i
 * \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
*/
class CargowithArmForceAD : public SystemDynamicsBaseAD {
public:
  /**
   * Constructor
   * @param [in] mass : Robot total mass.
   * @param [in] inertia : The Robot inertia given a fixed joint configuration
   * @param [in] armNum : Number of arm end-effectors.
   * @param [in] armHandlePositions : The positions of the arm end-effectors in the cargo frame.
   * @param [in] libraryFolder : Folder to store cppad files.
   * @param [in] recompileLibraries : Flag on whether to recompile the cppad.
   */
  CargowithArmForceAD(const scalar_t& mass, const matrix3_t& inertia,
                      int armNum, std::vector<vector_t> armHandlePositions,
                      const std::string& libraryFolder, bool recompileLibraries);

  ~CargowithArmForceAD() override = default;

  CargowithArmForceAD(const CargowithArmForceAD& rhs) = default;

  CargowithArmForceAD* clone() const override { return new CargowithArmForceAD(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, const ad_vector_t& parameters) const override;

private:
  scalar_t mass_;
  scalar_t g_ = 9.81;
  ad_matrix_t I_b_;
  int footNum_;
  int armNum_;
  int stateSize_;
  int inputSize_;
  std::vector<vector_t> armHandlePositions_;
};

} // namespace multi_robot
} // namespace ocs2