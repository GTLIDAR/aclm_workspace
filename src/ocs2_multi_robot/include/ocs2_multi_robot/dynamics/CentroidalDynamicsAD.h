#pragma once

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Centroidal dynamics class
 * x = [r, \dot r, \theta, l, p_ee]
 * u = [f, v_ee]
 * 
 * m * \ddot r = \sum f_j + mg
 * \dot l = \sum (p_ee - r) \cross f
 * \dot theta = W(I^(-1)*l); W(): map from angular velocity to EA dot
*/
class CentroidalDynamicsAD : public SystemDynamicsBaseAD {
public:
  /**
   * Constructor
   * @param [in] referenceManager : the reference manager with terrain map.
   * @param [in] mass : Robot total mass.
   * @param [in] libraryFolder : library folder path.
   * @param [in] recompileLibraries : whether to recompile the cppad
   */
  CentroidalDynamicsAD(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                       const scalar_t& mass, const std::string& libraryFolder, 
                       bool recompileLibraries);

  ~CentroidalDynamicsAD() override = default;

  CentroidalDynamicsAD(const CentroidalDynamicsAD& rhs) = default;

  CentroidalDynamicsAD* clone() const override { return new CentroidalDynamicsAD(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, const ad_vector_t& parameters) const override;

  vector_t getFlowMapParameters(scalar_t time, const PreComputation& preComputation) const override;

  size_t getNumFlowMapParameters() const override { return STATE_DIM_AE + 6; }

private:
  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  scalar_t mass_;
  scalar_t g_ = 9.81;
  // ad_matrix_t I_b_;
};

} // namespace multi_robot
} // namespace ocs2