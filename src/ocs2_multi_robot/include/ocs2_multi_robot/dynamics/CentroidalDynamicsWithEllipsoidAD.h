#pragma once

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include "ocs2_multi_robot/reference_manager/SwitchedModelReferenceManagerWithTerrain.h"

#include "ocs2_multi_robot/common/Types.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Centroidal dynamics class with ellipsoid (e: size; \gamma: orientation)
 * x = [r, \dot r, \theta, l, p_ee, e, \gamma]
 * u = [f, v_ee, \dot e, \omega_gamma]
 * 
 * m * \ddot r = \sum f_j + mg
 * \dot l = \sum (p_ee - r) \cross f
 * \dot \theta = W(I_ellip^(-1)*l); W(): map from angular velocity to EA dot
 * \dot \gamma = W(\omega_gamma)
*/
class CentroidalDynamicsWithEllipsoidAD : public SystemDynamicsBaseAD {
public:
  /**
   * Constructor
   * @param [in] mass : Robot total mass.
   * @param [in] libraryFolder : The 3 DoF contact index.
   * @param [in] recompileLibraries : The centroidal model information.
   */
  CentroidalDynamicsWithEllipsoidAD(const SwitchedModelReferenceManagerWithTerrain& referenceManager,
                       const scalar_t& mass, const std::string& libraryFolder, 
                       bool recompileLibraries);

  ~CentroidalDynamicsWithEllipsoidAD() override = default;

  CentroidalDynamicsWithEllipsoidAD(const CentroidalDynamicsWithEllipsoidAD& rhs) = default;

  CentroidalDynamicsWithEllipsoidAD* clone() const override { return new CentroidalDynamicsWithEllipsoidAD(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, const ad_vector_t& parameters) const override;

  // vector_t getFlowMapParameters(scalar_t time, const PreComputation& preComputation) const override;

  // size_t getNumFlowMapParameters() const override { return STATE_DIM_AE + 6; }

private:
  const SwitchedModelReferenceManagerWithTerrain* referenceManagerPtr_;
  scalar_t mass_;
  scalar_t g_ = 9.81;
  // ad_matrix_t I_b_;
};

} // namespace multi_robot
} // namespace ocs2