#pragma once

#include <math.h>

#include "ocs2_multi_robot/common/Types.h"
#include <ocs2_core/cost/StateCostCppAd.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace multi_robot {

/**
 * Ellipsoid moment of inertia tracking 
 * w_R_b * I_ellip_b * w_R_b.transpose() = I_w_desired
*/
class EllipsoidMoITrackingCostAD final : public StateCostCppAd {
public:
  EllipsoidMoITrackingCostAD(scalar_t weight, const scalar_t& mass, const std::string& libraryFolder,
                          bool recompileLibraries)
    : StateCostCppAd() {
    weight_ = weight;
    mass_ = mass;
    this->initialize(STATE_DIM_AE_WITH_ELLIPSOID, 6, "ellipsoid_moi_tracking", 
      libraryFolder, recompileLibraries);
  }

  ~EllipsoidMoITrackingCostAD() override = default;

  EllipsoidMoITrackingCostAD(const EllipsoidMoITrackingCostAD& rhs) = default;

  EllipsoidMoITrackingCostAD* clone() const override {return new EllipsoidMoITrackingCostAD(*this); }

  ad_scalar_t costFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override {
    const ad_vector_t e = state.segment(24, 3);
    const ad_vector_t gamma = state.segment(27, 3);
    
    ad_matrix_t w_R_b = getRotationMatrixFromZyxEulerAngles<ad_scalar_t>(gamma);
    Eigen::Matrix<ad_scalar_t, 3, 3> I_ellip_w, I_ellip_b, I_w_desired;
    ad_scalar_t I_xx = static_cast<ad_scalar_t>(1.0/5.0*mass_)*(pow(e(1), 2) + pow(e(2), 2));
    ad_scalar_t I_yy = static_cast<ad_scalar_t>(1.0/5.0*mass_)*(pow(e(2), 2) + pow(e(0), 2));
    ad_scalar_t I_zz = static_cast<ad_scalar_t>(1.0/5.0*mass_)*(pow(e(0), 2) + pow(e(1), 2));
    I_ellip_b.setIdentity();
    I_ellip_b(0, 0) = I_xx;
    I_ellip_b(1, 1) = I_yy;
    I_ellip_b(2, 2) = I_zz;
    I_ellip_w = w_R_b * I_ellip_b * w_R_b.transpose();

    I_w_desired << parameters(0), parameters(1), parameters(3),
                   parameters(1), parameters(2), parameters(4),
                   parameters(3), parameters(4), parameters(5); 

    return weight_*((I_ellip_w - I_w_desired).transpose()*(I_ellip_w - I_w_desired)).trace();
  }

  vector_t getParameters(scalar_t time, const TargetTrajectories& targetTrajectories,
                                 const PreComputation& /* preComputation */) const override {
    return targetTrajectories.getDesiredState(time).tail(6);
  }

private:
  scalar_t weight_;
  scalar_t mass_;

};
}
}