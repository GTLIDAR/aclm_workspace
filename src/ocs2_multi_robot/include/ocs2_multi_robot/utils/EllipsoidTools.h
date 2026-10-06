#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>

#include <utility>

namespace ocs2 {
namespace multi_robot {

/**
 * Compute the ellipsoid axes given the sorted eigen values and eigen vectors
*/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> CalEquimomentalEllipsoid(
                                            const Eigen::Matrix<SCALAR_T, 3, 1>& I_w_eigenval,
                                            const SCALAR_T& robot_mass) {
  // Sort the eigenvalues and eigenvectors
  SCALAR_T eval1 = I_w_eigenval[0];
  SCALAR_T eval2 = I_w_eigenval[1];
  SCALAR_T eval3 = I_w_eigenval[2];

  // Compute ellipsoid axes from eigenvalues
  Eigen::Matrix<SCALAR_T, 3, 1> ellipsoid_axes;
  SCALAR_T rho = pow(3.0/4.0*robot_mass, 5.0/2.0)*pow(8*M_PI/15.0, 3.0/2.0)/
               (pow(M_PI, 5.0/2.0)*pow(eval1*eval2*eval3, 0.5));
  ellipsoid_axes(0) = pow(eval1*eval2*eval3, 2.0/5.0)/(eval1*pow(8*M_PI*rho/15.0, 1.0/5.0));
  ellipsoid_axes(1) = pow(eval1*eval2*eval3, 2.0/5.0)/(eval2*pow(8*M_PI*rho/15.0, 1.0/5.0));
  ellipsoid_axes(2) = pow(eval1*eval2*eval3, 2.0/5.0)/(eval3*pow(8*M_PI*rho/15.0, 1.0/5.0));

  return ellipsoid_axes;
}

}
}