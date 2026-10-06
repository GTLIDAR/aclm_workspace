#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>

namespace ocs2 {
namespace multi_robot {

/**
 * Compute the derivative of rotation matrix right multiplied by a vector w.r.t. ZYX euler angles variables
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] right_vec: the vector being right multiplied
 * @return 3x3 matrix jacobian
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotMatrixJacobianWrtZYXEulerRightMultiplyVec(const Eigen::Matrix<SCALAR_T, 3, 1>& euler_angle,
                                                                              const Eigen::Matrix<SCALAR_T, 3, 1>& right_vec) {
  SCALAR_T x = euler_angle[2];
  SCALAR_T y = euler_angle[1];
  SCALAR_T z = euler_angle[0];

  // R is the rotation matrix from body(base) to world
  Eigen::Matrix<SCALAR_T, 3, 3> dR_dx, dR_dy, dR_dz;

  dR_dx <<  0, sin(x)*sin(z) + cos(x)*cos(z)*sin(y),   cos(x)*sin(z) - cos(z)*sin(x)*sin(y),
            0, cos(x)*sin(y)*sin(z) - cos(z)*sin(x), - cos(x)*cos(z) - sin(x)*sin(y)*sin(z),
            0,                        cos(x)*cos(y),                         -cos(y)*sin(x);

  dR_dy << -cos(z)*sin(y), cos(y)*cos(z)*sin(x), cos(x)*cos(y)*cos(z),
            -sin(y)*sin(z), cos(y)*sin(x)*sin(z), cos(x)*cos(y)*sin(z),
                  -cos(y),       -sin(x)*sin(y),       -cos(x)*sin(y);

  dR_dz << -cos(y)*sin(z), - cos(x)*cos(z) - sin(x)*sin(y)*sin(z), cos(z)*sin(x) - cos(x)*sin(y)*sin(z),
            cos(y)*cos(z),   cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
                        0,                                      0,                                    0;

  Eigen::Matrix<SCALAR_T, 3, 3> output;
  output.setZero();
  ///< if we want the derivative of R*vec that goes from body(base) to world
  output.col(0) = dR_dz*right_vec;
  output.col(1) = dR_dy*right_vec;
  output.col(2) = dR_dx*right_vec;

  ///< if we want the derivative of R*vec that goes from world to body(base)
  // output.col(0) = dR_dz.transpose()*right_vec;
  // output.col(1) = dR_dy.transpose()*right_vec;
  // output.col(2) = dR_dx.transpose()*right_vec;

  return output;
}

/**
 * Compute the derivative of rotation matrix transpose right multiplied by a vector w.r.t. ZYX euler angles variables
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] right_vec: the vector being right multiplied
 * @return 3x3 matrix jacobian
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotMatrixTransposeJacobianWrtZYXEulerRightMultiplyVec(const Eigen::Matrix<SCALAR_T, 3, 1>& euler_angle,
                                                                              const Eigen::Matrix<SCALAR_T, 3, 1>& right_vec) {
  SCALAR_T x = euler_angle[2];
  SCALAR_T y = euler_angle[1];
  SCALAR_T z = euler_angle[0];

  // R is the rotation matrix from body(base) to world
  Eigen::Matrix<SCALAR_T, 3, 3> dR_dx, dR_dy, dR_dz;

  dR_dx <<  0, sin(x)*sin(z) + cos(x)*cos(z)*sin(y),   cos(x)*sin(z) - cos(z)*sin(x)*sin(y),
            0, cos(x)*sin(y)*sin(z) - cos(z)*sin(x), - cos(x)*cos(z) - sin(x)*sin(y)*sin(z),
            0,                        cos(x)*cos(y),                         -cos(y)*sin(x);

  dR_dy << -cos(z)*sin(y), cos(y)*cos(z)*sin(x), cos(x)*cos(y)*cos(z),
            -sin(y)*sin(z), cos(y)*sin(x)*sin(z), cos(x)*cos(y)*sin(z),
                  -cos(y),       -sin(x)*sin(y),       -cos(x)*sin(y);

  dR_dz << -cos(y)*sin(z), - cos(x)*cos(z) - sin(x)*sin(y)*sin(z), cos(z)*sin(x) - cos(x)*sin(y)*sin(z),
            cos(y)*cos(z),   cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
                        0,                                      0,                                    0;

  Eigen::Matrix<SCALAR_T, 3, 3> output;
  output.setZero();

  ///< if we want the derivative of R*vec that goes from body(base) to world
//        output.col(0) = dR_dx*right_vec;
//        output.col(1) = dR_dy*right_vec;
//        output.col(2) = dR_dz*right_vec;

  ///< if we want the derivative of R*vec that goes from world to body(base)
  output.col(0) = dR_dz.transpose()*right_vec;
  output.col(1) = dR_dy.transpose()*right_vec;
  output.col(2) = dR_dx.transpose()*right_vec;
  return output;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> RotM2ZYXEuler(const Eigen::Matrix<SCALAR_T, 3, 3>&  R) 
{
  double sy = sqrt(R(0, 0)*R(0, 0) + R(1, 0)*R(1, 0));

  bool singular = sy < 1e-6;

  double x, y, z;
  if (! singular) {
    x = atan2(R(2, 1), R(2, 2));
    y = atan2(-R(2, 0), sy);
    z = atan2(R(1, 0), R(0, 0));
  } else {
    x = atan2(-R(1, 2), R(1, 1));
    y = atan2(-R(2, 0), sy);
    z = 0;
  }

  Eigen::Matrix<SCALAR_T, 3, 1> eulerZYX = Eigen::Matrix<SCALAR_T, 3, 1>::Zero();
  eulerZYX << z, y, x;

  return eulerZYX;
}

} // namespace multi_robot
} // namespace ocs2