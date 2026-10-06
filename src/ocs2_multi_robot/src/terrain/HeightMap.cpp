#include "ocs2_multi_robot/terrain/HeightMap.h"

#include <cmath>

namespace ocs2 {
namespace multi_robot {


double HeightMap::GetHeight(const Vector3d& pos) const {
  return GetHeight(pos[dim2X], pos[dim2Y]);
}

double HeightMap::GetDerivativeOfHeightWrt(Dim2D dim, double x, double y) const {
  switch (dim) {
      case dim2X:
          return GetHeightDerivWrtX(x, y);
      case dim2Y:
          return GetHeightDerivWrtY(x, y);
      default:
          assert(false && "Incorrect dimension");
          return 0.0;
  }
}

double HeightMap::GetDerivativeOfHeightWrt(Dim2D dim, const Vector3d& pos) const {
  return GetDerivativeOfHeightWrt(dim, pos[dim2X], pos[dim2Y]);
}

double HeightMap::GetFrictionCoeff() const {
  return friction_coeff_;
}

HeightMap::Vector3d HeightMap::GetNormalizedBasis(Direction basis, double x,
                                                  double y) const {
  return GetBasis(basis, x, y).normalized();
}

HeightMap::Vector3d HeightMap::GetNormalizedBasis(Direction direction,
                                                  const Vector3d& pos) const {
  return GetNormalizedBasis(direction, pos[dim2X], pos[dim2Y]);
}

HeightMap::Vector3d HeightMap::GetBasis(Direction basis, double x, double y,
                                        const DimDerivs& deriv) const {
  switch (basis) {
      case Normal:
          return GetNormal(x, y, deriv);
      case Tangent1:
          return GetTangent1(x, y, deriv);
      case Tangent2:
          return GetTangent2(x, y, deriv);
      default:
          assert(false); // basis does not exist
          return Vector3d(0.0, 0.0, 1.0);
  }
}

HeightMap::Vector3d HeightMap::GetDerivativeOfNormalizedBasisWrt(
        Direction basis, Dim2D dim, double x, double y) const {
  // inner derivative
  Vector3d dv_wrt_dim = GetBasis(basis, x, y, {dim});

  // outer derivative
  Vector3d v = GetBasis(basis, x, y, {});
  Vector3d dn_norm_wrt_n =
          GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(v, dim);
  return dn_norm_wrt_n.cwiseProduct(dv_wrt_dim);
}

HeightMap::Vector3d HeightMap::GetDerivativeOfNormalizedBasisWrt(
        Direction direction, Dim2D dim, const Vector3d& pos) const {
  return GetDerivativeOfNormalizedBasisWrt(direction, dim, pos[dim2X], pos[dim2Y]);
}

HeightMap::MatrixXd HeightMap::GetDerivativeOfBasis(Direction direction,
                                                    const Vector3d& pos) const {
  MatrixXd dv_dx(3, 3);

  // Derivative w.r.t. z is zero
  for (auto dim : {dim2X, dim2Y}) {
      Vector3d dvec = GetBasis(direction, pos[dim2X], pos[dim2Y], {dim});

      for (auto row : {dim3X, dim3Y, dim3Z}) {
          dv_dx(row, dim) = dvec[row];
      }
  }

  return dv_dx;
}

HeightMap::MatrixXd HeightMap::GetDerivativeOfNormalizedBasis(
        Direction direction, const Vector3d& pos) const {
  Vector3d v = GetBasis(direction, pos[dim2X], pos[dim2Y]);

  MatrixXd dvnorm_dv = GetDerivativeOfNormalizedVectorWrtNonNormalized(v);
  MatrixXd dv_dx = GetDerivativeOfBasis(direction, pos);

  MatrixXd dvnorm_dx = dvnorm_dv * dv_dx;

  return dvnorm_dx;
}

HeightMap::Vector3d HeightMap::GetNormal(double x, double y,
                                          const DimDerivs& deriv) const {
  Vector3d n;

  bool basis_requested = deriv.empty();

  for (auto dim : {dim2X, dim2Y}) {
      if (basis_requested)
          n(dim) = -GetDerivativeOfHeightWrt(dim, x, y);
      else
          n(dim) = -GetSecondDerivativeOfHeightWrt(dim, deriv.front(), x, y);
  }

  n(dim3Z) = basis_requested ? 1.0 : 0.0;

  return n;
}

HeightMap::Vector3d HeightMap::GetTangent1(double x, double y,
                                            const DimDerivs& deriv) const {
  Vector3d tx;

  bool basis_requested = deriv.empty();

  tx(dim3X) = basis_requested ? 1.0 : 0.0;
  tx(dim3Y) = 0.0;
  tx(dim3Z) = basis_requested ?
          GetDerivativeOfHeightWrt(dim2X, x, y) :
          GetSecondDerivativeOfHeightWrt(dim2X, deriv.front(), x, y);

  return tx;
}

HeightMap::Vector3d HeightMap::GetTangent2(double x, double y,
                                            const DimDerivs& deriv) const {
  Vector3d ty;

  bool basis_requested = deriv.empty();

  ty(dim3X) = 0.0;
  ty(dim3Y) = basis_requested ? 1.0 : 0.0;
  ty(dim3Z) = basis_requested ?
          GetDerivativeOfHeightWrt(dim2Y, x, y) :
          GetSecondDerivativeOfHeightWrt(dim2Y, deriv.front(), x, y);
  return ty;
}

HeightMap::Vector3d HeightMap::GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(
        const Vector3d& v, int idx) const {
  // see notebook or http://blog.mmacklin.com/2012/05/04/implicitsprings/
  return 1 / v.squaredNorm()
          * (v.norm() * Vector3d::Unit(idx) - v(idx) * v.normalized());
}

HeightMap::MatrixXd HeightMap::GetDerivativeOfNormalizedVectorWrtNonNormalized(
        const Vector3d& v) const {
  // see notebook or http://blog.mmacklin.com/2012/05/04/implicitsprings/
  //return 1 / v.squaredNorm() * (v.norm() * Vector3d::Unit(idx) - v(idx) * v.normalized());

  Vector3d vnorm = v.normalized();
  MatrixXd jac_dense = (MatrixXd::Identity(3, 3) - vnorm * vnorm.transpose())
                        * 1 / v.norm();

  return jac_dense;
}

double HeightMap::GetSecondDerivativeOfHeightWrt(Dim2D dim1, Dim2D dim2,
                                                  double x, double y) const {
  if (dim1 == dim2X) {
      if (dim2 == dim2X)
          return GetHeightDerivWrtXX(x, y);
      if (dim2 == dim2Y)
          return GetHeightDerivWrtXY(x, y);
  } else {
      if (dim2 == dim2X)
          return GetHeightDerivWrtYX(x, y);
      if (dim2 == dim2Y)
          return GetHeightDerivWrtYY(x, y);
  }

  assert(false); // second derivative not specified.
  return 0.0;
}

// first derivatives that must be implemented by the user
double HeightMap::GetHeightDerivWrtX(double, double) const {
  return 0.0;
}

double HeightMap::GetHeightDerivWrtY(double, double) const {
  return 0.0;
}

// second derivatives with respect to first letter, then second
double HeightMap::GetHeightDerivWrtXX(double, double) const {
  return 0.0;
}

double HeightMap::GetHeightDerivWrtXY(double, double) const {
  return 0.0;
}

double HeightMap::GetHeightDerivWrtYX(double, double) const {
  return 0.0;
}

double HeightMap::GetHeightDerivWrtYY(double, double) const {
  return 0.0;
}

} // namespace multi_robot
} // namespace ocs2