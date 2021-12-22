/// @file
/// Rotation matrix helper functions.

#ifndef SOPHUS_ROTATION_MATRIX_HPP
#define SOPHUS_ROTATION_MATRIX_HPP

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "types.hpp"

namespace Sophus {

/// Takes in arbitrary square matrix and returns true if it is
/// orthogonal.
template <class D>
SOPHUS_FUNC bool isOrthogonal(Eigen::MatrixBase<D> const& R) {
  using Scalar = typename D::Scalar;
  static int const N = D::RowsAtCompileTime;
  static int const M = D::ColsAtCompileTime;

  static_assert(N == M, "must be a square matrix");
  static_assert(N >= 2, "must have compile time dimension >= 2");

  return (R * R.transpose() - Matrix<Scalar, N, N>::Identity()).norm() <
         Constants<Scalar>::epsilon();
}

/// Takes in arbitrary square matrix and returns true if it is
/// "scaled-orthogonal" with positive determinant.
///
template <class D>
SOPHUS_FUNC bool isScaledOrthogonalAndPositive(Eigen::MatrixBase<D> const& sR) {
  using Scalar = typename D::Scalar;
  static int const N = D::RowsAtCompileTime;
  static int const M = D::ColsAtCompileTime;
  using std::pow;
  using std::sqrt;

  Scalar det = sR.determinant();

  if (det <= Scalar(0)) {
    return false;
  }

  Scalar scale_sqr = pow(det, Scalar(2. / N));

  static_assert(N == M, "must be a square matrix");
  static_assert(N >= 2, "must have compile time dimension >= 2");

  return (sR * sR.transpose() - scale_sqr * Matrix<Scalar, N, N>::Identity())
             .template lpNorm<Eigen::Infinity>() <
         sqrt(Constants<Scalar>::epsilon());
}

/// Takes in arbitrary square matrix (2x2 or larger) and returns closest
/// orthogonal matrix with positive determinant.
template <class D>
SOPHUS_FUNC enable_if_t<
    std::is_floating_point<typename D::Scalar>::value,
    Matrix<typename D::Scalar, D::RowsAtCompileTime, D::RowsAtCompileTime>>
makeRotationMatrix(Eigen::MatrixBase<D> const& R) {
  using Scalar = typename D::Scalar;
  static int const N = D::RowsAtCompileTime;
  static int const M = D::ColsAtCompileTime;

  static_assert(N == M, "must be a square matrix");
  static_assert(N >= 2, "must have compile time dimension >= 2");

  Eigen::JacobiSVD<Matrix<Scalar, N, N>> svd(
      R, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Determine determinant of orthogonal matrix U*V'.
  Scalar d = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  // Starting from the identity matrix D, set the last entry to d (+1 or
  // -1),  so that det(U*D*V') = 1.
  Matrix<Scalar, N, N> Diag = Matrix<Scalar, N, N>::Identity();
  Diag(N - 1, N - 1) = d;
  return svd.matrixU() * Diag * svd.matrixV().transpose();
}

}  // namespace Sophus

#endif  // SOPHUS_ROTATION_MATRIX_HPP
