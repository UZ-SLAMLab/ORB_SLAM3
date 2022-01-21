#ifndef SOPHUS_SIM_DETAILS_HPP
#define SOPHUS_SIM_DETAILS_HPP

#include "types.hpp"

namespace Sophus {
namespace details {

template <class Scalar, int N>
Matrix<Scalar, N, N> calcW(Matrix<Scalar, N, N> const& Omega,
                           Scalar const theta, Scalar const sigma) {
  using std::abs;
  using std::cos;
  using std::exp;
  using std::sin;
  static Matrix<Scalar, N, N> const I = Matrix<Scalar, N, N>::Identity();
  static Scalar const one(1);
  static Scalar const half(0.5);
  Matrix<Scalar, N, N> const Omega2 = Omega * Omega;
  Scalar const scale = exp(sigma);
  Scalar A, B, C;
  if (abs(sigma) < Constants<Scalar>::epsilon()) {
    C = one;
    if (abs(theta) < Constants<Scalar>::epsilon()) {
      A = half;
      B = Scalar(1. / 6.);
    } else {
      Scalar theta_sq = theta * theta;
      A = (one - cos(theta)) / theta_sq;
      B = (theta - sin(theta)) / (theta_sq * theta);
    }
  } else {
    C = (scale - one) / sigma;
    if (abs(theta) < Constants<Scalar>::epsilon()) {
      Scalar sigma_sq = sigma * sigma;
      A = ((sigma - one) * scale + one) / sigma_sq;
      B = (scale * half * sigma_sq + scale - one - sigma * scale) /
          (sigma_sq * sigma);
    } else {
      Scalar theta_sq = theta * theta;
      Scalar a = scale * sin(theta);
      Scalar b = scale * cos(theta);
      Scalar c = theta_sq + sigma * sigma;
      A = (a * sigma + (one - b) * theta) / (theta * c);
      B = (C - ((b - one) * sigma + a * theta) / (c)) * one / (theta_sq);
    }
  }
  return A * Omega + B * Omega2 + C * I;
}

template <class Scalar, int N>
Matrix<Scalar, N, N> calcWInv(Matrix<Scalar, N, N> const& Omega,
                              Scalar const theta, Scalar const sigma,
                              Scalar const scale) {
  using std::abs;
  using std::cos;
  using std::sin;
  static Matrix<Scalar, N, N> const I = Matrix<Scalar, N, N>::Identity();
  static Scalar const half(0.5);
  static Scalar const one(1);
  static Scalar const two(2);
  Matrix<Scalar, N, N> const Omega2 = Omega * Omega;
  Scalar const scale_sq = scale * scale;
  Scalar const theta_sq = theta * theta;
  Scalar const sin_theta = sin(theta);
  Scalar const cos_theta = cos(theta);

  Scalar a, b, c;
  if (abs(sigma * sigma) < Constants<Scalar>::epsilon()) {
    c = one - half * sigma;
    a = -half;
    if (abs(theta_sq) < Constants<Scalar>::epsilon()) {
      b = Scalar(1. / 12.);
    } else {
      b = (theta * sin_theta + two * cos_theta - two) /
          (two * theta_sq * (cos_theta - one));
    }
  } else {
    Scalar const scale_cu = scale_sq * scale;
    c = sigma / (scale - one);
    if (abs(theta_sq) < Constants<Scalar>::epsilon()) {
      a = (-sigma * scale + scale - one) / ((scale - one) * (scale - one));
      b = (scale_sq * sigma - two * scale_sq + scale * sigma + two * scale) /
          (two * scale_cu - Scalar(6) * scale_sq + Scalar(6) * scale - two);
    } else {
      Scalar const s_sin_theta = scale * sin_theta;
      Scalar const s_cos_theta = scale * cos_theta;
      a = (theta * s_cos_theta - theta - sigma * s_sin_theta) /
          (theta * (scale_sq - two * s_cos_theta + one));
      b = -scale *
          (theta * s_sin_theta - theta * sin_theta + sigma * s_cos_theta -
           scale * sigma + sigma * cos_theta - sigma) /
          (theta_sq * (scale_cu - two * scale * s_cos_theta - scale_sq +
                       two * s_cos_theta + scale - one));
    }
  }
  return a * Omega + b * Omega2 + c * I;
}

}  // namespace details
}  // namespace Sophus

#endif
