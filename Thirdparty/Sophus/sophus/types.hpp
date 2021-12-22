/// @file
/// Common type aliases.

#ifndef SOPHUS_TYPES_HPP
#define SOPHUS_TYPES_HPP

#include <type_traits>
#include "common.hpp"

namespace Sophus {

template <class Scalar, int M, int Options = 0>
using Vector = Eigen::Matrix<Scalar, M, 1, Options>;

template <class Scalar, int Options = 0>
using Vector2 = Vector<Scalar, 2, Options>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;

template <class Scalar, int Options = 0>
using Vector3 = Vector<Scalar, 3, Options>;
using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;

template <class Scalar>
using Vector4 = Vector<Scalar, 4>;
using Vector4f = Vector4<float>;
using Vector4d = Vector4<double>;

template <class Scalar>
using Vector6 = Vector<Scalar, 6>;
using Vector6f = Vector6<float>;
using Vector6d = Vector6<double>;

template <class Scalar>
using Vector7 = Vector<Scalar, 7>;
using Vector7f = Vector7<float>;
using Vector7d = Vector7<double>;

template <class Scalar, int M, int N>
using Matrix = Eigen::Matrix<Scalar, M, N>;

template <class Scalar>
using Matrix2 = Matrix<Scalar, 2, 2>;
using Matrix2f = Matrix2<float>;
using Matrix2d = Matrix2<double>;

template <class Scalar>
using Matrix3 = Matrix<Scalar, 3, 3>;
using Matrix3f = Matrix3<float>;
using Matrix3d = Matrix3<double>;

template <class Scalar>
using Matrix4 = Matrix<Scalar, 4, 4>;
using Matrix4f = Matrix4<float>;
using Matrix4d = Matrix4<double>;

template <class Scalar>
using Matrix6 = Matrix<Scalar, 6, 6>;
using Matrix6f = Matrix6<float>;
using Matrix6d = Matrix6<double>;

template <class Scalar>
using Matrix7 = Matrix<Scalar, 7, 7>;
using Matrix7f = Matrix7<float>;
using Matrix7d = Matrix7<double>;

template <class Scalar, int N, int Options = 0>
using ParametrizedLine = Eigen::ParametrizedLine<Scalar, N, Options>;

template <class Scalar, int Options = 0>
using ParametrizedLine3 = ParametrizedLine<Scalar, 3, Options>;
using ParametrizedLine3f = ParametrizedLine3<float>;
using ParametrizedLine3d = ParametrizedLine3<double>;

template <class Scalar, int Options = 0>
using ParametrizedLine2 = ParametrizedLine<Scalar, 2, Options>;
using ParametrizedLine2f = ParametrizedLine2<float>;
using ParametrizedLine2d = ParametrizedLine2<double>;

namespace details {
template <class Scalar>
class MaxMetric {
 public:
  static Scalar impl(Scalar s0, Scalar s1) {
    using std::abs;
    return abs(s0 - s1);
  }
};

template <class Scalar, int M, int N>
class MaxMetric<Matrix<Scalar, M, N>> {
 public:
  static Scalar impl(Matrix<Scalar, M, N> const& p0,
                     Matrix<Scalar, M, N> const& p1) {
    return (p0 - p1).template lpNorm<Eigen::Infinity>();
  }
};

template <class Scalar>
class SetToZero {
 public:
  static void impl(Scalar& s) { s = Scalar(0); }
};

template <class Scalar, int M, int N>
class SetToZero<Matrix<Scalar, M, N>> {
 public:
  static void impl(Matrix<Scalar, M, N>& v) { v.setZero(); }
};

template <class T1, class Scalar>
class SetElementAt;

template <class Scalar>
class SetElementAt<Scalar, Scalar> {
 public:
  static void impl(Scalar& s, Scalar value, int at) {
    SOPHUS_ENSURE(at == 0, "is %", at);
    s = value;
  }
};

template <class Scalar, int N>
class SetElementAt<Vector<Scalar, N>, Scalar> {
 public:
  static void impl(Vector<Scalar, N>& v, Scalar value, int at) {
    SOPHUS_ENSURE(at >= 0 && at < N, "is %", at);
    v[at] = value;
  }
};

template <class Scalar>
class SquaredNorm {
 public:
  static Scalar impl(Scalar const& s) { return s * s; }
};

template <class Scalar, int N>
class SquaredNorm<Matrix<Scalar, N, 1>> {
 public:
  static Scalar impl(Matrix<Scalar, N, 1> const& s) { return s.squaredNorm(); }
};

template <class Scalar>
class Transpose {
 public:
  static Scalar impl(Scalar const& s) { return s; }
};

template <class Scalar, int M, int N>
class Transpose<Matrix<Scalar, M, N>> {
 public:
  static Matrix<Scalar, M, N> impl(Matrix<Scalar, M, N> const& s) {
    return s.transpose();
  }
};
}  // namespace details

/// Returns maximum metric between two points ``p0`` and ``p1``, with ``p0, p1``
/// being matrices or a scalars.
///
template <class T>
auto maxMetric(T const& p0, T const& p1)
    -> decltype(details::MaxMetric<T>::impl(p0, p1)) {
  return details::MaxMetric<T>::impl(p0, p1);
}

/// Sets point ``p`` to zero, with ``p`` being a matrix or a scalar.
///
template <class T>
void setToZero(T& p) {
  return details::SetToZero<T>::impl(p);
}

/// Sets ``i``th component of ``p`` to ``value``, with ``p`` being a
/// matrix or a scalar. If ``p`` is a scalar, ``i`` must be ``0``.
///
template <class T, class Scalar>
void setElementAt(T& p, Scalar value, int i) {
  return details::SetElementAt<T, Scalar>::impl(p, value, i);
}

/// Returns the squared 2-norm of ``p``, with ``p`` being a vector or a scalar.
///
template <class T>
auto squaredNorm(T const& p) -> decltype(details::SquaredNorm<T>::impl(p)) {
  return details::SquaredNorm<T>::impl(p);
}

/// Returns ``p.transpose()`` if ``p`` is a matrix, and simply ``p`` if m is a
/// scalar.
///
template <class T>
auto transpose(T const& p) -> decltype(details::Transpose<T>::impl(T())) {
  return details::Transpose<T>::impl(p);
}

template <class Scalar>
struct IsFloatingPoint {
  static bool const value = std::is_floating_point<Scalar>::value;
};

template <class Scalar, int M, int N>
struct IsFloatingPoint<Matrix<Scalar, M, N>> {
  static bool const value = std::is_floating_point<Scalar>::value;
};

template <class Scalar_>
struct GetScalar {
  using Scalar = Scalar_;
};

template <class Scalar_, int M, int N>
struct GetScalar<Matrix<Scalar_, M, N>> {
  using Scalar = Scalar_;
};

/// If the Vector type is of fixed size, then IsFixedSizeVector::value will be
/// true.
template <typename Vector, int NumDimensions,
          typename = typename std::enable_if<
              Vector::RowsAtCompileTime == NumDimensions &&
              Vector::ColsAtCompileTime == 1>::type>
struct IsFixedSizeVector : std::true_type {};

/// Planes in 3d are hyperplanes.
template <class T>
using Plane3 = Eigen::Hyperplane<T, 3>;
using Plane3d = Plane3<double>;
using Plane3f = Plane3<float>;

/// Lines in 2d are hyperplanes.
template <class T>
using Line2 = Eigen::Hyperplane<T, 2>;
using Line2d = Line2<double>;
using Line2f = Line2<float>;

}  // namespace Sophus

#endif  // SOPHUS_TYPES_HPP
