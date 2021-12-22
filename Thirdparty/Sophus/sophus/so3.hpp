/// @file
/// Special orthogonal group SO(3) - rotation in 3d.

#ifndef SOPHUS_SO3_HPP
#define SOPHUS_SO3_HPP

#include "rotation_matrix.hpp"
#include "so2.hpp"
#include "types.hpp"

// Include only the selective set of Eigen headers that we need.
// This helps when using Sophus with unusual compilers, like nvcc.
#include <Eigen/src/Geometry/OrthoMethods.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/RotationBase.h>

namespace Sophus {
template <class Scalar_, int Options = 0>
class SO3;
using SO3d = SO3<double>;
using SO3f = SO3<float>;
}  // namespace Sophus

namespace Eigen {
namespace internal {

template <class Scalar_, int Options_>
struct traits<Sophus::SO3<Scalar_, Options_>> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Eigen::Quaternion<Scalar, Options>;
};

template <class Scalar_, int Options_>
struct traits<Map<Sophus::SO3<Scalar_>, Options_>>
    : traits<Sophus::SO3<Scalar_, Options_>> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Map<Eigen::Quaternion<Scalar>, Options>;
};

template <class Scalar_, int Options_>
struct traits<Map<Sophus::SO3<Scalar_> const, Options_>>
    : traits<Sophus::SO3<Scalar_, Options_> const> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Map<Eigen::Quaternion<Scalar> const, Options>;
};
}  // namespace internal
}  // namespace Eigen

namespace Sophus {

/// SO3 base type - implements SO3 class but is storage agnostic.
///
/// SO(3) is the group of rotations in 3d. As a matrix group, it is the set of
/// matrices which are orthogonal such that ``R * R' = I`` (with ``R'`` being
/// the transpose of ``R``) and have a positive determinant. In particular, the
/// determinant is 1. Internally, the group is represented as a unit quaternion.
/// Unit quaternion can be seen as members of the special unitary group SU(2).
/// SU(2) is a double cover of SO(3). Hence, for every rotation matrix ``R``,
/// there exist two unit quaternions: ``(r, v)`` and ``(-r, -v)``, with ``r``
/// the real part and ``v`` being the imaginary 3-vector part of the quaternion.
///
/// SO(3) is a compact, but non-commutative group. First it is compact since the
/// set of rotation matrices is a closed and bounded set. Second it is
/// non-commutative since the equation ``R_1 * R_2 = R_2 * R_1`` does not hold
/// in general. For example rotating an object by some degrees about its
/// ``x``-axis and then by some degrees about its y axis, does not lead to the
/// same orientation when rotation first about ``y`` and then about ``x``.
///
/// Class invariant: The 2-norm of ``unit_quaternion`` must be close to 1.
/// Technically speaking, it must hold that:
///
///   ``|unit_quaternion().squaredNorm() - 1| <= Constants::epsilon()``.
template <class Derived>
class SO3Base {
 public:
  static constexpr int Options = Eigen::internal::traits<Derived>::Options;
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using QuaternionType =
      typename Eigen::internal::traits<Derived>::QuaternionType;
  using QuaternionTemporaryType = Eigen::Quaternion<Scalar, Options>;

  /// Degrees of freedom of group, number of dimensions in tangent space.
  static int constexpr DoF = 3;
  /// Number of internal parameters used (quaternion is a 4-tuple).
  static int constexpr num_parameters = 4;
  /// Group transformations are 3x3 matrices.
  static int constexpr N = 3;
  using Transformation = Matrix<Scalar, N, N>;
  using Point = Vector3<Scalar>;
  using HomogeneousPoint = Vector4<Scalar>;
  using Line = ParametrizedLine3<Scalar>;
  using Tangent = Vector<Scalar, DoF>;
  using Adjoint = Matrix<Scalar, DoF, DoF>;

  struct TangentAndTheta {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Tangent tangent;
    Scalar theta;
  };

  /// For binary operations the return type is determined with the
  /// ScalarBinaryOpTraits feature of Eigen. This allows mixing concrete and Map
  /// types, as well as other compatible scalar types such as Ceres::Jet and
  /// double scalars with SO3 operations.
  template <typename OtherDerived>
  using ReturnScalar = typename Eigen::ScalarBinaryOpTraits<
      Scalar, typename OtherDerived::Scalar>::ReturnType;

  template <typename OtherDerived>
  using SO3Product = SO3<ReturnScalar<OtherDerived>>;

  template <typename PointDerived>
  using PointProduct = Vector3<ReturnScalar<PointDerived>>;

  template <typename HPointDerived>
  using HomogeneousPointProduct = Vector4<ReturnScalar<HPointDerived>>;

  /// Adjoint transformation
  //
  /// This function return the adjoint transformation ``Ad`` of the group
  /// element ``A`` such that for all ``x`` it holds that
  /// ``hat(Ad_A * x) = A * hat(x) A^{-1}``. See hat-operator below.
  //
  /// For SO(3), it simply returns the rotation matrix corresponding to ``A``.
  ///
  SOPHUS_FUNC Adjoint Adj() const { return matrix(); }

  /// Extract rotation angle about canonical X-axis
  ///
  template <class S = Scalar>
  SOPHUS_FUNC enable_if_t<std::is_floating_point<S>::value, S> angleX() const {
    Sophus::Matrix3<Scalar> R = matrix();
    Sophus::Matrix2<Scalar> Rx = R.template block<2, 2>(1, 1);
    return SO2<Scalar>(makeRotationMatrix(Rx)).log();
  }

  /// Extract rotation angle about canonical Y-axis
  ///
  template <class S = Scalar>
  SOPHUS_FUNC enable_if_t<std::is_floating_point<S>::value, S> angleY() const {
    Sophus::Matrix3<Scalar> R = matrix();
    Sophus::Matrix2<Scalar> Ry;
    // clang-format off
    Ry <<
      R(0, 0), R(2, 0),
      R(0, 2), R(2, 2);
    // clang-format on
    return SO2<Scalar>(makeRotationMatrix(Ry)).log();
  }

  /// Extract rotation angle about canonical Z-axis
  ///
  template <class S = Scalar>
  SOPHUS_FUNC enable_if_t<std::is_floating_point<S>::value, S> angleZ() const {
    Sophus::Matrix3<Scalar> R = matrix();
    Sophus::Matrix2<Scalar> Rz = R.template block<2, 2>(0, 0);
    return SO2<Scalar>(makeRotationMatrix(Rz)).log();
  }

  /// Returns copy of instance casted to NewScalarType.
  ///
  template <class NewScalarType>
  SOPHUS_FUNC SO3<NewScalarType> cast() const {
    return SO3<NewScalarType>(unit_quaternion().template cast<NewScalarType>());
  }

  /// This provides unsafe read/write access to internal data. SO(3) is
  /// represented by an Eigen::Quaternion (four parameters). When using direct
  /// write access, the user needs to take care of that the quaternion stays
  /// normalized.
  ///
  /// Note: The first three Scalars represent the imaginary parts, while the
  /// forth Scalar represent the real part.
  ///
  SOPHUS_FUNC Scalar* data() {
    return unit_quaternion_nonconst().coeffs().data();
  }

  /// Const version of data() above.
  ///
  SOPHUS_FUNC Scalar const* data() const {
    return unit_quaternion().coeffs().data();
  }

  /// Returns derivative of  this * SO3::exp(x)  wrt. x at x=0.
  ///
  SOPHUS_FUNC Matrix<Scalar, num_parameters, DoF> Dx_this_mul_exp_x_at_0()
      const {
    Matrix<Scalar, num_parameters, DoF> J;
    Eigen::Quaternion<Scalar> const q = unit_quaternion();
    Scalar const c0 = Scalar(0.5) * q.w();
    Scalar const c1 = Scalar(0.5) * q.z();
    Scalar const c2 = -c1;
    Scalar const c3 = Scalar(0.5) * q.y();
    Scalar const c4 = Scalar(0.5) * q.x();
    Scalar const c5 = -c4;
    Scalar const c6 = -c3;
    J(0, 0) = c0;
    J(0, 1) = c2;
    J(0, 2) = c3;
    J(1, 0) = c1;
    J(1, 1) = c0;
    J(1, 2) = c5;
    J(2, 0) = c6;
    J(2, 1) = c4;
    J(2, 2) = c0;
    J(3, 0) = c5;
    J(3, 1) = c6;
    J(3, 2) = c2;

    return J;
  }

  /// Returns internal parameters of SO(3).
  ///
  /// It returns (q.imag[0], q.imag[1], q.imag[2], q.real), with q being the
  /// unit quaternion.
  ///
  SOPHUS_FUNC Sophus::Vector<Scalar, num_parameters> params() const {
    return unit_quaternion().coeffs();
  }

  /// Returns group inverse.
  ///
  SOPHUS_FUNC SO3<Scalar> inverse() const {
    return SO3<Scalar>(unit_quaternion().conjugate());
  }

  /// Logarithmic map
  ///
  /// Computes the logarithm, the inverse of the group exponential which maps
  /// element of the group (rotation matrices) to elements of the tangent space
  /// (rotation-vector).
  ///
  /// To be specific, this function computes ``vee(logmat(.))`` with
  /// ``logmat(.)`` being the matrix logarithm and ``vee(.)`` the vee-operator
  /// of SO(3).
  ///
  SOPHUS_FUNC Tangent log() const { return logAndTheta().tangent; }

  /// As above, but also returns ``theta = |omega|``.
  ///
  SOPHUS_FUNC TangentAndTheta logAndTheta() const {
    TangentAndTheta J;
    using std::abs;
    using std::atan;
    using std::sqrt;
    Scalar squared_n = unit_quaternion().vec().squaredNorm();
    Scalar w = unit_quaternion().w();

    Scalar two_atan_nbyw_by_n;

    /// Atan-based log thanks to
    ///
    /// C. Hertzberg et al.:
    /// "Integrating Generic Sensor Fusion Algorithms with Sound State
    /// Representation through Encapsulation of Manifolds"
    /// Information Fusion, 2011

    if (squared_n < Constants<Scalar>::epsilon() * Constants<Scalar>::epsilon()) {
      // If quaternion is normalized and n=0, then w should be 1;
      // w=0 should never happen here!
      SOPHUS_ENSURE(abs(w) >= Constants<Scalar>::epsilon(),
                    "Quaternion (%) should be normalized!",
                    unit_quaternion().coeffs().transpose());
      Scalar squared_w = w * w;
      two_atan_nbyw_by_n =
          Scalar(2) / w - Scalar(2.0/3.0) * (squared_n) / (w * squared_w);
      J.theta = Scalar(2) * squared_n / w;
    } else {
      Scalar n = sqrt(squared_n);
      if (abs(w) < Constants<Scalar>::epsilon()) {
        if (w > Scalar(0)) {
          two_atan_nbyw_by_n = Constants<Scalar>::pi() / n;
        } else {
          two_atan_nbyw_by_n = -Constants<Scalar>::pi() / n;
        }
      } else {
        two_atan_nbyw_by_n = Scalar(2) * atan(n / w) / n;
      }
      J.theta = two_atan_nbyw_by_n * n;
    }

    J.tangent = two_atan_nbyw_by_n * unit_quaternion().vec();
    return J;
  }

  /// It re-normalizes ``unit_quaternion`` to unit length.
  ///
  /// Note: Because of the class invariant, there is typically no need to call
  /// this function directly.
  ///
  SOPHUS_FUNC void normalize() {
    Scalar length = unit_quaternion_nonconst().norm();
    SOPHUS_ENSURE(length >= Constants<Scalar>::epsilon(),
                  "Quaternion (%) should not be close to zero!",
                  unit_quaternion_nonconst().coeffs().transpose());
    unit_quaternion_nonconst().coeffs() /= length;
  }

  /// Returns 3x3 matrix representation of the instance.
  ///
  /// For SO(3), the matrix representation is an orthogonal matrix ``R`` with
  /// ``det(R)=1``, thus the so-called "rotation matrix".
  ///
  SOPHUS_FUNC Transformation matrix() const {
    return unit_quaternion().toRotationMatrix();
  }

  /// Assignment-like operator from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC SO3Base<Derived>& operator=(SO3Base<OtherDerived> const& other) {
    unit_quaternion_nonconst() = other.unit_quaternion();
    return *this;
  }

  /// Group multiplication, which is rotation concatenation.
  ///
  template <typename OtherDerived>
  SOPHUS_FUNC SO3Product<OtherDerived> operator*(
      SO3Base<OtherDerived> const& other) const {
    using QuaternionProductType =
        typename SO3Product<OtherDerived>::QuaternionType;
    const QuaternionType& a = unit_quaternion();
    const typename OtherDerived::QuaternionType& b = other.unit_quaternion();
    /// NOTE: We cannot use Eigen's Quaternion multiplication because it always
    /// returns a Quaternion of the same Scalar as this object, so it is not
    /// able to multiple Jets and doubles correctly.
    return SO3Product<OtherDerived>(QuaternionProductType(
        a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
        a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
        a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
        a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x()));
  }

  /// Group action on 3-points.
  ///
  /// This function rotates a 3 dimensional point ``p`` by the SO3 element
  ///  ``bar_R_foo`` (= rotation matrix): ``p_bar = bar_R_foo * p_foo``.
  ///
  /// Since SO3 is internally represented by a unit quaternion ``q``, it is
  /// implemented as ``p_bar = q * p_foo * q^{*}``
  /// with ``q^{*}`` being the quaternion conjugate of ``q``.
  ///
  /// Geometrically, ``p``  is rotated by angle ``|omega|`` around the
  /// axis ``omega/|omega|`` with ``omega := vee(log(bar_R_foo))``.
  ///
  /// For ``vee``-operator, see below.
  ///
  template <typename PointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<PointDerived, 3>::value>::type>
  SOPHUS_FUNC PointProduct<PointDerived> operator*(
      Eigen::MatrixBase<PointDerived> const& p) const {
    /// NOTE: We cannot use Eigen's Quaternion transformVector because it always
    /// returns a Vector3 of the same Scalar as this quaternion, so it is not
    /// able to be applied to Jets and doubles correctly.
    const QuaternionType& q = unit_quaternion();
    PointProduct<PointDerived> uv = q.vec().cross(p);
    uv += uv;
    return p + q.w() * uv + q.vec().cross(uv);
  }

  /// Group action on homogeneous 3-points. See above for more details.
  template <typename HPointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<HPointDerived, 4>::value>::type>
  SOPHUS_FUNC HomogeneousPointProduct<HPointDerived> operator*(
      Eigen::MatrixBase<HPointDerived> const& p) const {
    const auto rp = *this * p.template head<3>();
    return HomogeneousPointProduct<HPointDerived>(rp(0), rp(1), rp(2), p(3));
  }

  /// Group action on lines.
  ///
  /// This function rotates a parametrized line ``l(t) = o + t * d`` by the SO3
  /// element:
  ///
  /// Both direction ``d`` and origin ``o`` are rotated as a 3 dimensional point
  ///
  SOPHUS_FUNC Line operator*(Line const& l) const {
    return Line((*this) * l.origin(), (*this) * l.direction());
  }

  /// In-place group multiplication. This method is only valid if the return
  /// type of the multiplication is compatible with this SO3's Scalar type.
  ///
  template <typename OtherDerived,
            typename = typename std::enable_if<
                std::is_same<Scalar, ReturnScalar<OtherDerived>>::value>::type>
  SOPHUS_FUNC SO3Base<Derived>& operator*=(SO3Base<OtherDerived> const& other) {
    *static_cast<Derived*>(this) = *this * other;
    return *this;
  }

  /// Takes in quaternion, and normalizes it.
  ///
  /// Precondition: The quaternion must not be close to zero.
  ///
  SOPHUS_FUNC void setQuaternion(Eigen::Quaternion<Scalar> const& quaternion) {
    unit_quaternion_nonconst() = quaternion;
    normalize();
  }

  /// Accessor of unit quaternion.
  ///
  SOPHUS_FUNC QuaternionType const& unit_quaternion() const {
    return static_cast<Derived const*>(this)->unit_quaternion();
  }

 private:
  /// Mutator of unit_quaternion is private to ensure class invariant. That is
  /// the quaternion must stay close to unit length.
  ///
  SOPHUS_FUNC QuaternionType& unit_quaternion_nonconst() {
    return static_cast<Derived*>(this)->unit_quaternion_nonconst();
  }
};

/// SO3 using default storage; derived from SO3Base.
template <class Scalar_, int Options>
class SO3 : public SO3Base<SO3<Scalar_, Options>> {
 public:
  using Base = SO3Base<SO3<Scalar_, Options>>;
  static int constexpr DoF = Base::DoF;
  static int constexpr num_parameters = Base::num_parameters;

  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using QuaternionMember = Eigen::Quaternion<Scalar, Options>;

  /// ``Base`` is friend so unit_quaternion_nonconst can be accessed from
  /// ``Base``.
  friend class SO3Base<SO3<Scalar, Options>>;

  using Base::operator=;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Default constructor initializes unit quaternion to identity rotation.
  ///
  SOPHUS_FUNC SO3()
      : unit_quaternion_(Scalar(1), Scalar(0), Scalar(0), Scalar(0)) {}

  /// Copy constructor
  ///
  SOPHUS_FUNC SO3(SO3 const& other) = default;

  /// Copy-like constructor from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC SO3(SO3Base<OtherDerived> const& other)
      : unit_quaternion_(other.unit_quaternion()) {}

  /// Constructor from rotation matrix
  ///
  /// Precondition: rotation matrix needs to be orthogonal with determinant
  /// of 1.
  ///
  SOPHUS_FUNC SO3(Transformation const& R) : unit_quaternion_(R) {
    SOPHUS_ENSURE(isOrthogonal(R), "R is not orthogonal:\n %",
                  R * R.transpose());
    SOPHUS_ENSURE(R.determinant() > Scalar(0), "det(R) is not positive: %",
                  R.determinant());
  }

  /// Constructor from quaternion
  ///
  /// Precondition: quaternion must not be close to zero.
  ///
  template <class D>
  SOPHUS_FUNC explicit SO3(Eigen::QuaternionBase<D> const& quat)
      : unit_quaternion_(quat) {
    static_assert(
        std::is_same<typename Eigen::QuaternionBase<D>::Scalar, Scalar>::value,
        "Input must be of same scalar type");
    Base::normalize();
  }

  /// Accessor of unit quaternion.
  ///
  SOPHUS_FUNC QuaternionMember const& unit_quaternion() const {
    return unit_quaternion_;
  }

  /// Returns derivative of exp(x) wrt. x.
  ///
  SOPHUS_FUNC static Sophus::Matrix<Scalar, num_parameters, DoF> Dx_exp_x(
      Tangent const& omega) {
    using std::cos;
    using std::exp;
    using std::sin;
    using std::sqrt;
    Scalar const c0 = omega[0] * omega[0];
    Scalar const c1 = omega[1] * omega[1];
    Scalar const c2 = omega[2] * omega[2];
    Scalar const c3 = c0 + c1 + c2;

    if (c3 < Constants<Scalar>::epsilon()) {
      return Dx_exp_x_at_0();
    }

    Scalar const c4 = sqrt(c3);
    Scalar const c5 = 1.0 / c4;
    Scalar const c6 = 0.5 * c4;
    Scalar const c7 = sin(c6);
    Scalar const c8 = c5 * c7;
    Scalar const c9 = pow(c3, -3.0L / 2.0L);
    Scalar const c10 = c7 * c9;
    Scalar const c11 = Scalar(1.0) / c3;
    Scalar const c12 = cos(c6);
    Scalar const c13 = Scalar(0.5) * c11 * c12;
    Scalar const c14 = c7 * c9 * omega[0];
    Scalar const c15 = Scalar(0.5) * c11 * c12 * omega[0];
    Scalar const c16 = -c14 * omega[1] + c15 * omega[1];
    Scalar const c17 = -c14 * omega[2] + c15 * omega[2];
    Scalar const c18 = omega[1] * omega[2];
    Scalar const c19 = -c10 * c18 + c13 * c18;
    Scalar const c20 = Scalar(0.5) * c5 * c7;
    Sophus::Matrix<Scalar, num_parameters, DoF> J;
    J(0, 0) = -c0 * c10 + c0 * c13 + c8;
    J(0, 1) = c16;
    J(0, 2) = c17;
    J(1, 0) = c16;
    J(1, 1) = -c1 * c10 + c1 * c13 + c8;
    J(1, 2) = c19;
    J(2, 0) = c17;
    J(2, 1) = c19;
    J(2, 2) = -c10 * c2 + c13 * c2 + c8;
    J(3, 0) = -c20 * omega[0];
    J(3, 1) = -c20 * omega[1];
    J(3, 2) = -c20 * omega[2];
    return J;
  }

  /// Returns derivative of exp(x) wrt. x_i at x=0.
  ///
  SOPHUS_FUNC static Sophus::Matrix<Scalar, num_parameters, DoF>
  Dx_exp_x_at_0() {
    Sophus::Matrix<Scalar, num_parameters, DoF> J;
    // clang-format off
    J <<  Scalar(0.5),   Scalar(0),   Scalar(0),
            Scalar(0), Scalar(0.5),   Scalar(0),
            Scalar(0),   Scalar(0), Scalar(0.5),
            Scalar(0),   Scalar(0),   Scalar(0);
    // clang-format on
    return J;
  }

  /// Returns derivative of exp(x).matrix() wrt. ``x_i at x=0``.
  ///
  SOPHUS_FUNC static Transformation Dxi_exp_x_matrix_at_0(int i) {
    return generator(i);
  }

  /// Group exponential
  ///
  /// This functions takes in an element of tangent space (= rotation vector
  /// ``omega``) and returns the corresponding element of the group SO(3).
  ///
  /// To be more specific, this function computes ``expmat(hat(omega))``
  /// with ``expmat(.)`` being the matrix exponential and ``hat(.)`` being the
  /// hat()-operator of SO(3).
  ///
  SOPHUS_FUNC static SO3<Scalar> exp(Tangent const& omega) {
    Scalar theta;
    return expAndTheta(omega, &theta);
  }

  /// As above, but also returns ``theta = |omega|`` as out-parameter.
  ///
  /// Precondition: ``theta`` must not be ``nullptr``.
  ///
  SOPHUS_FUNC static SO3<Scalar> expAndTheta(Tangent const& omega,
                                             Scalar* theta) {
    SOPHUS_ENSURE(theta != nullptr, "must not be nullptr.");
    using std::abs;
    using std::cos;
    using std::sin;
    using std::sqrt;
    Scalar theta_sq = omega.squaredNorm();

    Scalar imag_factor;
    Scalar real_factor;
    if (theta_sq <
        Constants<Scalar>::epsilon() * Constants<Scalar>::epsilon()) {
      *theta = Scalar(0);
      Scalar theta_po4 = theta_sq * theta_sq;
      imag_factor = Scalar(0.5) - Scalar(1.0 / 48.0) * theta_sq +
                    Scalar(1.0 / 3840.0) * theta_po4;
      real_factor = Scalar(1) - Scalar(1.0 / 8.0) * theta_sq +
                    Scalar(1.0 / 384.0) * theta_po4;
    } else {
      *theta = sqrt(theta_sq);
      Scalar half_theta = Scalar(0.5) * (*theta);
      Scalar sin_half_theta = sin(half_theta);
      imag_factor = sin_half_theta / (*theta);
      real_factor = cos(half_theta);
    }

    SO3 q;
    q.unit_quaternion_nonconst() =
        QuaternionMember(real_factor, imag_factor * omega.x(),
                         imag_factor * omega.y(), imag_factor * omega.z());
    SOPHUS_ENSURE(abs(q.unit_quaternion().squaredNorm() - Scalar(1)) <
                      Sophus::Constants<Scalar>::epsilon(),
                  "SO3::exp failed! omega: %, real: %, img: %",
                  omega.transpose(), real_factor, imag_factor);
    return q;
  }

  /// Returns closest SO3 given arbitrary 3x3 matrix.
  ///
  template <class S = Scalar>
  static SOPHUS_FUNC enable_if_t<std::is_floating_point<S>::value, SO3>
  fitToSO3(Transformation const& R) {
    return SO3(::Sophus::makeRotationMatrix(R));
  }

  /// Returns the ith infinitesimal generators of SO(3).
  ///
  /// The infinitesimal generators of SO(3) are:
  ///
  /// ```
  ///         |  0  0  0 |
  ///   G_0 = |  0  0 -1 |
  ///         |  0  1  0 |
  ///
  ///         |  0  0  1 |
  ///   G_1 = |  0  0  0 |
  ///         | -1  0  0 |
  ///
  ///         |  0 -1  0 |
  ///   G_2 = |  1  0  0 |
  ///         |  0  0  0 |
  /// ```
  ///
  /// Precondition: ``i`` must be 0, 1 or 2.
  ///
  SOPHUS_FUNC static Transformation generator(int i) {
    SOPHUS_ENSURE(i >= 0 && i <= 2, "i should be in range [0,2].");
    Tangent e;
    e.setZero();
    e[i] = Scalar(1);
    return hat(e);
  }

  /// hat-operator
  ///
  /// It takes in the 3-vector representation ``omega`` (= rotation vector) and
  /// returns the corresponding matrix representation of Lie algebra element.
  ///
  /// Formally, the hat()-operator of SO(3) is defined as
  ///
  ///   ``hat(.): R^3 -> R^{3x3},  hat(omega) = sum_i omega_i * G_i``
  ///   (for i=0,1,2)
  ///
  /// with ``G_i`` being the ith infinitesimal generator of SO(3).
  ///
  /// The corresponding inverse is the vee()-operator, see below.
  ///
  SOPHUS_FUNC static Transformation hat(Tangent const& omega) {
    Transformation Omega;
    // clang-format off
    Omega <<
        Scalar(0), -omega(2),  omega(1),
         omega(2), Scalar(0), -omega(0),
        -omega(1),  omega(0), Scalar(0);
    // clang-format on
    return Omega;
  }

  /// Lie bracket
  ///
  /// It computes the Lie bracket of SO(3). To be more specific, it computes
  ///
  ///   ``[omega_1, omega_2]_so3 := vee([hat(omega_1), hat(omega_2)])``
  ///
  /// with ``[A,B] := AB-BA`` being the matrix commutator, ``hat(.)`` the
  /// hat()-operator and ``vee(.)`` the vee()-operator of SO3.
  ///
  /// For the Lie algebra so3, the Lie bracket is simply the cross product:
  ///
  /// ``[omega_1, omega_2]_so3 = omega_1 x omega_2.``
  ///
  SOPHUS_FUNC static Tangent lieBracket(Tangent const& omega1,
                                        Tangent const& omega2) {
    return omega1.cross(omega2);
  }

  /// Construct x-axis rotation.
  ///
  static SOPHUS_FUNC SO3 rotX(Scalar const& x) {
    return SO3::exp(Sophus::Vector3<Scalar>(x, Scalar(0), Scalar(0)));
  }

  /// Construct y-axis rotation.
  ///
  static SOPHUS_FUNC SO3 rotY(Scalar const& y) {
    return SO3::exp(Sophus::Vector3<Scalar>(Scalar(0), y, Scalar(0)));
  }

  /// Construct z-axis rotation.
  ///
  static SOPHUS_FUNC SO3 rotZ(Scalar const& z) {
    return SO3::exp(Sophus::Vector3<Scalar>(Scalar(0), Scalar(0), z));
  }

  /// Draw uniform sample from SO(3) manifold.
  /// Based on: http://planning.cs.uiuc.edu/node198.html
  ///
  template <class UniformRandomBitGenerator>
  static SO3 sampleUniform(UniformRandomBitGenerator& generator) {
    static_assert(IsUniformRandomBitGenerator<UniformRandomBitGenerator>::value,
                  "generator must meet the UniformRandomBitGenerator concept");

    std::uniform_real_distribution<Scalar> uniform(Scalar(0), Scalar(1));
    std::uniform_real_distribution<Scalar> uniform_twopi(
        Scalar(0), 2 * Constants<Scalar>::pi());

    const Scalar u1 = uniform(generator);
    const Scalar u2 = uniform_twopi(generator);
    const Scalar u3 = uniform_twopi(generator);

    const Scalar a = sqrt(1 - u1);
    const Scalar b = sqrt(u1);

    return SO3(
        QuaternionMember(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3)));
  }

  /// vee-operator
  ///
  /// It takes the 3x3-matrix representation ``Omega`` and maps it to the
  /// corresponding vector representation of Lie algebra.
  ///
  /// This is the inverse of the hat()-operator, see above.
  ///
  /// Precondition: ``Omega`` must have the following structure:
  ///
  ///                |  0 -c  b |
  ///                |  c  0 -a |
  ///                | -b  a  0 |
  ///
  SOPHUS_FUNC static Tangent vee(Transformation const& Omega) {
    return Tangent(Omega(2, 1), Omega(0, 2), Omega(1, 0));
  }

 protected:
  /// Mutator of unit_quaternion is protected to ensure class invariant.
  ///
  SOPHUS_FUNC QuaternionMember& unit_quaternion_nonconst() {
    return unit_quaternion_;
  }

  QuaternionMember unit_quaternion_;
};

}  // namespace Sophus

namespace Eigen {
/// Specialization of Eigen::Map for ``SO3``; derived from SO3Base.
///
/// Allows us to wrap SO3 objects around POD array (e.g. external c style
/// quaternion).
template <class Scalar_, int Options>
class Map<Sophus::SO3<Scalar_>, Options>
    : public Sophus::SO3Base<Map<Sophus::SO3<Scalar_>, Options>> {
 public:
  using Base = Sophus::SO3Base<Map<Sophus::SO3<Scalar_>, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  /// ``Base`` is friend so unit_quaternion_nonconst can be accessed from
  /// ``Base``.
  friend class Sophus::SO3Base<Map<Sophus::SO3<Scalar_>, Options>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  SOPHUS_FUNC Map(Scalar* coeffs) : unit_quaternion_(coeffs) {}

  /// Accessor of unit quaternion.
  ///
  SOPHUS_FUNC Map<Eigen::Quaternion<Scalar>, Options> const& unit_quaternion()
      const {
    return unit_quaternion_;
  }

 protected:
  /// Mutator of unit_quaternion is protected to ensure class invariant.
  ///
  SOPHUS_FUNC Map<Eigen::Quaternion<Scalar>, Options>&
  unit_quaternion_nonconst() {
    return unit_quaternion_;
  }

  Map<Eigen::Quaternion<Scalar>, Options> unit_quaternion_;
};

/// Specialization of Eigen::Map for ``SO3 const``; derived from SO3Base.
///
/// Allows us to wrap SO3 objects around POD array (e.g. external c style
/// quaternion).
template <class Scalar_, int Options>
class Map<Sophus::SO3<Scalar_> const, Options>
    : public Sophus::SO3Base<Map<Sophus::SO3<Scalar_> const, Options>> {
 public:
  using Base = Sophus::SO3Base<Map<Sophus::SO3<Scalar_> const, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  using Base::operator*=;
  using Base::operator*;

  SOPHUS_FUNC Map(Scalar const* coeffs) : unit_quaternion_(coeffs) {}

  /// Accessor of unit quaternion.
  ///
  SOPHUS_FUNC Map<Eigen::Quaternion<Scalar> const, Options> const&
  unit_quaternion() const {
    return unit_quaternion_;
  }

 protected:
  /// Mutator of unit_quaternion is protected to ensure class invariant.
  ///
  Map<Eigen::Quaternion<Scalar> const, Options> const unit_quaternion_;
};
}  // namespace Eigen

#endif
