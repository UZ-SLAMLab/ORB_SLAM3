/// @file
/// Special Euclidean group SE(3) - rotation and translation in 3d.

#ifndef SOPHUS_SE3_HPP
#define SOPHUS_SE3_HPP

#include "so3.hpp"

namespace Sophus {
template <class Scalar_, int Options = 0>
class SE3;
using SE3d = SE3<double>;
using SE3f = SE3<float>;
}  // namespace Sophus

namespace Eigen {
namespace internal {

template <class Scalar_, int Options>
struct traits<Sophus::SE3<Scalar_, Options>> {
  using Scalar = Scalar_;
  using TranslationType = Sophus::Vector3<Scalar, Options>;
  using SO3Type = Sophus::SO3<Scalar, Options>;
};

template <class Scalar_, int Options>
struct traits<Map<Sophus::SE3<Scalar_>, Options>>
    : traits<Sophus::SE3<Scalar_, Options>> {
  using Scalar = Scalar_;
  using TranslationType = Map<Sophus::Vector3<Scalar>, Options>;
  using SO3Type = Map<Sophus::SO3<Scalar>, Options>;
};

template <class Scalar_, int Options>
struct traits<Map<Sophus::SE3<Scalar_> const, Options>>
    : traits<Sophus::SE3<Scalar_, Options> const> {
  using Scalar = Scalar_;
  using TranslationType = Map<Sophus::Vector3<Scalar> const, Options>;
  using SO3Type = Map<Sophus::SO3<Scalar> const, Options>;
};
}  // namespace internal
}  // namespace Eigen

namespace Sophus {

/// SE3 base type - implements SE3 class but is storage agnostic.
///
/// SE(3) is the group of rotations  and translation in 3d. It is the
/// semi-direct product of SO(3) and the 3d Euclidean vector space.  The class
/// is represented using a composition of SO3  for rotation and a one 3-vector
/// for translation.
///
/// SE(3) is neither compact, nor a commutative group.
///
/// See SO3 for more details of the rotation representation in 3d.
///
template <class Derived>
class SE3Base {
 public:
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using TranslationType =
      typename Eigen::internal::traits<Derived>::TranslationType;
  using SO3Type = typename Eigen::internal::traits<Derived>::SO3Type;
  using QuaternionType = typename SO3Type::QuaternionType;
  /// Degrees of freedom of manifold, number of dimensions in tangent space
  /// (three for translation, three for rotation).
  static int constexpr DoF = 6;
  /// Number of internal parameters used (4-tuple for quaternion, three for
  /// translation).
  static int constexpr num_parameters = 7;
  /// Group transformations are 4x4 matrices.
  static int constexpr N = 4;
  using Transformation = Matrix<Scalar, N, N>;
  using Point = Vector3<Scalar>;
  using HomogeneousPoint = Vector4<Scalar>;
  using Line = ParametrizedLine3<Scalar>;
  using Tangent = Vector<Scalar, DoF>;
  using Adjoint = Matrix<Scalar, DoF, DoF>;

  /// For binary operations the return type is determined with the
  /// ScalarBinaryOpTraits feature of Eigen. This allows mixing concrete and Map
  /// types, as well as other compatible scalar types such as Ceres::Jet and
  /// double scalars with SE3 operations.
  template <typename OtherDerived>
  using ReturnScalar = typename Eigen::ScalarBinaryOpTraits<
      Scalar, typename OtherDerived::Scalar>::ReturnType;

  template <typename OtherDerived>
  using SE3Product = SE3<ReturnScalar<OtherDerived>>;

  template <typename PointDerived>
  using PointProduct = Vector3<ReturnScalar<PointDerived>>;

  template <typename HPointDerived>
  using HomogeneousPointProduct = Vector4<ReturnScalar<HPointDerived>>;

  /// Adjoint transformation
  ///
  /// This function return the adjoint transformation ``Ad`` of the group
  /// element ``A`` such that for all ``x`` it holds that
  /// ``hat(Ad_A * x) = A * hat(x) A^{-1}``. See hat-operator below.
  ///
  SOPHUS_FUNC Adjoint Adj() const {
    Sophus::Matrix3<Scalar> const R = so3().matrix();
    Adjoint res;
    res.block(0, 0, 3, 3) = R;
    res.block(3, 3, 3, 3) = R;
    res.block(0, 3, 3, 3) = SO3<Scalar>::hat(translation()) * R;
    res.block(3, 0, 3, 3) = Matrix3<Scalar>::Zero(3, 3);
    return res;
  }

  /// Extract rotation angle about canonical X-axis
  ///
  Scalar angleX() const { return so3().angleX(); }

  /// Extract rotation angle about canonical Y-axis
  ///
  Scalar angleY() const { return so3().angleY(); }

  /// Extract rotation angle about canonical Z-axis
  ///
  Scalar angleZ() const { return so3().angleZ(); }

  /// Returns copy of instance casted to NewScalarType.
  ///
  template <class NewScalarType>
  SOPHUS_FUNC SE3<NewScalarType> cast() const {
    return SE3<NewScalarType>(so3().template cast<NewScalarType>(),
                              translation().template cast<NewScalarType>());
  }

  /// Returns derivative of  this * exp(x)  wrt x at x=0.
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
    Scalar const c7 = q.w() * q.w();
    Scalar const c8 = q.x() * q.x();
    Scalar const c9 = q.y() * q.y();
    Scalar const c10 = -c9;
    Scalar const c11 = q.z() * q.z();
    Scalar const c12 = -c11;
    Scalar const c13 = Scalar(2) * q.w();
    Scalar const c14 = c13 * q.z();
    Scalar const c15 = Scalar(2) * q.x();
    Scalar const c16 = c15 * q.y();
    Scalar const c17 = c13 * q.y();
    Scalar const c18 = c15 * q.z();
    Scalar const c19 = c7 - c8;
    Scalar const c20 = c13 * q.x();
    Scalar const c21 = Scalar(2) * q.y() * q.z();
    J(0, 0) = 0;
    J(0, 1) = 0;
    J(0, 2) = 0;
    J(0, 3) = c0;
    J(0, 4) = c2;
    J(0, 5) = c3;
    J(1, 0) = 0;
    J(1, 1) = 0;
    J(1, 2) = 0;
    J(1, 3) = c1;
    J(1, 4) = c0;
    J(1, 5) = c5;
    J(2, 0) = 0;
    J(2, 1) = 0;
    J(2, 2) = 0;
    J(2, 3) = c6;
    J(2, 4) = c4;
    J(2, 5) = c0;
    J(3, 0) = 0;
    J(3, 1) = 0;
    J(3, 2) = 0;
    J(3, 3) = c5;
    J(3, 4) = c6;
    J(3, 5) = c2;
    J(4, 0) = c10 + c12 + c7 + c8;
    J(4, 1) = -c14 + c16;
    J(4, 2) = c17 + c18;
    J(4, 3) = 0;
    J(4, 4) = 0;
    J(4, 5) = 0;
    J(5, 0) = c14 + c16;
    J(5, 1) = c12 + c19 + c9;
    J(5, 2) = -c20 + c21;
    J(5, 3) = 0;
    J(5, 4) = 0;
    J(5, 5) = 0;
    J(6, 0) = -c17 + c18;
    J(6, 1) = c20 + c21;
    J(6, 2) = c10 + c11 + c19;
    J(6, 3) = 0;
    J(6, 4) = 0;
    J(6, 5) = 0;
    return J;
  }

  /// Returns group inverse.
  ///
  SOPHUS_FUNC SE3<Scalar> inverse() const {
    SO3<Scalar> invR = so3().inverse();
    return SE3<Scalar>(invR, invR * (translation() * Scalar(-1)));
  }

  /// Logarithmic map
  ///
  /// Computes the logarithm, the inverse of the group exponential which maps
  /// element of the group (rigid body transformations) to elements of the
  /// tangent space (twist).
  ///
  /// To be specific, this function computes ``vee(logmat(.))`` with
  /// ``logmat(.)`` being the matrix logarithm and ``vee(.)`` the vee-operator
  /// of SE(3).
  ///
  SOPHUS_FUNC Tangent log() const {
    // For the derivation of the logarithm of SE(3), see
    // J. Gallier, D. Xu, "Computing exponentials of skew symmetric matrices
    // and logarithms of orthogonal matrices", IJRA 2002.
    // https:///pdfs.semanticscholar.org/cfe3/e4b39de63c8cabd89bf3feff7f5449fc981d.pdf
    // (Sec. 6., pp. 8)
    using std::abs;
    using std::cos;
    using std::sin;
    Tangent upsilon_omega;
    auto omega_and_theta = so3().logAndTheta();
    Scalar theta = omega_and_theta.theta;
    upsilon_omega.template tail<3>() = omega_and_theta.tangent;
    Matrix3<Scalar> const Omega =
        SO3<Scalar>::hat(upsilon_omega.template tail<3>());

    if (abs(theta) < Constants<Scalar>::epsilon()) {
      Matrix3<Scalar> const V_inv = Matrix3<Scalar>::Identity() -
                                    Scalar(0.5) * Omega +
                                    Scalar(1. / 12.) * (Omega * Omega);

      upsilon_omega.template head<3>() = V_inv * translation();
    } else {
      Scalar const half_theta = Scalar(0.5) * theta;

      Matrix3<Scalar> const V_inv =
          (Matrix3<Scalar>::Identity() - Scalar(0.5) * Omega +
           (Scalar(1) -
            theta * cos(half_theta) / (Scalar(2) * sin(half_theta))) /
               (theta * theta) * (Omega * Omega));
      upsilon_omega.template head<3>() = V_inv * translation();
    }
    return upsilon_omega;
  }

  /// It re-normalizes the SO3 element.
  ///
  /// Note: Because of the class invariant of SO3, there is typically no need to
  /// call this function directly.
  ///
  SOPHUS_FUNC void normalize() { so3().normalize(); }

  /// Returns 4x4 matrix representation of the instance.
  ///
  /// It has the following form:
  ///
  ///   | R t |
  ///   | o 1 |
  ///
  /// where ``R`` is a 3x3 rotation matrix, ``t`` a translation 3-vector and
  /// ``o`` a 3-column vector of zeros.
  ///
  SOPHUS_FUNC Transformation matrix() const {
    Transformation homogenious_matrix;
    homogenious_matrix.template topLeftCorner<3, 4>() = matrix3x4();
    homogenious_matrix.row(3) =
        Matrix<Scalar, 1, 4>(Scalar(0), Scalar(0), Scalar(0), Scalar(1));
    return homogenious_matrix;
  }

  /// Returns the significant first three rows of the matrix above.
  ///
  SOPHUS_FUNC Matrix<Scalar, 3, 4> matrix3x4() const {
    Matrix<Scalar, 3, 4> matrix;
    matrix.template topLeftCorner<3, 3>() = rotationMatrix();
    matrix.col(3) = translation();
    return matrix;
  }

  /// Assignment-like operator from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC SE3Base<Derived>& operator=(SE3Base<OtherDerived> const& other) {
    so3() = other.so3();
    translation() = other.translation();
    return *this;
  }

  /// Group multiplication, which is rotation concatenation.
  ///
  template <typename OtherDerived>
  SOPHUS_FUNC SE3Product<OtherDerived> operator*(
      SE3Base<OtherDerived> const& other) const {
    return SE3Product<OtherDerived>(
        so3() * other.so3(), translation() + so3() * other.translation());
  }

  /// Group action on 3-points.
  ///
  /// This function rotates and translates a three dimensional point ``p`` by
  /// the SE(3) element ``bar_T_foo = (bar_R_foo, t_bar)`` (= rigid body
  /// transformation):
  ///
  ///   ``p_bar = bar_R_foo * p_foo + t_bar``.
  ///
  template <typename PointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<PointDerived, 3>::value>::type>
  SOPHUS_FUNC PointProduct<PointDerived> operator*(
      Eigen::MatrixBase<PointDerived> const& p) const {
    return so3() * p + translation();
  }

  /// Group action on homogeneous 3-points. See above for more details.
  ///
  template <typename HPointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<HPointDerived, 4>::value>::type>
  SOPHUS_FUNC HomogeneousPointProduct<HPointDerived> operator*(
      Eigen::MatrixBase<HPointDerived> const& p) const {
    const PointProduct<HPointDerived> tp =
        so3() * p.template head<3>() + p(3) * translation();
    return HomogeneousPointProduct<HPointDerived>(tp(0), tp(1), tp(2), p(3));
  }

  /// Group action on lines.
  ///
  /// This function rotates and translates a parametrized line
  /// ``l(t) = o + t * d`` by the SE(3) element:
  ///
  /// Origin is transformed using SE(3) action
  /// Direction is transformed using rotation part
  ///
  SOPHUS_FUNC Line operator*(Line const& l) const {
    return Line((*this) * l.origin(), so3() * l.direction());
  }

  /// In-place group multiplication. This method is only valid if the return
  /// type of the multiplication is compatible with this SE3's Scalar type.
  ///
  template <typename OtherDerived,
            typename = typename std::enable_if<
                std::is_same<Scalar, ReturnScalar<OtherDerived>>::value>::type>
  SOPHUS_FUNC SE3Base<Derived>& operator*=(SE3Base<OtherDerived> const& other) {
    *static_cast<Derived*>(this) = *this * other;
    return *this;
  }

  /// Returns rotation matrix.
  ///
  SOPHUS_FUNC Matrix3<Scalar> rotationMatrix() const { return so3().matrix(); }

  /// Mutator of SO3 group.
  ///
  SOPHUS_FUNC SO3Type& so3() { return static_cast<Derived*>(this)->so3(); }

  /// Accessor of SO3 group.
  ///
  SOPHUS_FUNC SO3Type const& so3() const {
    return static_cast<const Derived*>(this)->so3();
  }

  /// Takes in quaternion, and normalizes it.
  ///
  /// Precondition: The quaternion must not be close to zero.
  ///
  SOPHUS_FUNC void setQuaternion(Eigen::Quaternion<Scalar> const& quat) {
    so3().setQuaternion(quat);
  }

  /// Sets ``so3`` using ``rotation_matrix``.
  ///
  /// Precondition: ``R`` must be orthogonal and ``det(R)=1``.
  ///
  SOPHUS_FUNC void setRotationMatrix(Matrix3<Scalar> const& R) {
    SOPHUS_ENSURE(isOrthogonal(R), "R is not orthogonal:\n %", R);
    SOPHUS_ENSURE(R.determinant() > Scalar(0), "det(R) is not positive: %",
                  R.determinant());
    so3().setQuaternion(Eigen::Quaternion<Scalar>(R));
  }

  /// Returns internal parameters of SE(3).
  ///
  /// It returns (q.imag[0], q.imag[1], q.imag[2], q.real, t[0], t[1], t[2]),
  /// with q being the unit quaternion, t the translation 3-vector.
  ///
  SOPHUS_FUNC Sophus::Vector<Scalar, num_parameters> params() const {
    Sophus::Vector<Scalar, num_parameters> p;
    p << so3().params(), translation();
    return p;
  }

  /// Mutator of translation vector.
  ///
  SOPHUS_FUNC TranslationType& translation() {
    return static_cast<Derived*>(this)->translation();
  }

  /// Accessor of translation vector
  ///
  SOPHUS_FUNC TranslationType const& translation() const {
    return static_cast<Derived const*>(this)->translation();
  }

  /// Accessor of unit quaternion.
  ///
  SOPHUS_FUNC QuaternionType const& unit_quaternion() const {
    return this->so3().unit_quaternion();
  }
};

/// SE3 using default storage; derived from SE3Base.
template <class Scalar_, int Options>
class SE3 : public SE3Base<SE3<Scalar_, Options>> {
  using Base = SE3Base<SE3<Scalar_, Options>>;

 public:
  static int constexpr DoF = Base::DoF;
  static int constexpr num_parameters = Base::num_parameters;

  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using SO3Member = SO3<Scalar, Options>;
  using TranslationMember = Vector3<Scalar, Options>;

  using Base::operator=;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Default constructor initializes rigid body motion to the identity.
  ///
  SOPHUS_FUNC SE3();

  /// Copy constructor
  ///
  SOPHUS_FUNC SE3(SE3 const& other) = default;

  /// Copy-like constructor from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC SE3(SE3Base<OtherDerived> const& other)
      : so3_(other.so3()), translation_(other.translation()) {
    static_assert(std::is_same<typename OtherDerived::Scalar, Scalar>::value,
                  "must be same Scalar type");
  }

  /// Constructor from SO3 and translation vector
  ///
  template <class OtherDerived, class D>
  SOPHUS_FUNC SE3(SO3Base<OtherDerived> const& so3,
                  Eigen::MatrixBase<D> const& translation)
      : so3_(so3), translation_(translation) {
    static_assert(std::is_same<typename OtherDerived::Scalar, Scalar>::value,
                  "must be same Scalar type");
    static_assert(std::is_same<typename D::Scalar, Scalar>::value,
                  "must be same Scalar type");
  }

  /// Constructor from rotation matrix and translation vector
  ///
  /// Precondition: Rotation matrix needs to be orthogonal with determinant
  ///               of 1.
  ///
  SOPHUS_FUNC
  SE3(Matrix3<Scalar> const& rotation_matrix, Point const& translation)
      : so3_(rotation_matrix), translation_(translation) {}

  /// Constructor from quaternion and translation vector.
  ///
  /// Precondition: ``quaternion`` must not be close to zero.
  ///
  SOPHUS_FUNC SE3(Eigen::Quaternion<Scalar> const& quaternion,
                  Point const& translation)
      : so3_(quaternion), translation_(translation) {}

  /// Constructor from 4x4 matrix
  ///
  /// Precondition: Rotation matrix needs to be orthogonal with determinant
  ///               of 1. The last row must be ``(0, 0, 0, 1)``.
  ///
  SOPHUS_FUNC explicit SE3(Matrix4<Scalar> const& T)
      : so3_(T.template topLeftCorner<3, 3>()),
        translation_(T.template block<3, 1>(0, 3)) {
    SOPHUS_ENSURE((T.row(3) - Matrix<Scalar, 1, 4>(Scalar(0), Scalar(0),
                                                   Scalar(0), Scalar(1)))
                          .squaredNorm() < Constants<Scalar>::epsilon(),
                  "Last row is not (0,0,0,1), but (%).", T.row(3));
  }

  /// This provides unsafe read/write access to internal data. SO(3) is
  /// represented by an Eigen::Quaternion (four parameters). When using direct
  /// write access, the user needs to take care of that the quaternion stays
  /// normalized.
  ///
  SOPHUS_FUNC Scalar* data() {
    // so3_ and translation_ are laid out sequentially with no padding
    return so3_.data();
  }

  /// Const version of data() above.
  ///
  SOPHUS_FUNC Scalar const* data() const {
    // so3_ and translation_ are laid out sequentially with no padding
    return so3_.data();
  }

  /// Mutator of SO3
  ///
  SOPHUS_FUNC SO3Member& so3() { return so3_; }

  /// Accessor of SO3
  ///
  SOPHUS_FUNC SO3Member const& so3() const { return so3_; }

  /// Mutator of translation vector
  ///
  SOPHUS_FUNC TranslationMember& translation() { return translation_; }

  /// Accessor of translation vector
  ///
  SOPHUS_FUNC TranslationMember const& translation() const {
    return translation_;
  }

  /// Returns derivative of exp(x) wrt. x.
  ///
  SOPHUS_FUNC static Sophus::Matrix<Scalar, num_parameters, DoF> Dx_exp_x(
      Tangent const& upsilon_omega) {
    using std::cos;
    using std::pow;
    using std::sin;
    using std::sqrt;
    Sophus::Matrix<Scalar, num_parameters, DoF> J;
    Sophus::Vector<Scalar, 3> upsilon = upsilon_omega.template head<3>();
    Sophus::Vector<Scalar, 3> omega = upsilon_omega.template tail<3>();

    Scalar const c0 = omega[0] * omega[0];
    Scalar const c1 = omega[1] * omega[1];
    Scalar const c2 = omega[2] * omega[2];
    Scalar const c3 = c0 + c1 + c2;
    Scalar const o(0);
    Scalar const h(0.5);
    Scalar const i(1);

    if (c3 < Constants<Scalar>::epsilon()) {
      Scalar const ux = Scalar(0.5) * upsilon[0];
      Scalar const uy = Scalar(0.5) * upsilon[1];
      Scalar const uz = Scalar(0.5) * upsilon[2];

      /// clang-format off
      J << o, o, o, h, o, o, o, o, o, o, h, o, o, o, o, o, o, h, o, o, o, o, o,
          o, i, o, o, o, uz, -uy, o, i, o, -uz, o, ux, o, o, i, uy, -ux, o;
      /// clang-format on
      return J;
    }

    Scalar const c4 = sqrt(c3);
    Scalar const c5 = Scalar(1.0) / c4;
    Scalar const c6 = Scalar(0.5) * c4;
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
    Scalar const c20 = c5 * omega[0];
    Scalar const c21 = Scalar(0.5) * c7;
    Scalar const c22 = c5 * omega[1];
    Scalar const c23 = c5 * omega[2];
    Scalar const c24 = -c1;
    Scalar const c25 = -c2;
    Scalar const c26 = c24 + c25;
    Scalar const c27 = sin(c4);
    Scalar const c28 = -c27 + c4;
    Scalar const c29 = c28 * c9;
    Scalar const c30 = cos(c4);
    Scalar const c31 = -c30 + Scalar(1);
    Scalar const c32 = c11 * c31;
    Scalar const c33 = c32 * omega[2];
    Scalar const c34 = c29 * omega[0];
    Scalar const c35 = c34 * omega[1];
    Scalar const c36 = c32 * omega[1];
    Scalar const c37 = c34 * omega[2];
    Scalar const c38 = pow(c3, -5.0L / 2.0L);
    Scalar const c39 = Scalar(3) * c28 * c38 * omega[0];
    Scalar const c40 = c26 * c9;
    Scalar const c41 = -c20 * c30 + c20;
    Scalar const c42 = c27 * c9 * omega[0];
    Scalar const c43 = c42 * omega[1];
    Scalar const c44 = pow(c3, -2);
    Scalar const c45 = Scalar(2) * c31 * c44 * omega[0];
    Scalar const c46 = c45 * omega[1];
    Scalar const c47 = c29 * omega[2];
    Scalar const c48 = c43 - c46 + c47;
    Scalar const c49 = Scalar(3) * c0 * c28 * c38;
    Scalar const c50 = c9 * omega[0] * omega[2];
    Scalar const c51 = c41 * c50 - c49 * omega[2];
    Scalar const c52 = c9 * omega[0] * omega[1];
    Scalar const c53 = c41 * c52 - c49 * omega[1];
    Scalar const c54 = c42 * omega[2];
    Scalar const c55 = c45 * omega[2];
    Scalar const c56 = c29 * omega[1];
    Scalar const c57 = -c54 + c55 + c56;
    Scalar const c58 = Scalar(-2) * c56;
    Scalar const c59 = Scalar(3) * c28 * c38 * omega[1];
    Scalar const c60 = -c22 * c30 + c22;
    Scalar const c61 = -c18 * c39;
    Scalar const c62 = c32 + c61;
    Scalar const c63 = c27 * c9;
    Scalar const c64 = c1 * c63;
    Scalar const c65 = Scalar(2) * c31 * c44;
    Scalar const c66 = c1 * c65;
    Scalar const c67 = c50 * c60;
    Scalar const c68 = -c1 * c39 + c52 * c60;
    Scalar const c69 = c18 * c63;
    Scalar const c70 = c18 * c65;
    Scalar const c71 = c34 - c69 + c70;
    Scalar const c72 = Scalar(-2) * c47;
    Scalar const c73 = Scalar(3) * c28 * c38 * omega[2];
    Scalar const c74 = -c23 * c30 + c23;
    Scalar const c75 = -c32 + c61;
    Scalar const c76 = c2 * c63;
    Scalar const c77 = c2 * c65;
    Scalar const c78 = c52 * c74;
    Scalar const c79 = c34 + c69 - c70;
    Scalar const c80 = -c2 * c39 + c50 * c74;
    Scalar const c81 = -c0;
    Scalar const c82 = c25 + c81;
    Scalar const c83 = c32 * omega[0];
    Scalar const c84 = c18 * c29;
    Scalar const c85 = Scalar(-2) * c34;
    Scalar const c86 = c82 * c9;
    Scalar const c87 = c0 * c63;
    Scalar const c88 = c0 * c65;
    Scalar const c89 = c9 * omega[1] * omega[2];
    Scalar const c90 = c41 * c89;
    Scalar const c91 = c54 - c55 + c56;
    Scalar const c92 = -c1 * c73 + c60 * c89;
    Scalar const c93 = -c43 + c46 + c47;
    Scalar const c94 = -c2 * c59 + c74 * c89;
    Scalar const c95 = c24 + c81;
    Scalar const c96 = c9 * c95;
    J(0, 0) = o;
    J(0, 1) = o;
    J(0, 2) = o;
    J(0, 3) = -c0 * c10 + c0 * c13 + c8;
    J(0, 4) = c16;
    J(0, 5) = c17;
    J(1, 0) = o;
    J(1, 1) = o;
    J(1, 2) = o;
    J(1, 3) = c16;
    J(1, 4) = -c1 * c10 + c1 * c13 + c8;
    J(1, 5) = c19;
    J(2, 0) = o;
    J(2, 1) = o;
    J(2, 2) = o;
    J(2, 3) = c17;
    J(2, 4) = c19;
    J(2, 5) = -c10 * c2 + c13 * c2 + c8;
    J(3, 0) = o;
    J(3, 1) = o;
    J(3, 2) = o;
    J(3, 3) = -c20 * c21;
    J(3, 4) = -c21 * c22;
    J(3, 5) = -c21 * c23;
    J(4, 0) = c26 * c29 + Scalar(1);
    J(4, 1) = -c33 + c35;
    J(4, 2) = c36 + c37;
    J(4, 3) = upsilon[0] * (-c26 * c39 + c40 * c41) + upsilon[1] * (c53 + c57) +
              upsilon[2] * (c48 + c51);
    J(4, 4) = upsilon[0] * (-c26 * c59 + c40 * c60 + c58) +
              upsilon[1] * (c68 + c71) + upsilon[2] * (c62 + c64 - c66 + c67);
    J(4, 5) = upsilon[0] * (-c26 * c73 + c40 * c74 + c72) +
              upsilon[1] * (c75 - c76 + c77 + c78) + upsilon[2] * (c79 + c80);
    J(5, 0) = c33 + c35;
    J(5, 1) = c29 * c82 + Scalar(1);
    J(5, 2) = -c83 + c84;
    J(5, 3) = upsilon[0] * (c53 + c91) +
              upsilon[1] * (-c39 * c82 + c41 * c86 + c85) +
              upsilon[2] * (c75 - c87 + c88 + c90);
    J(5, 4) = upsilon[0] * (c68 + c79) + upsilon[1] * (-c59 * c82 + c60 * c86) +
              upsilon[2] * (c92 + c93);
    J(5, 5) = upsilon[0] * (c62 + c76 - c77 + c78) +
              upsilon[1] * (c72 - c73 * c82 + c74 * c86) +
              upsilon[2] * (c57 + c94);
    J(6, 0) = -c36 + c37;
    J(6, 1) = c83 + c84;
    J(6, 2) = c29 * c95 + Scalar(1);
    J(6, 3) = upsilon[0] * (c51 + c93) + upsilon[1] * (c62 + c87 - c88 + c90) +
              upsilon[2] * (-c39 * c95 + c41 * c96 + c85);
    J(6, 4) = upsilon[0] * (-c64 + c66 + c67 + c75) + upsilon[1] * (c48 + c92) +
              upsilon[2] * (c58 - c59 * c95 + c60 * c96);
    J(6, 5) = upsilon[0] * (c71 + c80) + upsilon[1] * (c91 + c94) +
              upsilon[2] * (-c73 * c95 + c74 * c96);

    return J;
  }

  /// Returns derivative of exp(x) wrt. x_i at x=0.
  ///
  SOPHUS_FUNC static Sophus::Matrix<Scalar, num_parameters, DoF>
  Dx_exp_x_at_0() {
    Sophus::Matrix<Scalar, num_parameters, DoF> J;
    Scalar const o(0);
    Scalar const h(0.5);
    Scalar const i(1);

    // clang-format off
    J << o, o, o, h, o, o, o,
         o, o, o, h, o, o, o,
         o, o, o, h, o, o, o,
         o, o, o, i, o, o, o,
         o, o, o, i, o, o, o,
         o, o, o, i, o, o, o;
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
  /// This functions takes in an element of tangent space (= twist ``a``) and
  /// returns the corresponding element of the group SE(3).
  ///
  /// The first three components of ``a`` represent the translational part
  /// ``upsilon`` in the tangent space of SE(3), while the last three components
  /// of ``a`` represents the rotation vector ``omega``.
  /// To be more specific, this function computes ``expmat(hat(a))`` with
  /// ``expmat(.)`` being the matrix exponential and ``hat(.)`` the hat-operator
  /// of SE(3), see below.
  ///
  SOPHUS_FUNC static SE3<Scalar> exp(Tangent const& a) {
    using std::cos;
    using std::sin;
    Vector3<Scalar> const omega = a.template tail<3>();

    Scalar theta;
    SO3<Scalar> const so3 = SO3<Scalar>::expAndTheta(omega, &theta);
    Matrix3<Scalar> const Omega = SO3<Scalar>::hat(omega);
    Matrix3<Scalar> const Omega_sq = Omega * Omega;
    Matrix3<Scalar> V;

    if (theta < Constants<Scalar>::epsilon()) {
      V = so3.matrix();
      /// Note: That is an accurate expansion!
    } else {
      Scalar theta_sq = theta * theta;
      V = (Matrix3<Scalar>::Identity() +
           (Scalar(1) - cos(theta)) / (theta_sq)*Omega +
           (theta - sin(theta)) / (theta_sq * theta) * Omega_sq);
    }
    return SE3<Scalar>(so3, V * a.template head<3>());
  }

  /// Returns closest SE3 given arbirary 4x4 matrix.
  ///
  template <class S = Scalar>
  SOPHUS_FUNC static enable_if_t<std::is_floating_point<S>::value, SE3>
  fitToSE3(Matrix4<Scalar> const& T) {
    return SE3(SO3<Scalar>::fitToSO3(T.template block<3, 3>(0, 0)),
               T.template block<3, 1>(0, 3));
  }

  /// Returns the ith infinitesimal generators of SE(3).
  ///
  /// The infinitesimal generators of SE(3) are:
  ///
  /// ```
  ///         |  0  0  0  1 |
  ///   G_0 = |  0  0  0  0 |
  ///         |  0  0  0  0 |
  ///         |  0  0  0  0 |
  ///
  ///         |  0  0  0  0 |
  ///   G_1 = |  0  0  0  1 |
  ///         |  0  0  0  0 |
  ///         |  0  0  0  0 |
  ///
  ///         |  0  0  0  0 |
  ///   G_2 = |  0  0  0  0 |
  ///         |  0  0  0  1 |
  ///         |  0  0  0  0 |
  ///
  ///         |  0  0  0  0 |
  ///   G_3 = |  0  0 -1  0 |
  ///         |  0  1  0  0 |
  ///         |  0  0  0  0 |
  ///
  ///         |  0  0  1  0 |
  ///   G_4 = |  0  0  0  0 |
  ///         | -1  0  0  0 |
  ///         |  0  0  0  0 |
  ///
  ///         |  0 -1  0  0 |
  ///   G_5 = |  1  0  0  0 |
  ///         |  0  0  0  0 |
  ///         |  0  0  0  0 |
  /// ```
  ///
  /// Precondition: ``i`` must be in [0, 5].
  ///
  SOPHUS_FUNC static Transformation generator(int i) {
    SOPHUS_ENSURE(i >= 0 && i <= 5, "i should be in range [0,5].");
    Tangent e;
    e.setZero();
    e[i] = Scalar(1);
    return hat(e);
  }

  /// hat-operator
  ///
  /// It takes in the 6-vector representation (= twist) and returns the
  /// corresponding matrix representation of Lie algebra element.
  ///
  /// Formally, the hat()-operator of SE(3) is defined as
  ///
  ///   ``hat(.): R^6 -> R^{4x4},  hat(a) = sum_i a_i * G_i``  (for i=0,...,5)
  ///
  /// with ``G_i`` being the ith infinitesimal generator of SE(3).
  ///
  /// The corresponding inverse is the vee()-operator, see below.
  ///
  SOPHUS_FUNC static Transformation hat(Tangent const& a) {
    Transformation Omega;
    Omega.setZero();
    Omega.template topLeftCorner<3, 3>() =
        SO3<Scalar>::hat(a.template tail<3>());
    Omega.col(3).template head<3>() = a.template head<3>();
    return Omega;
  }

  /// Lie bracket
  ///
  /// It computes the Lie bracket of SE(3). To be more specific, it computes
  ///
  ///   ``[omega_1, omega_2]_se3 := vee([hat(omega_1), hat(omega_2)])``
  ///
  /// with ``[A,B] := AB-BA`` being the matrix commutator, ``hat(.)`` the
  /// hat()-operator and ``vee(.)`` the vee()-operator of SE(3).
  ///
  SOPHUS_FUNC static Tangent lieBracket(Tangent const& a, Tangent const& b) {
    Vector3<Scalar> const upsilon1 = a.template head<3>();
    Vector3<Scalar> const upsilon2 = b.template head<3>();
    Vector3<Scalar> const omega1 = a.template tail<3>();
    Vector3<Scalar> const omega2 = b.template tail<3>();

    Tangent res;
    res.template head<3>() = omega1.cross(upsilon2) + upsilon1.cross(omega2);
    res.template tail<3>() = omega1.cross(omega2);

    return res;
  }

  /// Construct x-axis rotation.
  ///
  static SOPHUS_FUNC SE3 rotX(Scalar const& x) {
    return SE3(SO3<Scalar>::rotX(x), Sophus::Vector3<Scalar>::Zero());
  }

  /// Construct y-axis rotation.
  ///
  static SOPHUS_FUNC SE3 rotY(Scalar const& y) {
    return SE3(SO3<Scalar>::rotY(y), Sophus::Vector3<Scalar>::Zero());
  }

  /// Construct z-axis rotation.
  ///
  static SOPHUS_FUNC SE3 rotZ(Scalar const& z) {
    return SE3(SO3<Scalar>::rotZ(z), Sophus::Vector3<Scalar>::Zero());
  }

  /// Draw uniform sample from SE(3) manifold.
  ///
  /// Translations are drawn component-wise from the range [-1, 1].
  ///
  template <class UniformRandomBitGenerator>
  static SE3 sampleUniform(UniformRandomBitGenerator& generator) {
    std::uniform_real_distribution<Scalar> uniform(Scalar(-1), Scalar(1));
    return SE3(SO3<Scalar>::sampleUniform(generator),
               Vector3<Scalar>(uniform(generator), uniform(generator),
                               uniform(generator)));
  }

  /// Construct a translation only SE3 instance.
  ///
  template <class T0, class T1, class T2>
  static SOPHUS_FUNC SE3 trans(T0 const& x, T1 const& y, T2 const& z) {
    return SE3(SO3<Scalar>(), Vector3<Scalar>(x, y, z));
  }

  static SOPHUS_FUNC SE3 trans(Vector3<Scalar> const& xyz) {
    return SE3(SO3<Scalar>(), xyz);
  }

  /// Construct x-axis translation.
  ///
  static SOPHUS_FUNC SE3 transX(Scalar const& x) {
    return SE3::trans(x, Scalar(0), Scalar(0));
  }

  /// Construct y-axis translation.
  ///
  static SOPHUS_FUNC SE3 transY(Scalar const& y) {
    return SE3::trans(Scalar(0), y, Scalar(0));
  }

  /// Construct z-axis translation.
  ///
  static SOPHUS_FUNC SE3 transZ(Scalar const& z) {
    return SE3::trans(Scalar(0), Scalar(0), z);
  }

  /// vee-operator
  ///
  /// It takes 4x4-matrix representation ``Omega`` and maps it to the
  /// corresponding 6-vector representation of Lie algebra.
  ///
  /// This is the inverse of the hat()-operator, see above.
  ///
  /// Precondition: ``Omega`` must have the following structure:
  ///
  ///                |  0 -f  e  a |
  ///                |  f  0 -d  b |
  ///                | -e  d  0  c
  ///                |  0  0  0  0 | .
  ///
  SOPHUS_FUNC static Tangent vee(Transformation const& Omega) {
    Tangent upsilon_omega;
    upsilon_omega.template head<3>() = Omega.col(3).template head<3>();
    upsilon_omega.template tail<3>() =
        SO3<Scalar>::vee(Omega.template topLeftCorner<3, 3>());
    return upsilon_omega;
  }

 protected:
  SO3Member so3_;
  TranslationMember translation_;
};

template <class Scalar, int Options>
SE3<Scalar, Options>::SE3() : translation_(TranslationMember::Zero()) {
  static_assert(std::is_standard_layout<SE3>::value,
                "Assume standard layout for the use of offsetof check below.");
  static_assert(
      offsetof(SE3, so3_) + sizeof(Scalar) * SO3<Scalar>::num_parameters ==
          offsetof(SE3, translation_),
      "This class assumes packed storage and hence will only work "
      "correctly depending on the compiler (options) - in "
      "particular when using [this->data(), this-data() + "
      "num_parameters] to access the raw data in a contiguous fashion.");
}
}  // namespace Sophus

namespace Eigen {

/// Specialization of Eigen::Map for ``SE3``; derived from SE3Base.
///
/// Allows us to wrap SE3 objects around POD array.
template <class Scalar_, int Options>
class Map<Sophus::SE3<Scalar_>, Options>
    : public Sophus::SE3Base<Map<Sophus::SE3<Scalar_>, Options>> {
 public:
  using Base = Sophus::SE3Base<Map<Sophus::SE3<Scalar_>, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  SOPHUS_FUNC Map(Scalar* coeffs)
      : so3_(coeffs),
        translation_(coeffs + Sophus::SO3<Scalar>::num_parameters) {}

  /// Mutator of SO3
  ///
  SOPHUS_FUNC Map<Sophus::SO3<Scalar>, Options>& so3() { return so3_; }

  /// Accessor of SO3
  ///
  SOPHUS_FUNC Map<Sophus::SO3<Scalar>, Options> const& so3() const {
    return so3_;
  }

  /// Mutator of translation vector
  ///
  SOPHUS_FUNC Map<Sophus::Vector3<Scalar, Options>>& translation() {
    return translation_;
  }

  /// Accessor of translation vector
  ///
  SOPHUS_FUNC Map<Sophus::Vector3<Scalar, Options>> const& translation() const {
    return translation_;
  }

 protected:
  Map<Sophus::SO3<Scalar>, Options> so3_;
  Map<Sophus::Vector3<Scalar>, Options> translation_;
};

/// Specialization of Eigen::Map for ``SE3 const``; derived from SE3Base.
///
/// Allows us to wrap SE3 objects around POD array.
template <class Scalar_, int Options>
class Map<Sophus::SE3<Scalar_> const, Options>
    : public Sophus::SE3Base<Map<Sophus::SE3<Scalar_> const, Options>> {
 public:
  using Base = Sophus::SE3Base<Map<Sophus::SE3<Scalar_> const, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  using Base::operator*=;
  using Base::operator*;

  SOPHUS_FUNC Map(Scalar const* coeffs)
      : so3_(coeffs),
        translation_(coeffs + Sophus::SO3<Scalar>::num_parameters) {}

  /// Accessor of SO3
  ///
  SOPHUS_FUNC Map<Sophus::SO3<Scalar> const, Options> const& so3() const {
    return so3_;
  }

  /// Accessor of translation vector
  ///
  SOPHUS_FUNC Map<Sophus::Vector3<Scalar> const, Options> const& translation()
      const {
    return translation_;
  }

 protected:
  Map<Sophus::SO3<Scalar> const, Options> const so3_;
  Map<Sophus::Vector3<Scalar> const, Options> const translation_;
};
}  // namespace Eigen

#endif
