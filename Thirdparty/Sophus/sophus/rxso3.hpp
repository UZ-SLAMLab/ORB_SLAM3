/// @file
/// Direct product R X SO(3) - rotation and scaling in 3d.

#ifndef SOPHUS_RXSO3_HPP
#define SOPHUS_RXSO3_HPP

#include "so3.hpp"

namespace Sophus {
template <class Scalar_, int Options = 0>
class RxSO3;
using RxSO3d = RxSO3<double>;
using RxSO3f = RxSO3<float>;
}  // namespace Sophus

namespace Eigen {
namespace internal {

template <class Scalar_, int Options_>
struct traits<Sophus::RxSO3<Scalar_, Options_>> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Eigen::Quaternion<Scalar, Options>;
};

template <class Scalar_, int Options_>
struct traits<Map<Sophus::RxSO3<Scalar_>, Options_>>
    : traits<Sophus::RxSO3<Scalar_, Options_>> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Map<Eigen::Quaternion<Scalar>, Options>;
};

template <class Scalar_, int Options_>
struct traits<Map<Sophus::RxSO3<Scalar_> const, Options_>>
    : traits<Sophus::RxSO3<Scalar_, Options_> const> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Map<Eigen::Quaternion<Scalar> const, Options>;
};
}  // namespace internal
}  // namespace Eigen

namespace Sophus {

/// RxSO3 base type - implements RxSO3 class but is storage agnostic
///
/// This class implements the group ``R+ x SO(3)``, the direct product of the
/// group of positive scalar 3x3 matrices (= isomorph to the positive
/// real numbers) and the three-dimensional special orthogonal group SO(3).
/// Geometrically, it is the group of rotation and scaling in three dimensions.
/// As a matrix groups, RxSO3 consists of matrices of the form ``s * R``
/// where ``R`` is an orthogonal matrix with ``det(R)=1`` and ``s > 0``
/// being a positive real number.
///
/// Internally, RxSO3 is represented by the group of non-zero quaternions.
/// In particular, the scale equals the squared(!) norm of the quaternion ``q``,
/// ``s = |q|^2``. This is a most compact representation since the degrees of
/// freedom (DoF) of RxSO3 (=4) equals the number of internal parameters (=4).
///
/// This class has the explicit class invariant that the scale ``s`` is not
/// too close to zero. Strictly speaking, it must hold that:
///
///   ``quaternion().squaredNorm() >= Constants::epsilon()``.
///
/// In order to obey this condition, group multiplication is implemented with
/// saturation such that a product always has a scale which is equal or greater
/// this threshold.
template <class Derived>
class RxSO3Base {
 public:
  static constexpr int Options = Eigen::internal::traits<Derived>::Options;
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using QuaternionType =
      typename Eigen::internal::traits<Derived>::QuaternionType;
  using QuaternionTemporaryType = Eigen::Quaternion<Scalar, Options>;

  /// Degrees of freedom of manifold, number of dimensions in tangent space
  /// (three for rotation and one for scaling).
  static int constexpr DoF = 4;
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
  /// double scalars with RxSO3 operations.
  template <typename OtherDerived>
  using ReturnScalar = typename Eigen::ScalarBinaryOpTraits<
      Scalar, typename OtherDerived::Scalar>::ReturnType;

  template <typename OtherDerived>
  using RxSO3Product = RxSO3<ReturnScalar<OtherDerived>>;

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
  /// For RxSO(3), it simply returns the rotation matrix corresponding to ``A``.
  ///
  SOPHUS_FUNC Adjoint Adj() const {
    Adjoint res;
    res.setIdentity();
    res.template topLeftCorner<3, 3>() = rotationMatrix();
    return res;
  }

  /// Returns copy of instance casted to NewScalarType.
  ///
  template <class NewScalarType>
  SOPHUS_FUNC RxSO3<NewScalarType> cast() const {
    return RxSO3<NewScalarType>(quaternion().template cast<NewScalarType>());
  }

  /// This provides unsafe read/write access to internal data. RxSO(3) is
  /// represented by an Eigen::Quaternion (four parameters). When using direct
  /// write access, the user needs to take care of that the quaternion is not
  /// set close to zero.
  ///
  /// Note: The first three Scalars represent the imaginary parts, while the
  /// forth Scalar represent the real part.
  ///
  SOPHUS_FUNC Scalar* data() { return quaternion_nonconst().coeffs().data(); }

  /// Const version of data() above.
  ///
  SOPHUS_FUNC Scalar const* data() const {
    return quaternion().coeffs().data();
  }

  /// Returns group inverse.
  ///
  SOPHUS_FUNC RxSO3<Scalar> inverse() const {
    return RxSO3<Scalar>(quaternion().inverse());
  }

  /// Logarithmic map
  ///
  /// Computes the logarithm, the inverse of the group exponential which maps
  /// element of the group (scaled rotation matrices) to elements of the tangent
  /// space (rotation-vector plus logarithm of scale factor).
  ///
  /// To be specific, this function computes ``vee(logmat(.))`` with
  /// ``logmat(.)`` being the matrix logarithm and ``vee(.)`` the vee-operator
  /// of RxSO3.
  ///
  SOPHUS_FUNC Tangent log() const { return logAndTheta().tangent; }

  /// As above, but also returns ``theta = |omega|``.
  ///
  SOPHUS_FUNC TangentAndTheta logAndTheta() const {
    using std::log;

    Scalar scale = quaternion().squaredNorm();
    TangentAndTheta result;
    result.tangent[3] = log(scale);
    auto omega_and_theta = SO3<Scalar>(quaternion()).logAndTheta();
    result.tangent.template head<3>() = omega_and_theta.tangent;
    result.theta = omega_and_theta.theta;
    return result;
  }
  /// Returns 3x3 matrix representation of the instance.
  ///
  /// For RxSO3, the matrix representation is an scaled orthogonal matrix ``sR``
  /// with ``det(R)=s^3``, thus a scaled rotation matrix ``R``  with scale
  /// ``s``.
  ///
  SOPHUS_FUNC Transformation matrix() const {
    Transformation sR;

    Scalar const vx_sq = quaternion().vec().x() * quaternion().vec().x();
    Scalar const vy_sq = quaternion().vec().y() * quaternion().vec().y();
    Scalar const vz_sq = quaternion().vec().z() * quaternion().vec().z();
    Scalar const w_sq = quaternion().w() * quaternion().w();
    Scalar const two_vx = Scalar(2) * quaternion().vec().x();
    Scalar const two_vy = Scalar(2) * quaternion().vec().y();
    Scalar const two_vz = Scalar(2) * quaternion().vec().z();
    Scalar const two_vx_vy = two_vx * quaternion().vec().y();
    Scalar const two_vx_vz = two_vx * quaternion().vec().z();
    Scalar const two_vx_w = two_vx * quaternion().w();
    Scalar const two_vy_vz = two_vy * quaternion().vec().z();
    Scalar const two_vy_w = two_vy * quaternion().w();
    Scalar const two_vz_w = two_vz * quaternion().w();

    sR(0, 0) = vx_sq - vy_sq - vz_sq + w_sq;
    sR(1, 0) = two_vx_vy + two_vz_w;
    sR(2, 0) = two_vx_vz - two_vy_w;

    sR(0, 1) = two_vx_vy - two_vz_w;
    sR(1, 1) = -vx_sq + vy_sq - vz_sq + w_sq;
    sR(2, 1) = two_vx_w + two_vy_vz;

    sR(0, 2) = two_vx_vz + two_vy_w;
    sR(1, 2) = -two_vx_w + two_vy_vz;
    sR(2, 2) = -vx_sq - vy_sq + vz_sq + w_sq;
    return sR;
  }

  /// Assignment-like operator from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC RxSO3Base<Derived>& operator=(
      RxSO3Base<OtherDerived> const& other) {
    quaternion_nonconst() = other.quaternion();
    return *this;
  }

  /// Group multiplication, which is rotation concatenation and scale
  /// multiplication.
  ///
  /// Note: This function performs saturation for products close to zero in
  /// order to ensure the class invariant.
  ///
  template <typename OtherDerived>
  SOPHUS_FUNC RxSO3Product<OtherDerived> operator*(
      RxSO3Base<OtherDerived> const& other) const {
    using ResultT = ReturnScalar<OtherDerived>;
    typename RxSO3Product<OtherDerived>::QuaternionType result_quaternion(
        quaternion() * other.quaternion());

    ResultT scale = result_quaternion.squaredNorm();
    if (scale < Constants<ResultT>::epsilon()) {
      SOPHUS_ENSURE(scale > ResultT(0), "Scale must be greater zero.");
      /// Saturation to ensure class invariant.
      result_quaternion.normalize();
      result_quaternion.coeffs() *= sqrt(Constants<Scalar>::epsilon());
    }
    return RxSO3Product<OtherDerived>(result_quaternion);
  }

  /// Group action on 3-points.
  ///
  /// This function rotates a 3 dimensional point ``p`` by the SO3 element
  ///  ``bar_R_foo`` (= rotation matrix) and scales it by the scale factor
  ///  ``s``:
  ///
  ///   ``p_bar = s * (bar_R_foo * p_foo)``.
  ///
  template <typename PointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<PointDerived, 3>::value>::type>
  SOPHUS_FUNC PointProduct<PointDerived> operator*(
      Eigen::MatrixBase<PointDerived> const& p) const {
    // Follows http:///eigen.tuxfamily.org/bz/show_bug.cgi?id=459
    Scalar scale = quaternion().squaredNorm();
    PointProduct<PointDerived> two_vec_cross_p = quaternion().vec().cross(p);
    two_vec_cross_p += two_vec_cross_p;
    return scale * p + (quaternion().w() * two_vec_cross_p +
                        quaternion().vec().cross(two_vec_cross_p));
  }

  /// Group action on homogeneous 3-points. See above for more details.
  ///
  template <typename HPointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<HPointDerived, 4>::value>::type>
  SOPHUS_FUNC HomogeneousPointProduct<HPointDerived> operator*(
      Eigen::MatrixBase<HPointDerived> const& p) const {
    const auto rsp = *this * p.template head<3>();
    return HomogeneousPointProduct<HPointDerived>(rsp(0), rsp(1), rsp(2), p(3));
  }

  /// Group action on lines.
  ///
  /// This function rotates a parametrized line ``l(t) = o + t * d`` by the SO3
  /// element ans scales it by the scale factor:
  ///
  /// Origin ``o`` is rotated and scaled
  /// Direction ``d`` is rotated (preserving it's norm)
  ///
  SOPHUS_FUNC Line operator*(Line const& l) const {
    return Line((*this) * l.origin(),
                (*this) * l.direction() / quaternion().squaredNorm());
  }

  /// In-place group multiplication. This method is only valid if the return
  /// type of the multiplication is compatible with this SO3's Scalar type.
  ///
  /// Note: This function performs saturation for products close to zero in
  /// order to ensure the class invariant.
  ///
  template <typename OtherDerived,
            typename = typename std::enable_if<
                std::is_same<Scalar, ReturnScalar<OtherDerived>>::value>::type>
  SOPHUS_FUNC RxSO3Base<Derived>& operator*=(
      RxSO3Base<OtherDerived> const& other) {
    *static_cast<Derived*>(this) = *this * other;
    return *this;
  }

  /// Returns internal parameters of RxSO(3).
  ///
  /// It returns (q.imag[0], q.imag[1], q.imag[2], q.real), with q being the
  /// quaternion.
  ///
  SOPHUS_FUNC Sophus::Vector<Scalar, num_parameters> params() const {
    return quaternion().coeffs();
  }

  /// Sets non-zero quaternion
  ///
  /// Precondition: ``quat`` must not be close to zero.
  SOPHUS_FUNC void setQuaternion(Eigen::Quaternion<Scalar> const& quat) {
    SOPHUS_ENSURE(quat.squaredNorm() > Constants<Scalar>::epsilon() *
                                           Constants<Scalar>::epsilon(),
                  "Scale factor must be greater-equal epsilon.");
    static_cast<Derived*>(this)->quaternion_nonconst() = quat;
  }

  /// Accessor of quaternion.
  ///
  SOPHUS_FUNC QuaternionType const& quaternion() const {
    return static_cast<Derived const*>(this)->quaternion();
  }

  /// Returns rotation matrix.
  ///
  SOPHUS_FUNC Transformation rotationMatrix() const {
    QuaternionTemporaryType norm_quad = quaternion();
    norm_quad.normalize();
    return norm_quad.toRotationMatrix();
  }

  /// Returns scale.
  ///
  SOPHUS_FUNC
  Scalar scale() const { return quaternion().squaredNorm(); }

  /// Setter of quaternion using rotation matrix ``R``, leaves scale as is.
  ///
  SOPHUS_FUNC void setRotationMatrix(Transformation const& R) {
    using std::sqrt;
    Scalar saved_scale = scale();
    quaternion_nonconst() = R;
    quaternion_nonconst().coeffs() *= sqrt(saved_scale);
  }

  /// Sets scale and leaves rotation as is.
  ///
  /// Note: This function as a significant computational cost, since it has to
  /// call the square root twice.
  ///
  SOPHUS_FUNC
  void setScale(Scalar const& scale) {
    using std::sqrt;
    quaternion_nonconst().normalize();
    quaternion_nonconst().coeffs() *= sqrt(scale);
  }

  /// Setter of quaternion using scaled rotation matrix ``sR``.
  ///
  /// Precondition: The 3x3 matrix must be "scaled orthogonal"
  ///               and have a positive determinant.
  ///
  SOPHUS_FUNC void setScaledRotationMatrix(Transformation const& sR) {
    Transformation squared_sR = sR * sR.transpose();
    Scalar squared_scale =
        Scalar(1. / 3.) *
        (squared_sR(0, 0) + squared_sR(1, 1) + squared_sR(2, 2));
    SOPHUS_ENSURE(squared_scale >= Constants<Scalar>::epsilon() *
                                       Constants<Scalar>::epsilon(),
                  "Scale factor must be greater-equal epsilon.");
    Scalar scale = sqrt(squared_scale);
    quaternion_nonconst() = sR / scale;
    quaternion_nonconst().coeffs() *= sqrt(scale);
  }

  /// Setter of SO(3) rotations, leaves scale as is.
  ///
  SOPHUS_FUNC void setSO3(SO3<Scalar> const& so3) {
    using std::sqrt;
    Scalar saved_scale = scale();
    quaternion_nonconst() = so3.unit_quaternion();
    quaternion_nonconst().coeffs() *= sqrt(saved_scale);
  }

  SOPHUS_FUNC SO3<Scalar> so3() const { return SO3<Scalar>(quaternion()); }

 private:
  /// Mutator of quaternion is private to ensure class invariant.
  ///
  SOPHUS_FUNC QuaternionType& quaternion_nonconst() {
    return static_cast<Derived*>(this)->quaternion_nonconst();
  }
};

/// RxSO3 using storage; derived from RxSO3Base.
template <class Scalar_, int Options>
class RxSO3 : public RxSO3Base<RxSO3<Scalar_, Options>> {
 public:
  using Base = RxSO3Base<RxSO3<Scalar_, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using QuaternionMember = Eigen::Quaternion<Scalar, Options>;

  /// ``Base`` is friend so quaternion_nonconst can be accessed from ``Base``.
  friend class RxSO3Base<RxSO3<Scalar_, Options>>;

  using Base::operator=;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Default constructor initializes quaternion to identity rotation and scale
  /// to 1.
  ///
  SOPHUS_FUNC RxSO3()
      : quaternion_(Scalar(1), Scalar(0), Scalar(0), Scalar(0)) {}

  /// Copy constructor
  ///
  SOPHUS_FUNC RxSO3(RxSO3 const& other) = default;

  /// Copy-like constructor from OtherDerived
  ///
  template <class OtherDerived>
  SOPHUS_FUNC RxSO3(RxSO3Base<OtherDerived> const& other)
      : quaternion_(other.quaternion()) {}

  /// Constructor from scaled rotation matrix
  ///
  /// Precondition: rotation matrix need to be scaled orthogonal with
  /// determinant of ``s^3``.
  ///
  SOPHUS_FUNC explicit RxSO3(Transformation const& sR) {
    this->setScaledRotationMatrix(sR);
  }

  /// Constructor from scale factor and rotation matrix ``R``.
  ///
  /// Precondition: Rotation matrix ``R`` must to be orthogonal with determinant
  ///               of 1 and ``scale`` must not be close to zero.
  ///
  SOPHUS_FUNC RxSO3(Scalar const& scale, Transformation const& R)
      : quaternion_(R) {
    SOPHUS_ENSURE(scale >= Constants<Scalar>::epsilon(),
                  "Scale factor must be greater-equal epsilon.");
    using std::sqrt;
    quaternion_.coeffs() *= sqrt(scale);
  }

  /// Constructor from scale factor and SO3
  ///
  /// Precondition: ``scale`` must not to be close to zero.
  ///
  SOPHUS_FUNC RxSO3(Scalar const& scale, SO3<Scalar> const& so3)
      : quaternion_(so3.unit_quaternion()) {
    SOPHUS_ENSURE(scale >= Constants<Scalar>::epsilon(),
                  "Scale factor must be greater-equal epsilon.");
    using std::sqrt;
    quaternion_.coeffs() *= sqrt(scale);
  }

  /// Constructor from quaternion
  ///
  /// Precondition: quaternion must not be close to zero.
  ///
  template <class D>
  SOPHUS_FUNC explicit RxSO3(Eigen::QuaternionBase<D> const& quat)
      : quaternion_(quat) {
    static_assert(std::is_same<typename D::Scalar, Scalar>::value,
                  "must be same Scalar type.");
    SOPHUS_ENSURE(quaternion_.squaredNorm() >= Constants<Scalar>::epsilon(),
                  "Scale factor must be greater-equal epsilon.");
  }

  /// Accessor of quaternion.
  ///
  SOPHUS_FUNC QuaternionMember const& quaternion() const { return quaternion_; }

  /// Returns derivative of exp(x).matrix() wrt. ``x_i at x=0``.
  ///
  SOPHUS_FUNC static Transformation Dxi_exp_x_matrix_at_0(int i) {
    return generator(i);
  }
  /// Group exponential
  ///
  /// This functions takes in an element of tangent space (= rotation 3-vector
  /// plus logarithm of scale) and returns the corresponding element of the
  /// group RxSO3.
  ///
  /// To be more specific, thixs function computes ``expmat(hat(omega))``
  /// with ``expmat(.)`` being the matrix exponential and ``hat(.)`` being the
  /// hat()-operator of RSO3.
  ///
  SOPHUS_FUNC static RxSO3<Scalar> exp(Tangent const& a) {
    Scalar theta;
    return expAndTheta(a, &theta);
  }

  /// As above, but also returns ``theta = |omega|`` as out-parameter.
  ///
  /// Precondition: ``theta`` must not be ``nullptr``.
  ///
  SOPHUS_FUNC static RxSO3<Scalar> expAndTheta(Tangent const& a,
                                               Scalar* theta) {
    SOPHUS_ENSURE(theta != nullptr, "must not be nullptr.");
    using std::exp;
    using std::sqrt;

    Vector3<Scalar> const omega = a.template head<3>();
    Scalar sigma = a[3];
    Scalar sqrt_scale = sqrt(exp(sigma));
    Eigen::Quaternion<Scalar> quat =
        SO3<Scalar>::expAndTheta(omega, theta).unit_quaternion();
    quat.coeffs() *= sqrt_scale;
    return RxSO3<Scalar>(quat);
  }

  /// Returns the ith infinitesimal generators of ``R+ x SO(3)``.
  ///
  /// The infinitesimal generators of RxSO3 are:
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
  ///
  ///         |  1  0  0 |
  ///   G_2 = |  0  1  0 |
  ///         |  0  0  1 |
  /// ```
  ///
  /// Precondition: ``i`` must be 0, 1, 2 or 3.
  ///
  SOPHUS_FUNC static Transformation generator(int i) {
    SOPHUS_ENSURE(i >= 0 && i <= 3, "i should be in range [0,3].");
    Tangent e;
    e.setZero();
    e[i] = Scalar(1);
    return hat(e);
  }

  /// hat-operator
  ///
  /// It takes in the 4-vector representation ``a`` (= rotation vector plus
  /// logarithm of scale) and  returns the corresponding matrix representation
  /// of Lie algebra element.
  ///
  /// Formally, the hat()-operator of RxSO3 is defined as
  ///
  ///   ``hat(.): R^4 -> R^{3x3},  hat(a) = sum_i a_i * G_i``  (for i=0,1,2,3)
  ///
  /// with ``G_i`` being the ith infinitesimal generator of RxSO3.
  ///
  /// The corresponding inverse is the vee()-operator, see below.
  ///
  SOPHUS_FUNC static Transformation hat(Tangent const& a) {
    Transformation A;
    // clang-format off
    A <<  a(3), -a(2),  a(1),
          a(2),  a(3), -a(0),
         -a(1),  a(0),  a(3);
    // clang-format on
    return A;
  }

  /// Lie bracket
  ///
  /// It computes the Lie bracket of RxSO(3). To be more specific, it computes
  ///
  ///   ``[omega_1, omega_2]_rxso3 := vee([hat(omega_1), hat(omega_2)])``
  ///
  /// with ``[A,B] := AB-BA`` being the matrix commutator, ``hat(.)`` the
  /// hat()-operator and ``vee(.)`` the vee()-operator of RxSO3.
  ///
  SOPHUS_FUNC static Tangent lieBracket(Tangent const& a, Tangent const& b) {
    Vector3<Scalar> const omega1 = a.template head<3>();
    Vector3<Scalar> const omega2 = b.template head<3>();
    Vector4<Scalar> res;
    res.template head<3>() = omega1.cross(omega2);
    res[3] = Scalar(0);
    return res;
  }

  /// Draw uniform sample from RxSO(3) manifold.
  ///
  /// The scale factor is drawn uniformly in log2-space from [-1, 1],
  /// hence the scale is in [0.5, 2].
  ///
  template <class UniformRandomBitGenerator>
  static RxSO3 sampleUniform(UniformRandomBitGenerator& generator) {
    std::uniform_real_distribution<Scalar> uniform(Scalar(-1), Scalar(1));
    using std::exp2;
    return RxSO3(exp2(uniform(generator)),
                 SO3<Scalar>::sampleUniform(generator));
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
  ///                |  d -c  b |
  ///                |  c  d -a |
  ///                | -b  a  d |
  ///
  SOPHUS_FUNC static Tangent vee(Transformation const& Omega) {
    using std::abs;
    return Tangent(Omega(2, 1), Omega(0, 2), Omega(1, 0), Omega(0, 0));
  }

 protected:
  SOPHUS_FUNC QuaternionMember& quaternion_nonconst() { return quaternion_; }

  QuaternionMember quaternion_;
};

}  // namespace Sophus

namespace Eigen {

/// Specialization of Eigen::Map for ``RxSO3``; derived from RxSO3Base
///
/// Allows us to wrap RxSO3 objects around POD array (e.g. external c style
/// quaternion).
template <class Scalar_, int Options>
class Map<Sophus::RxSO3<Scalar_>, Options>
    : public Sophus::RxSO3Base<Map<Sophus::RxSO3<Scalar_>, Options>> {
 public:
  using Base = Sophus::RxSO3Base<Map<Sophus::RxSO3<Scalar_>, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  /// ``Base`` is friend so quaternion_nonconst can be accessed from ``Base``.
  friend class Sophus::RxSO3Base<Map<Sophus::RxSO3<Scalar_>, Options>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  SOPHUS_FUNC Map(Scalar* coeffs) : quaternion_(coeffs) {}

  /// Accessor of quaternion.
  ///
  SOPHUS_FUNC
  Map<Eigen::Quaternion<Scalar>, Options> const& quaternion() const {
    return quaternion_;
  }

 protected:
  SOPHUS_FUNC Map<Eigen::Quaternion<Scalar>, Options>& quaternion_nonconst() {
    return quaternion_;
  }

  Map<Eigen::Quaternion<Scalar>, Options> quaternion_;
};

/// Specialization of Eigen::Map for ``RxSO3 const``; derived from RxSO3Base.
///
/// Allows us to wrap RxSO3 objects around POD array (e.g. external c style
/// quaternion).
template <class Scalar_, int Options>
class Map<Sophus::RxSO3<Scalar_> const, Options>
    : public Sophus::RxSO3Base<Map<Sophus::RxSO3<Scalar_> const, Options>> {
 public:
  using Base = Sophus::RxSO3Base<Map<Sophus::RxSO3<Scalar_> const, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  using Base::operator*=;
  using Base::operator*;

  SOPHUS_FUNC
  Map(Scalar const* coeffs) : quaternion_(coeffs) {}

  /// Accessor of quaternion.
  ///
  SOPHUS_FUNC
  Map<Eigen::Quaternion<Scalar> const, Options> const& quaternion() const {
    return quaternion_;
  }

 protected:
  Map<Eigen::Quaternion<Scalar> const, Options> const quaternion_;
};
}  // namespace Eigen

#endif  /// SOPHUS_RXSO3_HPP
