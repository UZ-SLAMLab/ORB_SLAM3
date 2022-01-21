/// @file
/// Similarity group Sim(3) - scaling, rotation and translation in 3d.

#ifndef SOPHUS_SIM3_HPP
#define SOPHUS_SIM3_HPP

#include "rxso3.hpp"
#include "sim_details.hpp"

namespace Sophus {
template <class Scalar_, int Options = 0>
class Sim3;
using Sim3d = Sim3<double>;
using Sim3f = Sim3<float>;
}  // namespace Sophus

namespace Eigen {
namespace internal {

template <class Scalar_, int Options>
struct traits<Sophus::Sim3<Scalar_, Options>> {
  using Scalar = Scalar_;
  using TranslationType = Sophus::Vector3<Scalar, Options>;
  using RxSO3Type = Sophus::RxSO3<Scalar, Options>;
};

template <class Scalar_, int Options>
struct traits<Map<Sophus::Sim3<Scalar_>, Options>>
    : traits<Sophus::Sim3<Scalar_, Options>> {
  using Scalar = Scalar_;
  using TranslationType = Map<Sophus::Vector3<Scalar>, Options>;
  using RxSO3Type = Map<Sophus::RxSO3<Scalar>, Options>;
};

template <class Scalar_, int Options>
struct traits<Map<Sophus::Sim3<Scalar_> const, Options>>
    : traits<Sophus::Sim3<Scalar_, Options> const> {
  using Scalar = Scalar_;
  using TranslationType = Map<Sophus::Vector3<Scalar> const, Options>;
  using RxSO3Type = Map<Sophus::RxSO3<Scalar> const, Options>;
};
}  // namespace internal
}  // namespace Eigen

namespace Sophus {

/// Sim3 base type - implements Sim3 class but is storage agnostic.
///
/// Sim(3) is the group of rotations  and translation and scaling in 3d. It is
/// the semi-direct product of R+xSO(3) and the 3d Euclidean vector space.  The
/// class is represented using a composition of RxSO3  for scaling plus
/// rotation and a 3-vector for translation.
///
/// Sim(3) is neither compact, nor a commutative group.
///
/// See RxSO3 for more details of the scaling + rotation representation in 3d.
///
template <class Derived>
class Sim3Base {
 public:
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using TranslationType =
      typename Eigen::internal::traits<Derived>::TranslationType;
  using RxSO3Type = typename Eigen::internal::traits<Derived>::RxSO3Type;
  using QuaternionType = typename RxSO3Type::QuaternionType;

  /// Degrees of freedom of manifold, number of dimensions in tangent space
  /// (three for translation, three for rotation and one for scaling).
  static int constexpr DoF = 7;
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
  /// double scalars with Sim3 operations.
  template <typename OtherDerived>
  using ReturnScalar = typename Eigen::ScalarBinaryOpTraits<
      Scalar, typename OtherDerived::Scalar>::ReturnType;

  template <typename OtherDerived>
  using Sim3Product = Sim3<ReturnScalar<OtherDerived>>;

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
    Matrix3<Scalar> const R = rxso3().rotationMatrix();
    Adjoint res;
    res.setZero();
    res.template block<3, 3>(0, 0) = rxso3().matrix();
    res.template block<3, 3>(0, 3) = SO3<Scalar>::hat(translation()) * R;
    res.template block<3, 1>(0, 6) = -translation();

    res.template block<3, 3>(3, 3) = R;

    res(6, 6) = Scalar(1);
    return res;
  }

  /// Returns copy of instance casted to NewScalarType.
  ///
  template <class NewScalarType>
  SOPHUS_FUNC Sim3<NewScalarType> cast() const {
    return Sim3<NewScalarType>(rxso3().template cast<NewScalarType>(),
                               translation().template cast<NewScalarType>());
  }

  /// Returns group inverse.
  ///
  SOPHUS_FUNC Sim3<Scalar> inverse() const {
    RxSO3<Scalar> invR = rxso3().inverse();
    return Sim3<Scalar>(invR, invR * (translation() * Scalar(-1)));
  }

  /// Logarithmic map
  ///
  /// Computes the logarithm, the inverse of the group exponential which maps
  /// element of the group (rigid body transformations) to elements of the
  /// tangent space (twist).
  ///
  /// To be specific, this function computes ``vee(logmat(.))`` with
  /// ``logmat(.)`` being the matrix logarithm and ``vee(.)`` the vee-operator
  /// of Sim(3).
  ///
  SOPHUS_FUNC Tangent log() const {
    // The derivation of the closed-form Sim(3) logarithm for is done
    // analogously to the closed-form solution of the SE(3) logarithm, see
    // J. Gallier, D. Xu, "Computing exponentials of skew symmetric matrices
    // and logarithms of orthogonal matrices", IJRA 2002.
    // https:///pdfs.semanticscholar.org/cfe3/e4b39de63c8cabd89bf3feff7f5449fc981d.pdf
    // (Sec. 6., pp. 8)
    Tangent res;
    auto omega_sigma_and_theta = rxso3().logAndTheta();
    Vector3<Scalar> const omega =
        omega_sigma_and_theta.tangent.template head<3>();
    Scalar sigma = omega_sigma_and_theta.tangent[3];
    Matrix3<Scalar> const Omega = SO3<Scalar>::hat(omega);
    Matrix3<Scalar> const W_inv = details::calcWInv<Scalar, 3>(
        Omega, omega_sigma_and_theta.theta, sigma, scale());

    res.segment(0, 3) = W_inv * translation();
    res.segment(3, 3) = omega;
    res[6] = sigma;
    return res;
  }

  /// Returns 4x4 matrix representation of the instance.
  ///
  /// It has the following form:
  ///
  ///     | s*R t |
  ///     |  o  1 |
  ///
  /// where ``R`` is a 3x3 rotation matrix, ``s`` a scale factor, ``t`` a
  /// translation 3-vector and ``o`` a 3-column vector of zeros.
  ///
  SOPHUS_FUNC Transformation matrix() const {
    Transformation homogenious_matrix;
    homogenious_matrix.template topLeftCorner<3, 4>() = matrix3x4();
    homogenious_matrix.row(3) =
        Matrix<Scalar, 4, 1>(Scalar(0), Scalar(0), Scalar(0), Scalar(1));
    return homogenious_matrix;
  }

  /// Returns the significant first three rows of the matrix above.
  ///
  SOPHUS_FUNC Matrix<Scalar, 3, 4> matrix3x4() const {
    Matrix<Scalar, 3, 4> matrix;
    matrix.template topLeftCorner<3, 3>() = rxso3().matrix();
    matrix.col(3) = translation();
    return matrix;
  }

  /// Assignment-like operator from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC Sim3Base<Derived>& operator=(
      Sim3Base<OtherDerived> const& other) {
    rxso3() = other.rxso3();
    translation() = other.translation();
    return *this;
  }

  /// Group multiplication, which is rotation plus scaling concatenation.
  ///
  /// Note: That scaling is calculated with saturation. See RxSO3 for
  /// details.
  ///
  template <typename OtherDerived>
  SOPHUS_FUNC Sim3Product<OtherDerived> operator*(
      Sim3Base<OtherDerived> const& other) const {
    return Sim3Product<OtherDerived>(
        rxso3() * other.rxso3(), translation() + rxso3() * other.translation());
  }

  /// Group action on 3-points.
  ///
  /// This function rotates, scales and translates a three dimensional point
  /// ``p`` by the Sim(3) element ``(bar_sR_foo, t_bar)`` (= similarity
  /// transformation):
  ///
  ///   ``p_bar = bar_sR_foo * p_foo + t_bar``.
  ///
  template <typename PointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<PointDerived, 3>::value>::type>
  SOPHUS_FUNC PointProduct<PointDerived> operator*(
      Eigen::MatrixBase<PointDerived> const& p) const {
    return rxso3() * p + translation();
  }

  /// Group action on homogeneous 3-points. See above for more details.
  ///
  template <typename HPointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<HPointDerived, 4>::value>::type>
  SOPHUS_FUNC HomogeneousPointProduct<HPointDerived> operator*(
      Eigen::MatrixBase<HPointDerived> const& p) const {
    const PointProduct<HPointDerived> tp =
        rxso3() * p.template head<3>() + p(3) * translation();
    return HomogeneousPointProduct<HPointDerived>(tp(0), tp(1), tp(2), p(3));
  }

  /// Group action on lines.
  ///
  /// This function rotates, scales and translates a parametrized line
  /// ``l(t) = o + t * d`` by the Sim(3) element:
  ///
  /// Origin ``o`` is rotated, scaled and translated
  /// Direction ``d`` is rotated
  ///
  SOPHUS_FUNC Line operator*(Line const& l) const {
    Line rotatedLine = rxso3() * l;
    return Line(rotatedLine.origin() + translation(), rotatedLine.direction());
  }

  /// In-place group multiplication. This method is only valid if the return
  /// type of the multiplication is compatible with this SO3's Scalar type.
  ///
  template <typename OtherDerived,
            typename = typename std::enable_if<
                std::is_same<Scalar, ReturnScalar<OtherDerived>>::value>::type>
  SOPHUS_FUNC Sim3Base<Derived>& operator*=(
      Sim3Base<OtherDerived> const& other) {
    *static_cast<Derived*>(this) = *this * other;
    return *this;
  }

  /// Returns internal parameters of Sim(3).
  ///
  /// It returns (q.imag[0], q.imag[1], q.imag[2], q.real, t[0], t[1], t[2]),
  /// with q being the quaternion, t the translation 3-vector.
  ///
  SOPHUS_FUNC Sophus::Vector<Scalar, num_parameters> params() const {
    Sophus::Vector<Scalar, num_parameters> p;
    p << rxso3().params(), translation();
    return p;
  }

  /// Setter of non-zero quaternion.
  ///
  /// Precondition: ``quat`` must not be close to zero.
  ///
  SOPHUS_FUNC void setQuaternion(Eigen::Quaternion<Scalar> const& quat) {
    rxso3().setQuaternion(quat);
  }

  /// Accessor of quaternion.
  ///
  SOPHUS_FUNC QuaternionType const& quaternion() const {
    return rxso3().quaternion();
  }

  /// Returns Rotation matrix
  ///
  SOPHUS_FUNC Matrix3<Scalar> rotationMatrix() const {
    return rxso3().rotationMatrix();
  }

  /// Mutator of SO3 group.
  ///
  SOPHUS_FUNC RxSO3Type& rxso3() {
    return static_cast<Derived*>(this)->rxso3();
  }

  /// Accessor of SO3 group.
  ///
  SOPHUS_FUNC RxSO3Type const& rxso3() const {
    return static_cast<Derived const*>(this)->rxso3();
  }

  /// Returns scale.
  ///
  SOPHUS_FUNC Scalar scale() const { return rxso3().scale(); }

  /// Setter of quaternion using rotation matrix ``R``, leaves scale as is.
  ///
  SOPHUS_FUNC void setRotationMatrix(Matrix3<Scalar>& R) {
    rxso3().setRotationMatrix(R);
  }

  /// Sets scale and leaves rotation as is.
  ///
  /// Note: This function as a significant computational cost, since it has to
  /// call the square root twice.
  ///
  SOPHUS_FUNC void setScale(Scalar const& scale) { rxso3().setScale(scale); }

  /// Setter of quaternion using scaled rotation matrix ``sR``.
  ///
  /// Precondition: The 3x3 matrix must be "scaled orthogonal"
  ///               and have a positive determinant.
  ///
  SOPHUS_FUNC void setScaledRotationMatrix(Matrix3<Scalar> const& sR) {
    rxso3().setScaledRotationMatrix(sR);
  }

  /// Mutator of translation vector
  ///
  SOPHUS_FUNC TranslationType& translation() {
    return static_cast<Derived*>(this)->translation();
  }

  /// Accessor of translation vector
  ///
  SOPHUS_FUNC TranslationType const& translation() const {
    return static_cast<Derived const*>(this)->translation();
  }
};

/// Sim3 using default storage; derived from Sim3Base.
template <class Scalar_, int Options>
class Sim3 : public Sim3Base<Sim3<Scalar_, Options>> {
 public:
  using Base = Sim3Base<Sim3<Scalar_, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using RxSo3Member = RxSO3<Scalar, Options>;
  using TranslationMember = Vector3<Scalar, Options>;

  using Base::operator=;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Default constructor initializes similarity transform to the identity.
  ///
  SOPHUS_FUNC Sim3();

  /// Copy constructor
  ///
  SOPHUS_FUNC Sim3(Sim3 const& other) = default;

  /// Copy-like constructor from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC Sim3(Sim3Base<OtherDerived> const& other)
      : rxso3_(other.rxso3()), translation_(other.translation()) {
    static_assert(std::is_same<typename OtherDerived::Scalar, Scalar>::value,
                  "must be same Scalar type");
  }

  /// Constructor from RxSO3 and translation vector
  ///
  template <class OtherDerived, class D>
  SOPHUS_FUNC Sim3(RxSO3Base<OtherDerived> const& rxso3,
                   Eigen::MatrixBase<D> const& translation)
      : rxso3_(rxso3), translation_(translation) {
    static_assert(std::is_same<typename OtherDerived::Scalar, Scalar>::value,
                  "must be same Scalar type");
    static_assert(std::is_same<typename D::Scalar, Scalar>::value,
                  "must be same Scalar type");
  }

  /// Constructor from quaternion and translation vector.
  ///
  /// Precondition: quaternion must not be close to zero.
  ///
  template <class D1, class D2>
  SOPHUS_FUNC Sim3(Eigen::QuaternionBase<D1> const& quaternion,
                   Eigen::MatrixBase<D2> const& translation)
      : rxso3_(quaternion), translation_(translation) {
    static_assert(std::is_same<typename D1::Scalar, Scalar>::value,
                  "must be same Scalar type");
    static_assert(std::is_same<typename D2::Scalar, Scalar>::value,
                  "must be same Scalar type");
  }

  /// Constructor from 4x4 matrix
  ///
  /// Precondition: Top-left 3x3 matrix needs to be "scaled-orthogonal" with
  ///               positive determinant. The last row must be ``(0, 0, 0, 1)``.
  ///
  SOPHUS_FUNC explicit Sim3(Matrix<Scalar, 4, 4> const& T)
      : rxso3_(T.template topLeftCorner<3, 3>()),
        translation_(T.template block<3, 1>(0, 3)) {}

  /// This provides unsafe read/write access to internal data. Sim(3) is
  /// represented by an Eigen::Quaternion (four parameters) and a 3-vector. When
  /// using direct write access, the user needs to take care of that the
  /// quaternion is not set close to zero.
  ///
  SOPHUS_FUNC Scalar* data() {
    // rxso3_ and translation_ are laid out sequentially with no padding
    return rxso3_.data();
  }

  /// Const version of data() above.
  ///
  SOPHUS_FUNC Scalar const* data() const {
    // rxso3_ and translation_ are laid out sequentially with no padding
    return rxso3_.data();
  }

  /// Accessor of RxSO3
  ///
  SOPHUS_FUNC RxSo3Member& rxso3() { return rxso3_; }

  /// Mutator of RxSO3
  ///
  SOPHUS_FUNC RxSo3Member const& rxso3() const { return rxso3_; }

  /// Mutator of translation vector
  ///
  SOPHUS_FUNC TranslationMember& translation() { return translation_; }

  /// Accessor of translation vector
  ///
  SOPHUS_FUNC TranslationMember const& translation() const {
    return translation_;
  }

  /// Returns derivative of exp(x).matrix() wrt. ``x_i at x=0``.
  ///
  SOPHUS_FUNC static Transformation Dxi_exp_x_matrix_at_0(int i) {
    return generator(i);
  }

  /// Group exponential
  ///
  /// This functions takes in an element of tangent space and returns the
  /// corresponding element of the group Sim(3).
  ///
  /// The first three components of ``a`` represent the translational part
  /// ``upsilon`` in the tangent space of Sim(3), the following three components
  /// of ``a`` represents the rotation vector ``omega`` and the final component
  /// represents the logarithm of the scaling factor ``sigma``.
  /// To be more specific, this function computes ``expmat(hat(a))`` with
  /// ``expmat(.)`` being the matrix exponential and ``hat(.)`` the hat-operator
  /// of Sim(3), see below.
  ///
  SOPHUS_FUNC static Sim3<Scalar> exp(Tangent const& a) {
    // For the derivation of the exponential map of Sim(3) see
    // H. Strasdat, "Local Accuracy and Global Consistency for Efficient Visual
    // SLAM", PhD thesis, 2012.
    // http:///hauke.strasdat.net/files/strasdat_thesis_2012.pdf (A.5, pp. 186)
    Vector3<Scalar> const upsilon = a.segment(0, 3);
    Vector3<Scalar> const omega = a.segment(3, 3);
    Scalar const sigma = a[6];
    Scalar theta;
    RxSO3<Scalar> rxso3 =
        RxSO3<Scalar>::expAndTheta(a.template tail<4>(), &theta);
    Matrix3<Scalar> const Omega = SO3<Scalar>::hat(omega);

    Matrix3<Scalar> const W = details::calcW<Scalar, 3>(Omega, theta, sigma);
    return Sim3<Scalar>(rxso3, W * upsilon);
  }

  /// Returns the ith infinitesimal generators of Sim(3).
  ///
  /// The infinitesimal generators of Sim(3) are:
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
  ///
  ///         |  1  0  0  0 |
  ///   G_6 = |  0  1  0  0 |
  ///         |  0  0  1  0 |
  ///         |  0  0  0  0 |
  /// ```
  ///
  /// Precondition: ``i`` must be in [0, 6].
  ///
  SOPHUS_FUNC static Transformation generator(int i) {
    SOPHUS_ENSURE(i >= 0 || i <= 6, "i should be in range [0,6].");
    Tangent e;
    e.setZero();
    e[i] = Scalar(1);
    return hat(e);
  }

  /// hat-operator
  ///
  /// It takes in the 7-vector representation and returns the corresponding
  /// matrix representation of Lie algebra element.
  ///
  /// Formally, the hat()-operator of Sim(3) is defined as
  ///
  ///   ``hat(.): R^7 -> R^{4x4},  hat(a) = sum_i a_i * G_i``  (for i=0,...,6)
  ///
  /// with ``G_i`` being the ith infinitesimal generator of Sim(3).
  ///
  /// The corresponding inverse is the vee()-operator, see below.
  ///
  SOPHUS_FUNC static Transformation hat(Tangent const& a) {
    Transformation Omega;
    Omega.template topLeftCorner<3, 3>() =
        RxSO3<Scalar>::hat(a.template tail<4>());
    Omega.col(3).template head<3>() = a.template head<3>();
    Omega.row(3).setZero();
    return Omega;
  }

  /// Lie bracket
  ///
  /// It computes the Lie bracket of Sim(3). To be more specific, it computes
  ///
  ///   ``[omega_1, omega_2]_sim3 := vee([hat(omega_1), hat(omega_2)])``
  ///
  /// with ``[A,B] := AB-BA`` being the matrix commutator, ``hat(.)`` the
  /// hat()-operator and ``vee(.)`` the vee()-operator of Sim(3).
  ///
  SOPHUS_FUNC static Tangent lieBracket(Tangent const& a, Tangent const& b) {
    Vector3<Scalar> const upsilon1 = a.template head<3>();
    Vector3<Scalar> const upsilon2 = b.template head<3>();
    Vector3<Scalar> const omega1 = a.template segment<3>(3);
    Vector3<Scalar> const omega2 = b.template segment<3>(3);
    Scalar sigma1 = a[6];
    Scalar sigma2 = b[6];

    Tangent res;
    res.template head<3>() = SO3<Scalar>::hat(omega1) * upsilon2 +
                             SO3<Scalar>::hat(upsilon1) * omega2 +
                             sigma1 * upsilon2 - sigma2 * upsilon1;
    res.template segment<3>(3) = omega1.cross(omega2);
    res[6] = Scalar(0);

    return res;
  }

  /// Draw uniform sample from Sim(3) manifold.
  ///
  /// Translations are drawn component-wise from the range [-1, 1].
  /// The scale factor is drawn uniformly in log2-space from [-1, 1],
  /// hence the scale is in [0.5, 2].
  ///
  template <class UniformRandomBitGenerator>
  static Sim3 sampleUniform(UniformRandomBitGenerator& generator) {
    std::uniform_real_distribution<Scalar> uniform(Scalar(-1), Scalar(1));
    return Sim3(RxSO3<Scalar>::sampleUniform(generator),
                Vector3<Scalar>(uniform(generator), uniform(generator),
                                uniform(generator)));
  }

  /// vee-operator
  ///
  /// It takes the 4x4-matrix representation ``Omega`` and maps it to the
  /// corresponding 7-vector representation of Lie algebra.
  ///
  /// This is the inverse of the hat()-operator, see above.
  ///
  /// Precondition: ``Omega`` must have the following structure:
  ///
  ///                |  g -f  e  a |
  ///                |  f  g -d  b |
  ///                | -e  d  g  c |
  ///                |  0  0  0  0 |
  ///
  SOPHUS_FUNC static Tangent vee(Transformation const& Omega) {
    Tangent upsilon_omega_sigma;
    upsilon_omega_sigma.template head<3>() = Omega.col(3).template head<3>();
    upsilon_omega_sigma.template tail<4>() =
        RxSO3<Scalar>::vee(Omega.template topLeftCorner<3, 3>());
    return upsilon_omega_sigma;
  }

 protected:
  RxSo3Member rxso3_;
  TranslationMember translation_;
};

template <class Scalar, int Options>
Sim3<Scalar, Options>::Sim3() : translation_(TranslationMember::Zero()) {
  static_assert(std::is_standard_layout<Sim3>::value,
                "Assume standard layout for the use of offsetof check below.");
  static_assert(
      offsetof(Sim3, rxso3_) + sizeof(Scalar) * RxSO3<Scalar>::num_parameters ==
          offsetof(Sim3, translation_),
      "This class assumes packed storage and hence will only work "
      "correctly depending on the compiler (options) - in "
      "particular when using [this->data(), this-data() + "
      "num_parameters] to access the raw data in a contiguous fashion.");
}

}  // namespace Sophus

namespace Eigen {

/// Specialization of Eigen::Map for ``Sim3``; derived from Sim3Base.
///
/// Allows us to wrap Sim3 objects around POD array.
template <class Scalar_, int Options>
class Map<Sophus::Sim3<Scalar_>, Options>
    : public Sophus::Sim3Base<Map<Sophus::Sim3<Scalar_>, Options>> {
 public:
  using Base = Sophus::Sim3Base<Map<Sophus::Sim3<Scalar_>, Options>>;
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
      : rxso3_(coeffs),
        translation_(coeffs + Sophus::RxSO3<Scalar>::num_parameters) {}

  /// Mutator of RxSO3
  ///
  SOPHUS_FUNC Map<Sophus::RxSO3<Scalar>, Options>& rxso3() { return rxso3_; }

  /// Accessor of RxSO3
  ///
  SOPHUS_FUNC Map<Sophus::RxSO3<Scalar>, Options> const& rxso3() const {
    return rxso3_;
  }

  /// Mutator of translation vector
  ///
  SOPHUS_FUNC Map<Sophus::Vector3<Scalar>, Options>& translation() {
    return translation_;
  }

  /// Accessor of translation vector
  SOPHUS_FUNC Map<Sophus::Vector3<Scalar>, Options> const& translation() const {
    return translation_;
  }

 protected:
  Map<Sophus::RxSO3<Scalar>, Options> rxso3_;
  Map<Sophus::Vector3<Scalar>, Options> translation_;
};

/// Specialization of Eigen::Map for ``Sim3 const``; derived from Sim3Base.
///
/// Allows us to wrap RxSO3 objects around POD array.
template <class Scalar_, int Options>
class Map<Sophus::Sim3<Scalar_> const, Options>
    : public Sophus::Sim3Base<Map<Sophus::Sim3<Scalar_> const, Options>> {
  using Base = Sophus::Sim3Base<Map<Sophus::Sim3<Scalar_> const, Options>>;

 public:
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  using Base::operator*=;
  using Base::operator*;

  SOPHUS_FUNC Map(Scalar const* coeffs)
      : rxso3_(coeffs),
        translation_(coeffs + Sophus::RxSO3<Scalar>::num_parameters) {}

  /// Accessor of RxSO3
  ///
  SOPHUS_FUNC Map<Sophus::RxSO3<Scalar> const, Options> const& rxso3() const {
    return rxso3_;
  }

  /// Accessor of translation vector
  ///
  SOPHUS_FUNC Map<Sophus::Vector3<Scalar> const, Options> const& translation()
      const {
    return translation_;
  }

 protected:
  Map<Sophus::RxSO3<Scalar> const, Options> const rxso3_;
  Map<Sophus::Vector3<Scalar> const, Options> const translation_;
};
}  // namespace Eigen

#endif
