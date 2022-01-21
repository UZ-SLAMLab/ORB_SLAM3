/// @file
/// Similarity group Sim(2) - scaling, rotation and translation in 2d.

#ifndef SOPHUS_SIM2_HPP
#define SOPHUS_SIM2_HPP

#include "rxso2.hpp"
#include "sim_details.hpp"

namespace Sophus {
template <class Scalar_, int Options = 0>
class Sim2;
using Sim2d = Sim2<double>;
using Sim2f = Sim2<float>;
}  // namespace Sophus

namespace Eigen {
namespace internal {

template <class Scalar_, int Options>
struct traits<Sophus::Sim2<Scalar_, Options>> {
  using Scalar = Scalar_;
  using TranslationType = Sophus::Vector2<Scalar, Options>;
  using RxSO2Type = Sophus::RxSO2<Scalar, Options>;
};

template <class Scalar_, int Options>
struct traits<Map<Sophus::Sim2<Scalar_>, Options>>
    : traits<Sophus::Sim2<Scalar_, Options>> {
  using Scalar = Scalar_;
  using TranslationType = Map<Sophus::Vector2<Scalar>, Options>;
  using RxSO2Type = Map<Sophus::RxSO2<Scalar>, Options>;
};

template <class Scalar_, int Options>
struct traits<Map<Sophus::Sim2<Scalar_> const, Options>>
    : traits<Sophus::Sim2<Scalar_, Options> const> {
  using Scalar = Scalar_;
  using TranslationType = Map<Sophus::Vector2<Scalar> const, Options>;
  using RxSO2Type = Map<Sophus::RxSO2<Scalar> const, Options>;
};
}  // namespace internal
}  // namespace Eigen

namespace Sophus {

/// Sim2 base type - implements Sim2 class but is storage agnostic.
///
/// Sim(2) is the group of rotations  and translation and scaling in 2d. It is
/// the semi-direct product of R+xSO(2) and the 2d Euclidean vector space. The
/// class is represented using a composition of RxSO2  for scaling plus
/// rotation and a 2-vector for translation.
///
/// Sim(2) is neither compact, nor a commutative group.
///
/// See RxSO2 for more details of the scaling + rotation representation in 2d.
///
template <class Derived>
class Sim2Base {
 public:
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using TranslationType =
      typename Eigen::internal::traits<Derived>::TranslationType;
  using RxSO2Type = typename Eigen::internal::traits<Derived>::RxSO2Type;

  /// Degrees of freedom of manifold, number of dimensions in tangent space
  /// (two for translation, one for rotation and one for scaling).
  static int constexpr DoF = 4;
  /// Number of internal parameters used (2-tuple for complex number, two for
  /// translation).
  static int constexpr num_parameters = 4;
  /// Group transformations are 3x3 matrices.
  static int constexpr N = 3;
  using Transformation = Matrix<Scalar, N, N>;
  using Point = Vector2<Scalar>;
  using HomogeneousPoint = Vector3<Scalar>;
  using Line = ParametrizedLine2<Scalar>;
  using Tangent = Vector<Scalar, DoF>;
  using Adjoint = Matrix<Scalar, DoF, DoF>;

  /// For binary operations the return type is determined with the
  /// ScalarBinaryOpTraits feature of Eigen. This allows mixing concrete and Map
  /// types, as well as other compatible scalar types such as Ceres::Jet and
  /// double scalars with SIM2 operations.
  template <typename OtherDerived>
  using ReturnScalar = typename Eigen::ScalarBinaryOpTraits<
      Scalar, typename OtherDerived::Scalar>::ReturnType;

  template <typename OtherDerived>
  using Sim2Product = Sim2<ReturnScalar<OtherDerived>>;

  template <typename PointDerived>
  using PointProduct = Vector2<ReturnScalar<PointDerived>>;

  template <typename HPointDerived>
  using HomogeneousPointProduct = Vector3<ReturnScalar<HPointDerived>>;

  /// Adjoint transformation
  ///
  /// This function return the adjoint transformation ``Ad`` of the group
  /// element ``A`` such that for all ``x`` it holds that
  /// ``hat(Ad_A * x) = A * hat(x) A^{-1}``. See hat-operator below.
  ///
  SOPHUS_FUNC Adjoint Adj() const {
    Adjoint res;
    res.setZero();
    res.template block<2, 2>(0, 0) = rxso2().matrix();
    res(0, 2) = translation()[1];
    res(1, 2) = -translation()[0];
    res.template block<2, 1>(0, 3) = -translation();

    res(2, 2) = Scalar(1);

    res(3, 3) = Scalar(1);
    return res;
  }

  /// Returns copy of instance casted to NewScalarType.
  ///
  template <class NewScalarType>
  SOPHUS_FUNC Sim2<NewScalarType> cast() const {
    return Sim2<NewScalarType>(rxso2().template cast<NewScalarType>(),
                               translation().template cast<NewScalarType>());
  }

  /// Returns group inverse.
  ///
  SOPHUS_FUNC Sim2<Scalar> inverse() const {
    RxSO2<Scalar> invR = rxso2().inverse();
    return Sim2<Scalar>(invR, invR * (translation() * Scalar(-1)));
  }

  /// Logarithmic map
  ///
  /// Computes the logarithm, the inverse of the group exponential which maps
  /// element of the group (rigid body transformations) to elements of the
  /// tangent space (twist).
  ///
  /// To be specific, this function computes ``vee(logmat(.))`` with
  /// ``logmat(.)`` being the matrix logarithm and ``vee(.)`` the vee-operator
  /// of Sim(2).
  ///
  SOPHUS_FUNC Tangent log() const {
    /// The derivation of the closed-form Sim(2) logarithm for is done
    /// analogously to the closed-form solution of the SE(2) logarithm, see
    /// J. Gallier, D. Xu, "Computing exponentials of skew symmetric matrices
    /// and logarithms of orthogonal matrices", IJRA 2002.
    /// https:///pdfs.semanticscholar.org/cfe3/e4b39de63c8cabd89bf3feff7f5449fc981d.pdf
    /// (Sec. 6., pp. 8)
    Tangent res;
    Vector2<Scalar> const theta_sigma = rxso2().log();
    Scalar const theta = theta_sigma[0];
    Scalar const sigma = theta_sigma[1];
    Matrix2<Scalar> const Omega = SO2<Scalar>::hat(theta);
    Matrix2<Scalar> const W_inv =
        details::calcWInv<Scalar, 2>(Omega, theta, sigma, scale());

    res.segment(0, 2) = W_inv * translation();
    res[2] = theta;
    res[3] = sigma;
    return res;
  }

  /// Returns 3x3 matrix representation of the instance.
  ///
  /// It has the following form:
  ///
  ///   | s*R t |
  ///   |  o  1 |
  ///
  /// where ``R`` is a 2x2 rotation matrix, ``s`` a scale factor, ``t`` a
  /// translation 2-vector and ``o`` a 2-column vector of zeros.
  ///
  SOPHUS_FUNC Transformation matrix() const {
    Transformation homogenious_matrix;
    homogenious_matrix.template topLeftCorner<2, 3>() = matrix2x3();
    homogenious_matrix.row(2) =
        Matrix<Scalar, 3, 1>(Scalar(0), Scalar(0), Scalar(1));
    return homogenious_matrix;
  }

  /// Returns the significant first two rows of the matrix above.
  ///
  SOPHUS_FUNC Matrix<Scalar, 2, 3> matrix2x3() const {
    Matrix<Scalar, 2, 3> matrix;
    matrix.template topLeftCorner<2, 2>() = rxso2().matrix();
    matrix.col(2) = translation();
    return matrix;
  }

  /// Assignment-like operator from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC Sim2Base<Derived>& operator=(
      Sim2Base<OtherDerived> const& other) {
    rxso2() = other.rxso2();
    translation() = other.translation();
    return *this;
  }

  /// Group multiplication, which is rotation plus scaling concatenation.
  ///
  /// Note: That scaling is calculated with saturation. See RxSO2 for
  /// details.
  ///
  template <typename OtherDerived>
  SOPHUS_FUNC Sim2Product<OtherDerived> operator*(
      Sim2Base<OtherDerived> const& other) const {
    return Sim2Product<OtherDerived>(
        rxso2() * other.rxso2(), translation() + rxso2() * other.translation());
  }

  /// Group action on 2-points.
  ///
  /// This function rotates, scales and translates a two dimensional point
  /// ``p`` by the Sim(2) element ``(bar_sR_foo, t_bar)`` (= similarity
  /// transformation):
  ///
  ///   ``p_bar = bar_sR_foo * p_foo + t_bar``.
  ///
  template <typename PointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<PointDerived, 2>::value>::type>
  SOPHUS_FUNC PointProduct<PointDerived> operator*(
      Eigen::MatrixBase<PointDerived> const& p) const {
    return rxso2() * p + translation();
  }

  /// Group action on homogeneous 2-points. See above for more details.
  ///
  template <typename HPointDerived,
            typename = typename std::enable_if<
                IsFixedSizeVector<HPointDerived, 3>::value>::type>
  SOPHUS_FUNC HomogeneousPointProduct<HPointDerived> operator*(
      Eigen::MatrixBase<HPointDerived> const& p) const {
    const PointProduct<HPointDerived> tp =
        rxso2() * p.template head<2>() + p(2) * translation();
    return HomogeneousPointProduct<HPointDerived>(tp(0), tp(1), p(2));
  }

  /// Group action on lines.
  ///
  /// This function rotates, scales and translates a parametrized line
  /// ``l(t) = o + t * d`` by the Sim(2) element:
  ///
  /// Origin ``o`` is rotated, scaled and translated
  /// Direction ``d`` is rotated
  ///
  SOPHUS_FUNC Line operator*(Line const& l) const {
    Line rotatedLine = rxso2() * l;
    return Line(rotatedLine.origin() + translation(), rotatedLine.direction());
  }

  /// Returns internal parameters of Sim(2).
  ///
  /// It returns (c[0], c[1], t[0], t[1]),
  /// with c being the complex number, t the translation 3-vector.
  ///
  SOPHUS_FUNC Sophus::Vector<Scalar, num_parameters> params() const {
    Sophus::Vector<Scalar, num_parameters> p;
    p << rxso2().params(), translation();
    return p;
  }

  /// In-place group multiplication. This method is only valid if the return
  /// type of the multiplication is compatible with this SO2's Scalar type.
  ///
  template <typename OtherDerived,
            typename = typename std::enable_if<
                std::is_same<Scalar, ReturnScalar<OtherDerived>>::value>::type>
  SOPHUS_FUNC Sim2Base<Derived>& operator*=(
      Sim2Base<OtherDerived> const& other) {
    *static_cast<Derived*>(this) = *this * other;
    return *this;
  }

  /// Setter of non-zero complex number.
  ///
  /// Precondition: ``z`` must not be close to zero.
  ///
  SOPHUS_FUNC void setComplex(Vector2<Scalar> const& z) {
    rxso2().setComplex(z);
  }

  /// Accessor of complex number.
  ///
  SOPHUS_FUNC
  typename Eigen::internal::traits<Derived>::RxSO2Type::ComplexType const&
  complex() const {
    return rxso2().complex();
  }

  /// Returns Rotation matrix
  ///
  SOPHUS_FUNC Matrix2<Scalar> rotationMatrix() const {
    return rxso2().rotationMatrix();
  }

  /// Mutator of SO2 group.
  ///
  SOPHUS_FUNC RxSO2Type& rxso2() {
    return static_cast<Derived*>(this)->rxso2();
  }

  /// Accessor of SO2 group.
  ///
  SOPHUS_FUNC RxSO2Type const& rxso2() const {
    return static_cast<Derived const*>(this)->rxso2();
  }

  /// Returns scale.
  ///
  SOPHUS_FUNC Scalar scale() const { return rxso2().scale(); }

  /// Setter of complex number using rotation matrix ``R``, leaves scale as is.
  ///
  SOPHUS_FUNC void setRotationMatrix(Matrix2<Scalar>& R) {
    rxso2().setRotationMatrix(R);
  }

  /// Sets scale and leaves rotation as is.
  ///
  /// Note: This function as a significant computational cost, since it has to
  /// call the square root twice.
  ///
  SOPHUS_FUNC void setScale(Scalar const& scale) { rxso2().setScale(scale); }

  /// Setter of complexnumber using scaled rotation matrix ``sR``.
  ///
  /// Precondition: The 2x2 matrix must be "scaled orthogonal"
  ///               and have a positive determinant.
  ///
  SOPHUS_FUNC void setScaledRotationMatrix(Matrix2<Scalar> const& sR) {
    rxso2().setScaledRotationMatrix(sR);
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

/// Sim2 using default storage; derived from Sim2Base.
template <class Scalar_, int Options>
class Sim2 : public Sim2Base<Sim2<Scalar_, Options>> {
 public:
  using Base = Sim2Base<Sim2<Scalar_, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using RxSo2Member = RxSO2<Scalar, Options>;
  using TranslationMember = Vector2<Scalar, Options>;

  using Base::operator=;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Default constructor initializes similarity transform to the identity.
  ///
  SOPHUS_FUNC Sim2();

  /// Copy constructor
  ///
  SOPHUS_FUNC Sim2(Sim2 const& other) = default;

  /// Copy-like constructor from OtherDerived.
  ///
  template <class OtherDerived>
  SOPHUS_FUNC Sim2(Sim2Base<OtherDerived> const& other)
      : rxso2_(other.rxso2()), translation_(other.translation()) {
    static_assert(std::is_same<typename OtherDerived::Scalar, Scalar>::value,
                  "must be same Scalar type");
  }

  /// Constructor from RxSO2 and translation vector
  ///
  template <class OtherDerived, class D>
  SOPHUS_FUNC Sim2(RxSO2Base<OtherDerived> const& rxso2,
                   Eigen::MatrixBase<D> const& translation)
      : rxso2_(rxso2), translation_(translation) {
    static_assert(std::is_same<typename OtherDerived::Scalar, Scalar>::value,
                  "must be same Scalar type");
    static_assert(std::is_same<typename D::Scalar, Scalar>::value,
                  "must be same Scalar type");
  }

  /// Constructor from complex number and translation vector.
  ///
  /// Precondition: complex number must not be close to zero.
  ///
  template <class D>
  SOPHUS_FUNC Sim2(Vector2<Scalar> const& complex_number,
                   Eigen::MatrixBase<D> const& translation)
      : rxso2_(complex_number), translation_(translation) {
    static_assert(std::is_same<typename D::Scalar, Scalar>::value,
                  "must be same Scalar type");
  }

  /// Constructor from 3x3 matrix
  ///
  /// Precondition: Top-left 2x2 matrix needs to be "scaled-orthogonal" with
  ///               positive determinant. The last row must be ``(0, 0, 1)``.
  ///
  SOPHUS_FUNC explicit Sim2(Matrix<Scalar, 3, 3> const& T)
      : rxso2_((T.template topLeftCorner<2, 2>()).eval()),
        translation_(T.template block<2, 1>(0, 2)) {}

  /// This provides unsafe read/write access to internal data. Sim(2) is
  /// represented by a complex number (two parameters) and a 2-vector. When
  /// using direct write access, the user needs to take care of that the
  /// complex number is not set close to zero.
  ///
  SOPHUS_FUNC Scalar* data() {
    // rxso2_ and translation_ are laid out sequentially with no padding
    return rxso2_.data();
  }

  /// Const version of data() above.
  ///
  SOPHUS_FUNC Scalar const* data() const {
    // rxso2_ and translation_ are laid out sequentially with no padding
    return rxso2_.data();
  }

  /// Accessor of RxSO2
  ///
  SOPHUS_FUNC RxSo2Member& rxso2() { return rxso2_; }

  /// Mutator of RxSO2
  ///
  SOPHUS_FUNC RxSo2Member const& rxso2() const { return rxso2_; }

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

  /// Derivative of Lie bracket with respect to first element.
  ///
  /// This function returns ``D_a [a, b]`` with ``D_a`` being the
  /// differential operator with respect to ``a``, ``[a, b]`` being the lie
  /// bracket of the Lie algebra sim(2).
  /// See ``lieBracket()`` below.
  ///

  /// Group exponential
  ///
  /// This functions takes in an element of tangent space and returns the
  /// corresponding element of the group Sim(2).
  ///
  /// The first two components of ``a`` represent the translational part
  /// ``upsilon`` in the tangent space of Sim(2), the following two components
  /// of ``a`` represents the rotation ``theta`` and the final component
  /// represents the logarithm of the scaling factor ``sigma``.
  /// To be more specific, this function computes ``expmat(hat(a))`` with
  /// ``expmat(.)`` being the matrix exponential and ``hat(.)`` the hat-operator
  /// of Sim(2), see below.
  ///
  SOPHUS_FUNC static Sim2<Scalar> exp(Tangent const& a) {
    // For the derivation of the exponential map of Sim(N) see
    // H. Strasdat, "Local Accuracy and Global Consistency for Efficient Visual
    // SLAM", PhD thesis, 2012.
    // http:///hauke.strasdat.net/files/strasdat_thesis_2012.pdf (A.5, pp. 186)
    Vector2<Scalar> const upsilon = a.segment(0, 2);
    Scalar const theta = a[2];
    Scalar const sigma = a[3];
    RxSO2<Scalar> rxso2 = RxSO2<Scalar>::exp(a.template tail<2>());
    Matrix2<Scalar> const Omega = SO2<Scalar>::hat(theta);
    Matrix2<Scalar> const W = details::calcW<Scalar, 2>(Omega, theta, sigma);
    return Sim2<Scalar>(rxso2, W * upsilon);
  }

  /// Returns the ith infinitesimal generators of Sim(2).
  ///
  /// The infinitesimal generators of Sim(2) are:
  ///
  /// ```
  ///         |  0  0  1 |
  ///   G_0 = |  0  0  0 |
  ///         |  0  0  0 |
  ///
  ///         |  0  0  0 |
  ///   G_1 = |  0  0  1 |
  ///         |  0  0  0 |
  ///
  ///         |  0 -1  0 |
  ///   G_2 = |  1  0  0 |
  ///         |  0  0  0 |
  ///
  ///         |  1  0  0 |
  ///   G_3 = |  0  1  0 |
  ///         |  0  0  0 |
  /// ```
  ///
  /// Precondition: ``i`` must be in [0, 3].
  ///
  SOPHUS_FUNC static Transformation generator(int i) {
    SOPHUS_ENSURE(i >= 0 || i <= 3, "i should be in range [0,3].");
    Tangent e;
    e.setZero();
    e[i] = Scalar(1);
    return hat(e);
  }

  /// hat-operator
  ///
  /// It takes in the 4-vector representation and returns the corresponding
  /// matrix representation of Lie algebra element.
  ///
  /// Formally, the hat()-operator of Sim(2) is defined as
  ///
  ///   ``hat(.): R^4 -> R^{3x3},  hat(a) = sum_i a_i * G_i``  (for i=0,...,6)
  ///
  /// with ``G_i`` being the ith infinitesimal generator of Sim(2).
  ///
  /// The corresponding inverse is the vee()-operator, see below.
  ///
  SOPHUS_FUNC static Transformation hat(Tangent const& a) {
    Transformation Omega;
    Omega.template topLeftCorner<2, 2>() =
        RxSO2<Scalar>::hat(a.template tail<2>());
    Omega.col(2).template head<2>() = a.template head<2>();
    Omega.row(2).setZero();
    return Omega;
  }

  /// Lie bracket
  ///
  /// It computes the Lie bracket of Sim(2). To be more specific, it computes
  ///
  ///   ``[omega_1, omega_2]_sim2 := vee([hat(omega_1), hat(omega_2)])``
  ///
  /// with ``[A,B] := AB-BA`` being the matrix commutator, ``hat(.)`` the
  /// hat()-operator and ``vee(.)`` the vee()-operator of Sim(2).
  ///
  SOPHUS_FUNC static Tangent lieBracket(Tangent const& a, Tangent const& b) {
    Vector2<Scalar> const upsilon1 = a.template head<2>();
    Vector2<Scalar> const upsilon2 = b.template head<2>();
    Scalar const theta1 = a[2];
    Scalar const theta2 = b[2];
    Scalar const sigma1 = a[3];
    Scalar const sigma2 = b[3];

    Tangent res;
    res[0] = -theta1 * upsilon2[1] + theta2 * upsilon1[1] +
             sigma1 * upsilon2[0] - sigma2 * upsilon1[0];
    res[1] = theta1 * upsilon2[0] - theta2 * upsilon1[0] +
             sigma1 * upsilon2[1] - sigma2 * upsilon1[1];
    res[2] = Scalar(0);
    res[3] = Scalar(0);

    return res;
  }

  /// Draw uniform sample from Sim(2) manifold.
  ///
  /// Translations are drawn component-wise from the range [-1, 1].
  /// The scale factor is drawn uniformly in log2-space from [-1, 1],
  /// hence the scale is in [0.5, 2].
  ///
  template <class UniformRandomBitGenerator>
  static Sim2 sampleUniform(UniformRandomBitGenerator& generator) {
    std::uniform_real_distribution<Scalar> uniform(Scalar(-1), Scalar(1));
    return Sim2(RxSO2<Scalar>::sampleUniform(generator),
                Vector2<Scalar>(uniform(generator), uniform(generator)));
  }

  /// vee-operator
  ///
  /// It takes the 3x3-matrix representation ``Omega`` and maps it to the
  /// corresponding 4-vector representation of Lie algebra.
  ///
  /// This is the inverse of the hat()-operator, see above.
  ///
  /// Precondition: ``Omega`` must have the following structure:
  ///
  ///                |  d -c  a |
  ///                |  c  d  b |
  ///                |  0  0  0 |
  ///
  SOPHUS_FUNC static Tangent vee(Transformation const& Omega) {
    Tangent upsilon_omega_sigma;
    upsilon_omega_sigma.template head<2>() = Omega.col(2).template head<2>();
    upsilon_omega_sigma.template tail<2>() =
        RxSO2<Scalar>::vee(Omega.template topLeftCorner<2, 2>());
    return upsilon_omega_sigma;
  }

 protected:
  RxSo2Member rxso2_;
  TranslationMember translation_;
};

template <class Scalar, int Options>
Sim2<Scalar, Options>::Sim2() : translation_(TranslationMember::Zero()) {
  static_assert(std::is_standard_layout<Sim2>::value,
                "Assume standard layout for the use of offsetof check below.");
  static_assert(
      offsetof(Sim2, rxso2_) + sizeof(Scalar) * RxSO2<Scalar>::num_parameters ==
          offsetof(Sim2, translation_),
      "This class assumes packed storage and hence will only work "
      "correctly depending on the compiler (options) - in "
      "particular when using [this->data(), this-data() + "
      "num_parameters] to access the raw data in a contiguous fashion.");
}

}  // namespace Sophus

namespace Eigen {

/// Specialization of Eigen::Map for ``Sim2``; derived from Sim2Base.
///
/// Allows us to wrap Sim2 objects around POD array.
template <class Scalar_, int Options>
class Map<Sophus::Sim2<Scalar_>, Options>
    : public Sophus::Sim2Base<Map<Sophus::Sim2<Scalar_>, Options>> {
 public:
  using Base = Sophus::Sim2Base<Map<Sophus::Sim2<Scalar_>, Options>>;
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
      : rxso2_(coeffs),
        translation_(coeffs + Sophus::RxSO2<Scalar>::num_parameters) {}

  /// Mutator of RxSO2
  ///
  SOPHUS_FUNC Map<Sophus::RxSO2<Scalar>, Options>& rxso2() { return rxso2_; }

  /// Accessor of RxSO2
  ///
  SOPHUS_FUNC Map<Sophus::RxSO2<Scalar>, Options> const& rxso2() const {
    return rxso2_;
  }

  /// Mutator of translation vector
  ///
  SOPHUS_FUNC Map<Sophus::Vector2<Scalar>, Options>& translation() {
    return translation_;
  }

  /// Accessor of translation vector
  SOPHUS_FUNC Map<Sophus::Vector2<Scalar>, Options> const& translation() const {
    return translation_;
  }

 protected:
  Map<Sophus::RxSO2<Scalar>, Options> rxso2_;
  Map<Sophus::Vector2<Scalar>, Options> translation_;
};

/// Specialization of Eigen::Map for ``Sim2 const``; derived from Sim2Base.
///
/// Allows us to wrap RxSO2 objects around POD array.
template <class Scalar_, int Options>
class Map<Sophus::Sim2<Scalar_> const, Options>
    : public Sophus::Sim2Base<Map<Sophus::Sim2<Scalar_> const, Options>> {
 public:
  using Base = Sophus::Sim2Base<Map<Sophus::Sim2<Scalar_> const, Options>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  using Base::operator*=;
  using Base::operator*;

  SOPHUS_FUNC Map(Scalar const* coeffs)
      : rxso2_(coeffs),
        translation_(coeffs + Sophus::RxSO2<Scalar>::num_parameters) {}

  /// Accessor of RxSO2
  ///
  SOPHUS_FUNC Map<Sophus::RxSO2<Scalar> const, Options> const& rxso2() const {
    return rxso2_;
  }

  /// Accessor of translation vector
  ///
  SOPHUS_FUNC Map<Sophus::Vector2<Scalar> const, Options> const& translation()
      const {
    return translation_;
  }

 protected:
  Map<Sophus::RxSO2<Scalar> const, Options> const rxso2_;
  Map<Sophus::Vector2<Scalar> const, Options> const translation_;
};
}  // namespace Eigen

#endif
