#include <ceres/ceres.h>
#include <iostream>
#include <sophus/se3.hpp>

#include "local_parameterization_se3.hpp"

// Eigen's ostream operator is not compatible with ceres::Jet types.
// In particular, Eigen assumes that the scalar type (here Jet<T,N>) can be
// casted to an arithmetic type, which is not true for ceres::Jet.
// Unfortunately, the ceres::Jet class does not define a conversion
// operator (http://en.cppreference.com/w/cpp/language/cast_operator).
//
// This workaround creates a template specialization for Eigen's cast_impl,
// when casting from a ceres::Jet type. It relies on Eigen's internal API and
// might break with future versions of Eigen.
namespace Eigen {
namespace internal {

template <class T, int N, typename NewType>
struct cast_impl<ceres::Jet<T, N>, NewType> {
  EIGEN_DEVICE_FUNC
  static inline NewType run(ceres::Jet<T, N> const& x) {
    return static_cast<NewType>(x.a);
  }
};

}  // namespace internal
}  // namespace Eigen

struct TestSE3CostFunctor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TestSE3CostFunctor(Sophus::SE3d T_aw) : T_aw(T_aw) {}

  template <class T>
  bool operator()(T const* const sT_wa, T* sResiduals) const {
    Eigen::Map<Sophus::SE3<T> const> const T_wa(sT_wa);
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(sResiduals);

    // We are able to mix Sophus types with doubles and Jet types without
    // needing to cast to T.
    residuals = (T_aw * T_wa).log();
    // Reverse order of multiplication. This forces the compiler to verify that
    // (Jet, double) and (double, Jet) SE3 multiplication work correctly.
    residuals = (T_wa * T_aw).log();
    // Finally, ensure that Jet-to-Jet multiplication works.
    residuals = (T_wa * T_aw.cast<T>()).log();
    return true;
  }

  Sophus::SE3d T_aw;
};

struct TestPointCostFunctor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TestPointCostFunctor(Sophus::SE3d T_aw, Eigen::Vector3d point_a)
      : T_aw(T_aw), point_a(point_a) {}

  template <class T>
  bool operator()(T const* const sT_wa, T const* const spoint_b,
                  T* sResiduals) const {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    Eigen::Map<Sophus::SE3<T> const> const T_wa(sT_wa);
    Eigen::Map<Vector3T const> point_b(spoint_b);
    Eigen::Map<Vector3T> residuals(sResiduals);

    // Multiply SE3d by Jet Vector3.
    Vector3T point_b_prime = T_aw * point_b;
    // Ensure Jet SE3 multiplication with Jet Vector3.
    point_b_prime = T_aw.cast<T>() * point_b;

    // Multiply Jet SE3 with Vector3d.
    Vector3T point_a_prime = T_wa * point_a;
    // Ensure Jet SE3 multiplication with Jet Vector3.
    point_a_prime = T_wa * point_a.cast<T>();

    residuals = point_b_prime - point_a_prime;
    return true;
  }

  Sophus::SE3d T_aw;
  Eigen::Vector3d point_a;
};

bool test(Sophus::SE3d const& T_w_targ, Sophus::SE3d const& T_w_init,
          Sophus::SE3d::Point const& point_a_init,
          Sophus::SE3d::Point const& point_b) {
  static constexpr int kNumPointParameters = 3;

  // Optimisation parameters.
  Sophus::SE3d T_wr = T_w_init;
  Sophus::SE3d::Point point_a = point_a_init;

  // Build the problem.
  ceres::Problem problem;

  // Specify local update rule for our parameter
  problem.AddParameterBlock(T_wr.data(), Sophus::SE3d::num_parameters,
                            new Sophus::test::LocalParameterizationSE3);

  // Create and add cost functions. Derivatives will be evaluated via
  // automatic differentiation
  ceres::CostFunction* cost_function1 =
      new ceres::AutoDiffCostFunction<TestSE3CostFunctor, Sophus::SE3d::DoF,
                                      Sophus::SE3d::num_parameters>(
          new TestSE3CostFunctor(T_w_targ.inverse()));
  problem.AddResidualBlock(cost_function1, NULL, T_wr.data());
  ceres::CostFunction* cost_function2 =
      new ceres::AutoDiffCostFunction<TestPointCostFunctor, kNumPointParameters,
                                      Sophus::SE3d::num_parameters,
                                      kNumPointParameters>(
          new TestPointCostFunctor(T_w_targ.inverse(), point_b));
  problem.AddResidualBlock(cost_function2, NULL, T_wr.data(), point_a.data());

  // Set solver options (precision / method)
  ceres::Solver::Options options;
  options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.linear_solver_type = ceres::DENSE_QR;

  // Solve
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;

  // Difference between target and parameter
  double const mse = (T_w_targ.inverse() * T_wr).log().squaredNorm();
  bool const passed = mse < 10. * Sophus::Constants<double>::epsilon();
  return passed;
}

template <typename Scalar>
bool CreateSE3FromMatrix(const Eigen::Matrix<Scalar, 4, 4>& mat) {
  Sophus::SE3<Scalar> se3 = Sophus::SE3<Scalar>(mat);
  std::cout << se3.translation().x() << std::endl;
  return true;
}

int main(int, char**) {
  using SE3Type = Sophus::SE3<double>;
  using SO3Type = Sophus::SO3<double>;
  using Point = SE3Type::Point;
  double const kPi = Sophus::Constants<double>::pi();

  std::vector<SE3Type> se3_vec;
  se3_vec.push_back(
      SE3Type(SO3Type::exp(Point(0.2, 0.5, 0.0)), Point(0, 0, 0)));
  se3_vec.push_back(
      SE3Type(SO3Type::exp(Point(0.2, 0.5, -1.0)), Point(10, 0, 0)));
  se3_vec.push_back(SE3Type(SO3Type::exp(Point(0., 0., 0.)), Point(0, 100, 5)));
  se3_vec.push_back(
      SE3Type(SO3Type::exp(Point(0., 0., 0.00001)), Point(0, 0, 0)));
  se3_vec.push_back(SE3Type(SO3Type::exp(Point(0., 0., 0.00001)),
                            Point(0, -0.00000001, 0.0000000001)));
  se3_vec.push_back(
      SE3Type(SO3Type::exp(Point(0., 0., 0.00001)), Point(0.01, 0, 0)));
  se3_vec.push_back(SE3Type(SO3Type::exp(Point(kPi, 0, 0)), Point(4, -5, 0)));
  se3_vec.push_back(
      SE3Type(SO3Type::exp(Point(0.2, 0.5, 0.0)), Point(0, 0, 0)) *
      SE3Type(SO3Type::exp(Point(kPi, 0, 0)), Point(0, 0, 0)) *
      SE3Type(SO3Type::exp(Point(-0.2, -0.5, -0.0)), Point(0, 0, 0)));
  se3_vec.push_back(
      SE3Type(SO3Type::exp(Point(0.3, 0.5, 0.1)), Point(2, 0, -7)) *
      SE3Type(SO3Type::exp(Point(kPi, 0, 0)), Point(0, 0, 0)) *
      SE3Type(SO3Type::exp(Point(-0.3, -0.5, -0.1)), Point(0, 6, 0)));

  std::vector<Point> point_vec;
  point_vec.emplace_back(1.012, 2.73, -1.4);
  point_vec.emplace_back(9.2, -7.3, -4.4);
  point_vec.emplace_back(2.5, 0.1, 9.1);
  point_vec.emplace_back(12.3, 1.9, 3.8);
  point_vec.emplace_back(-3.21, 3.42, 2.3);
  point_vec.emplace_back(-8.0, 6.1, -1.1);
  point_vec.emplace_back(0.0, 2.5, 5.9);
  point_vec.emplace_back(7.1, 7.8, -14);
  point_vec.emplace_back(5.8, 9.2, 0.0);

  for (size_t i = 0; i < se3_vec.size(); ++i) {
    const int other_index = (i + 3) % se3_vec.size();
    bool const passed = test(se3_vec[i], se3_vec[other_index], point_vec[i],
                             point_vec[other_index]);
    if (!passed) {
      std::cerr << "failed!" << std::endl << std::endl;
      exit(-1);
    }
  }

  Eigen::Matrix<ceres::Jet<double, 28>, 4, 4> mat;
  mat.setIdentity();
  std::cout << CreateSE3FromMatrix(mat) << std::endl;

  return 0;
}
