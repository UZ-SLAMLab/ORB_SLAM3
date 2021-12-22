#ifndef SOPUHS_TESTS_HPP
#define SOPUHS_TESTS_HPP

#include <array>

#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

#include <sophus/average.hpp>
#include <sophus/interpolate.hpp>
#include <sophus/num_diff.hpp>
#include <sophus/test_macros.hpp>

#ifdef SOPHUS_CERES
#include <ceres/jet.h>
#endif

namespace Sophus {

template <class LieGroup_>
class LieGroupTests {
 public:
  using LieGroup = LieGroup_;
  using Scalar = typename LieGroup::Scalar;
  using Transformation = typename LieGroup::Transformation;
  using Tangent = typename LieGroup::Tangent;
  using Point = typename LieGroup::Point;
  using HomogeneousPoint = typename LieGroup::HomogeneousPoint;
  using ConstPointMap = Eigen::Map<const Point>;
  using Line = typename LieGroup::Line;
  using Adjoint = typename LieGroup::Adjoint;
  static int constexpr N = LieGroup::N;
  static int constexpr DoF = LieGroup::DoF;
  static int constexpr num_parameters = LieGroup::num_parameters;

  LieGroupTests(
      std::vector<LieGroup, Eigen::aligned_allocator<LieGroup>> const&
          group_vec,
      std::vector<Tangent, Eigen::aligned_allocator<Tangent>> const&
          tangent_vec,
      std::vector<Point, Eigen::aligned_allocator<Point>> const& point_vec)
      : group_vec_(group_vec),
        tangent_vec_(tangent_vec),
        point_vec_(point_vec) {}

  bool adjointTest() {
    bool passed = true;
    for (size_t i = 0; i < group_vec_.size(); ++i) {
      Transformation T = group_vec_[i].matrix();
      Adjoint Ad = group_vec_[i].Adj();
      for (size_t j = 0; j < tangent_vec_.size(); ++j) {
        Tangent x = tangent_vec_[j];

        Transformation I;
        I.setIdentity();
        Tangent ad1 = Ad * x;
        Tangent ad2 = LieGroup::vee(T * LieGroup::hat(x) *
                                    group_vec_[i].inverse().matrix());
        SOPHUS_TEST_APPROX(passed, ad1, ad2, Scalar(10) * kSmallEps,
                           "Adjoint case %, %", i, j);
      }
    }
    return passed;
  }

  bool contructorAndAssignmentTest() {
    bool passed = true;
    for (LieGroup foo_T_bar : group_vec_) {
      LieGroup foo_T2_bar = foo_T_bar;
      SOPHUS_TEST_APPROX(passed, foo_T_bar.matrix(), foo_T2_bar.matrix(),
                         kSmallEps, "Copy constructor: %\nvs\n %",
                         transpose(foo_T_bar.matrix()),
                         transpose(foo_T2_bar.matrix()));
      LieGroup foo_T3_bar;
      foo_T3_bar = foo_T_bar;
      SOPHUS_TEST_APPROX(passed, foo_T_bar.matrix(), foo_T3_bar.matrix(),
                         kSmallEps, "Copy assignment: %\nvs\n %",
                         transpose(foo_T_bar.matrix()),
                         transpose(foo_T3_bar.matrix()));

      LieGroup foo_T4_bar(foo_T_bar.matrix());
      SOPHUS_TEST_APPROX(
          passed, foo_T_bar.matrix(), foo_T4_bar.matrix(), kSmallEps,
          "Constructor from homogeneous matrix: %\nvs\n %",
          transpose(foo_T_bar.matrix()), transpose(foo_T4_bar.matrix()));

      Eigen::Map<LieGroup> foo_Tmap_bar(foo_T_bar.data());
      LieGroup foo_T5_bar = foo_Tmap_bar;
      SOPHUS_TEST_APPROX(
          passed, foo_T_bar.matrix(), foo_T5_bar.matrix(), kSmallEps,
          "Assignment from Eigen::Map type: %\nvs\n %",
          transpose(foo_T_bar.matrix()), transpose(foo_T5_bar.matrix()));

      Eigen::Map<LieGroup const> foo_Tcmap_bar(foo_T_bar.data());
      LieGroup foo_T6_bar;
      foo_T6_bar = foo_Tcmap_bar;
      SOPHUS_TEST_APPROX(
          passed, foo_T_bar.matrix(), foo_T5_bar.matrix(), kSmallEps,
          "Assignment from Eigen::Map type: %\nvs\n %",
          transpose(foo_T_bar.matrix()), transpose(foo_T5_bar.matrix()));

      LieGroup I;
      Eigen::Map<LieGroup> foo_Tmap2_bar(I.data());
      foo_Tmap2_bar = foo_T_bar;
      SOPHUS_TEST_APPROX(passed, foo_Tmap2_bar.matrix(), foo_T_bar.matrix(),
                         kSmallEps, "Assignment to Eigen::Map type: %\nvs\n %",
                         transpose(foo_Tmap2_bar.matrix()),
                         transpose(foo_T_bar.matrix()));
    }
    return passed;
  }

  bool derivativeTest() {
    bool passed = true;

    LieGroup g;
    for (int i = 0; i < DoF; ++i) {
      Transformation Gi = g.Dxi_exp_x_matrix_at_0(i);
      Transformation Gi2 = curveNumDiff(
          [i](Scalar xi) -> Transformation {
            Tangent x;
            setToZero(x);
            setElementAt(x, xi, i);
            return LieGroup::exp(x).matrix();
          },
          Scalar(0));
      SOPHUS_TEST_APPROX(passed, Gi, Gi2, kSmallEpsSqrt,
                         "Dxi_exp_x_matrix_at_ case %", i);
    }

    return passed;
  }

  template <class G = LieGroup>
  enable_if_t<std::is_same<G, Sophus::SO2<Scalar>>::value ||
                  std::is_same<G, Sophus::SO3<Scalar>>::value ||
                  std::is_same<G, Sophus::SE2<Scalar>>::value ||
                  std::is_same<G, Sophus::SE3<Scalar>>::value,
              bool>
  additionalDerivativeTest() {
    bool passed = true;
    for (size_t j = 0; j < tangent_vec_.size(); ++j) {
      Tangent a = tangent_vec_[j];
      Eigen::Matrix<Scalar, num_parameters, DoF> J = LieGroup::Dx_exp_x(a);
      Eigen::Matrix<Scalar, num_parameters, DoF> J_num =
          vectorFieldNumDiff<Scalar, num_parameters, DoF>(
              [](Tangent const& x) -> Sophus::Vector<Scalar, num_parameters> {
                return LieGroup::exp(x).params();
              },
              a);

      SOPHUS_TEST_APPROX(passed, J, J_num, 3 * kSmallEpsSqrt,
                         "Dx_exp_x case: %", j);
    }

    Tangent o;
    setToZero(o);
    Eigen::Matrix<Scalar, num_parameters, DoF> J = LieGroup::Dx_exp_x_at_0();
    Eigen::Matrix<Scalar, num_parameters, DoF> J_num =
        vectorFieldNumDiff<Scalar, num_parameters, DoF>(
            [](Tangent const& x) -> Sophus::Vector<Scalar, num_parameters> {
              return LieGroup::exp(x).params();
            },
            o);
    SOPHUS_TEST_APPROX(passed, J, J_num, kSmallEpsSqrt, "Dx_exp_x_at_0");

    for (size_t i = 0; i < group_vec_.size(); ++i) {
      LieGroup T = group_vec_[i];

      Eigen::Matrix<Scalar, num_parameters, DoF> J = T.Dx_this_mul_exp_x_at_0();
      Eigen::Matrix<Scalar, num_parameters, DoF> J_num =
          vectorFieldNumDiff<Scalar, num_parameters, DoF>(
              [T](Tangent const& x) -> Sophus::Vector<Scalar, num_parameters> {
                return (T * LieGroup::exp(x)).params();
              },
              o);

      SOPHUS_TEST_APPROX(passed, J, J_num, kSmallEpsSqrt,
                         "Dx_this_mul_exp_x_at_0 case: %", i);
    }

    return passed;
  }

  template <class G = LieGroup>
  enable_if_t<!std::is_same<G, Sophus::SO2<Scalar>>::value &&
                  !std::is_same<G, Sophus::SO3<Scalar>>::value &&
                  !std::is_same<G, Sophus::SE2<Scalar>>::value &&
                  !std::is_same<G, Sophus::SE3<Scalar>>::value,
              bool>
  additionalDerivativeTest() {
    return true;
  }

  bool productTest() {
    bool passed = true;

    for (size_t i = 0; i < group_vec_.size() - 1; ++i) {
      LieGroup T1 = group_vec_[i];
      LieGroup T2 = group_vec_[i + 1];
      LieGroup mult = T1 * T2;
      T1 *= T2;
      SOPHUS_TEST_APPROX(passed, T1.matrix(), mult.matrix(), kSmallEps,
                         "Product case: %", i);
    }
    return passed;
  }

  bool expLogTest() {
    bool passed = true;

    for (size_t i = 0; i < group_vec_.size(); ++i) {
      Transformation T1 = group_vec_[i].matrix();
      Transformation T2 = LieGroup::exp(group_vec_[i].log()).matrix();
      SOPHUS_TEST_APPROX(passed, T1, T2, kSmallEps, "G - exp(log(G)) case: %",
                         i);
    }
    return passed;
  }

  bool expMapTest() {
    bool passed = true;
    for (size_t i = 0; i < tangent_vec_.size(); ++i) {
      Tangent omega = tangent_vec_[i];
      Transformation exp_x = LieGroup::exp(omega).matrix();
      Transformation expmap_hat_x = (LieGroup::hat(omega)).exp();
      SOPHUS_TEST_APPROX(passed, exp_x, expmap_hat_x, Scalar(10) * kSmallEps,
                         "expmap(hat(x)) - exp(x) case: %", i);
    }
    return passed;
  }

  bool groupActionTest() {
    bool passed = true;

    for (size_t i = 0; i < group_vec_.size(); ++i) {
      for (size_t j = 0; j < point_vec_.size(); ++j) {
        Point const& p = point_vec_[j];
        Point point1 = group_vec_[i] * p;

        HomogeneousPoint hp = p.homogeneous();
        HomogeneousPoint hpoint1 = group_vec_[i] * hp;

        ConstPointMap p_map(p.data());
        Point pointmap1 = group_vec_[i] * p_map;

        Transformation T = group_vec_[i].matrix();
        Point gt_point1 = map(T, p);

        SOPHUS_TEST_APPROX(passed, point1, gt_point1, kSmallEps,
                           "Transform point case: %", i);
        SOPHUS_TEST_APPROX(passed, hpoint1.hnormalized().eval(), gt_point1,
                           kSmallEps, "Transform homogeneous point case: %", i);
        SOPHUS_TEST_APPROX(passed, pointmap1, gt_point1, kSmallEps,
                           "Transform map point case: %", i);
      }
    }
    return passed;
  }

  bool lineActionTest() {
    bool passed = point_vec_.size() > 1;

    for (size_t i = 0; i < group_vec_.size(); ++i) {
      for (size_t j = 0; j + 1 < point_vec_.size(); ++j) {
        Point const& p1 = point_vec_[j];
        Point const& p2 = point_vec_[j + 1];
        Line l = Line::Through(p1, p2);
        Point p1_t = group_vec_[i] * p1;
        Point p2_t = group_vec_[i] * p2;
        Line l_t = group_vec_[i] * l;

        SOPHUS_TEST_APPROX(passed, l_t.squaredDistance(p1_t),
                           static_cast<Scalar>(0), kSmallEps,
                           "Transform line case (1st point) : %", i);
        SOPHUS_TEST_APPROX(passed, l_t.squaredDistance(p2_t),
                           static_cast<Scalar>(0), kSmallEps,
                           "Transform line case (2nd point) : %", i);
        SOPHUS_TEST_APPROX(passed, l_t.direction().squaredNorm(),
                           l.direction().squaredNorm(), kSmallEps,
                           "Transform line case (direction) : %", i);
      }
    }
    return passed;
  }

  bool lieBracketTest() {
    bool passed = true;
    for (size_t i = 0; i < tangent_vec_.size(); ++i) {
      for (size_t j = 0; j < tangent_vec_.size(); ++j) {
        Tangent tangent1 =
            LieGroup::lieBracket(tangent_vec_[i], tangent_vec_[j]);
        Transformation hati = LieGroup::hat(tangent_vec_[i]);
        Transformation hatj = LieGroup::hat(tangent_vec_[j]);

        Tangent tangent2 = LieGroup::vee(hati * hatj - hatj * hati);
        SOPHUS_TEST_APPROX(passed, tangent1, tangent2, kSmallEps,
                           "Lie Bracket case: %", i);
      }
    }
    return passed;
  }

  bool veeHatTest() {
    bool passed = true;
    for (size_t i = 0; i < tangent_vec_.size(); ++i) {
      SOPHUS_TEST_APPROX(passed, Tangent(tangent_vec_[i]),
                         LieGroup::vee(LieGroup::hat(tangent_vec_[i])),
                         kSmallEps, "Hat-vee case: %", i);
    }
    return passed;
  }

  bool newDeleteSmokeTest() {
    bool passed = true;
    LieGroup* raw_ptr = nullptr;
    raw_ptr = new LieGroup();
    SOPHUS_TEST_NEQ(passed, reinterpret_cast<std::uintptr_t>(raw_ptr), 0);
    delete raw_ptr;
    return passed;
  }

  bool interpolateAndMeanTest() {
    bool passed = true;
    using std::sqrt;
    Scalar const eps = Constants<Scalar>::epsilon();
    Scalar const sqrt_eps = sqrt(eps);
    // TODO: Improve accuracy of ``interpolate`` (and hence ``exp`` and ``log``)
    //       so that we can use more accurate bounds in these tests, i.e.
    //       ``eps`` instead of ``sqrt_eps``.

    for (LieGroup const& foo_T_bar : group_vec_) {
      for (LieGroup const& foo_T_baz : group_vec_) {
        // Test boundary conditions ``alpha=0`` and ``alpha=1``.
        LieGroup foo_T_quiz = interpolate(foo_T_bar, foo_T_baz, Scalar(0));
        SOPHUS_TEST_APPROX(passed, foo_T_quiz.matrix(), foo_T_bar.matrix(),
                           sqrt_eps);
        foo_T_quiz = interpolate(foo_T_bar, foo_T_baz, Scalar(1));
        SOPHUS_TEST_APPROX(passed, foo_T_quiz.matrix(), foo_T_baz.matrix(),
                           sqrt_eps);
      }
    }
    for (Scalar alpha :
         {Scalar(0.1), Scalar(0.5), Scalar(0.75), Scalar(0.99)}) {
      for (LieGroup const& foo_T_bar : group_vec_) {
        for (LieGroup const& foo_T_baz : group_vec_) {
          LieGroup foo_T_quiz = interpolate(foo_T_bar, foo_T_baz, alpha);
          // test left-invariance:
          //
          // dash_T_foo * interp(foo_T_bar, foo_T_baz)
          // == interp(dash_T_foo * foo_T_bar, dash_T_foo * foo_T_baz)

          if (interp_details::Traits<LieGroup>::hasShortestPathAmbiguity(
                  foo_T_bar.inverse() * foo_T_baz)) {
            // skip check since there is a shortest path ambiguity
            continue;
          }
          for (LieGroup const& dash_T_foo : group_vec_) {
            LieGroup dash_T_quiz = interpolate(dash_T_foo * foo_T_bar,
                                               dash_T_foo * foo_T_baz, alpha);
            SOPHUS_TEST_APPROX(passed, dash_T_quiz.matrix(),
                               (dash_T_foo * foo_T_quiz).matrix(), sqrt_eps);
          }
          // test inverse-invariance:
          //
          // interp(foo_T_bar, foo_T_baz).inverse()
          // == interp(foo_T_bar.inverse(), dash_T_foo.inverse())
          LieGroup quiz_T_foo =
              interpolate(foo_T_bar.inverse(), foo_T_baz.inverse(), alpha);
          SOPHUS_TEST_APPROX(passed, quiz_T_foo.inverse().matrix(),
                             foo_T_quiz.matrix(), sqrt_eps);
        }
      }

      for (LieGroup const& bar_T_foo : group_vec_) {
        for (LieGroup const& baz_T_foo : group_vec_) {
          LieGroup quiz_T_foo = interpolate(bar_T_foo, baz_T_foo, alpha);
          // test right-invariance:
          //
          // interp(bar_T_foo, bar_T_foo) * foo_T_dash
          // == interp(bar_T_foo * foo_T_dash, bar_T_foo * foo_T_dash)

          if (interp_details::Traits<LieGroup>::hasShortestPathAmbiguity(
                  bar_T_foo * baz_T_foo.inverse())) {
            // skip check since there is a shortest path ambiguity
            continue;
          }
          for (LieGroup const& foo_T_dash : group_vec_) {
            LieGroup quiz_T_dash = interpolate(bar_T_foo * foo_T_dash,
                                               baz_T_foo * foo_T_dash, alpha);
            SOPHUS_TEST_APPROX(passed, quiz_T_dash.matrix(),
                               (quiz_T_foo * foo_T_dash).matrix(), sqrt_eps);
          }
        }
      }
    }

    for (LieGroup const& foo_T_bar : group_vec_) {
      for (LieGroup const& foo_T_baz : group_vec_) {
        if (interp_details::Traits<LieGroup>::hasShortestPathAmbiguity(
                foo_T_bar.inverse() * foo_T_baz)) {
          // skip check since there is a shortest path ambiguity
          continue;
        }

        // test average({A, B}) == interp(A, B):
        LieGroup foo_T_quiz = interpolate(foo_T_bar, foo_T_baz, 0.5);
        optional<LieGroup> foo_T_iaverage = iterativeMean(
            std::array<LieGroup, 2>({{foo_T_bar, foo_T_baz}}), 20);
        optional<LieGroup> foo_T_average =
            average(std::array<LieGroup, 2>({{foo_T_bar, foo_T_baz}}));
        SOPHUS_TEST(passed, bool(foo_T_average),
                    "log(foo_T_bar): %\nlog(foo_T_baz): %",
                    transpose(foo_T_bar.log()), transpose(foo_T_baz.log()));
        if (foo_T_average) {
          SOPHUS_TEST_APPROX(
              passed, foo_T_quiz.matrix(), foo_T_average->matrix(), sqrt_eps,
              "log(foo_T_bar): %\nlog(foo_T_baz): %\n"
              "log(interp): %\nlog(average): %",
              transpose(foo_T_bar.log()), transpose(foo_T_baz.log()),
              transpose(foo_T_quiz.log()), transpose(foo_T_average->log()));
        }
        SOPHUS_TEST(passed, bool(foo_T_iaverage),
                    "log(foo_T_bar): %\nlog(foo_T_baz): %\n"
                    "log(interp): %\nlog(iaverage): %",
                    transpose(foo_T_bar.log()), transpose(foo_T_baz.log()),
                    transpose(foo_T_quiz.log()),
                    transpose(foo_T_iaverage->log()));
        if (foo_T_iaverage) {
          SOPHUS_TEST_APPROX(
              passed, foo_T_quiz.matrix(), foo_T_iaverage->matrix(), sqrt_eps,
              "log(foo_T_bar): %\nlog(foo_T_baz): %",
              transpose(foo_T_bar.log()), transpose(foo_T_baz.log()));
        }
      }
    }

    return passed;
  }

  bool testRandomSmoke() {
    bool passed = true;
    std::default_random_engine engine;
    for (int i = 0; i < 100; ++i) {
      LieGroup g = LieGroup::sampleUniform(engine);
      SOPHUS_TEST_EQUAL(passed, g.params(), g.params());
    }
    return passed;
  }

  template <class S = Scalar>
  enable_if_t<std::is_floating_point<S>::value, bool> doAllTestsPass() {
    return doesLargeTestSetPass();
  }

  template <class S = Scalar>
  enable_if_t<!std::is_floating_point<S>::value, bool> doAllTestsPass() {
    return doesSmallTestSetPass();
  }

 private:
  bool doesSmallTestSetPass() {
    bool passed = true;
    passed &= adjointTest();
    passed &= contructorAndAssignmentTest();
    passed &= productTest();
    passed &= expLogTest();
    passed &= groupActionTest();
    passed &= lineActionTest();
    passed &= lieBracketTest();
    passed &= veeHatTest();
    passed &= newDeleteSmokeTest();
    return passed;
  }

  bool doesLargeTestSetPass() {
    bool passed = true;
    passed &= doesSmallTestSetPass();
    passed &= additionalDerivativeTest();
    passed &= derivativeTest();
    passed &= expMapTest();
    passed &= interpolateAndMeanTest();
    passed &= testRandomSmoke();
    return passed;
  }

  Scalar const kSmallEps = Constants<Scalar>::epsilon();
  Scalar const kSmallEpsSqrt = Constants<Scalar>::epsilonSqrt();

  Eigen::Matrix<Scalar, N - 1, 1> map(
      Eigen::Matrix<Scalar, N, N> const& T,
      Eigen::Matrix<Scalar, N - 1, 1> const& p) {
    return T.template topLeftCorner<N - 1, N - 1>() * p +
           T.template topRightCorner<N - 1, 1>();
  }

  Eigen::Matrix<Scalar, N, 1> map(Eigen::Matrix<Scalar, N, N> const& T,
                                  Eigen::Matrix<Scalar, N, 1> const& p) {
    return T * p;
  }

  std::vector<LieGroup, Eigen::aligned_allocator<LieGroup>> group_vec_;
  std::vector<Tangent, Eigen::aligned_allocator<Tangent>> tangent_vec_;
  std::vector<Point, Eigen::aligned_allocator<Point>> point_vec_;
};

template <class Scalar>
std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> getTestSE3s() {
  Scalar const kPi = Constants<Scalar>::pi();
  std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> se3_vec;
  se3_vec.push_back(SE3<Scalar>(
      SO3<Scalar>::exp(Vector3<Scalar>(Scalar(0.2), Scalar(0.5), Scalar(0.0))),
      Vector3<Scalar>(Scalar(0), Scalar(0), Scalar(0))));
  se3_vec.push_back(SE3<Scalar>(
      SO3<Scalar>::exp(Vector3<Scalar>(Scalar(0.2), Scalar(0.5), Scalar(-1.0))),
      Vector3<Scalar>(Scalar(10), Scalar(0), Scalar(0))));
  se3_vec.push_back(
      SE3<Scalar>::trans(Vector3<Scalar>(Scalar(0), Scalar(100), Scalar(5))));
  se3_vec.push_back(SE3<Scalar>::rotZ(Scalar(0.00001)));
  se3_vec.push_back(
      SE3<Scalar>::trans(Scalar(0), Scalar(-0.00000001), Scalar(0.0000000001)) *
      SE3<Scalar>::rotZ(Scalar(0.00001)));
  se3_vec.push_back(SE3<Scalar>::transX(Scalar(0.01)) *
                    SE3<Scalar>::rotZ(Scalar(0.00001)));
  se3_vec.push_back(SE3<Scalar>::trans(Scalar(4), Scalar(-5), Scalar(0)) *
                    SE3<Scalar>::rotX(kPi));
  se3_vec.push_back(
      SE3<Scalar>(SO3<Scalar>::exp(
                      Vector3<Scalar>(Scalar(0.2), Scalar(0.5), Scalar(0.0))),
                  Vector3<Scalar>(Scalar(0), Scalar(0), Scalar(0))) *
      SE3<Scalar>::rotX(kPi) *
      SE3<Scalar>(SO3<Scalar>::exp(Vector3<Scalar>(Scalar(-0.2), Scalar(-0.5),
                                                   Scalar(-0.0))),
                  Vector3<Scalar>(Scalar(0), Scalar(0), Scalar(0))));
  se3_vec.push_back(
      SE3<Scalar>(SO3<Scalar>::exp(
                      Vector3<Scalar>(Scalar(0.3), Scalar(0.5), Scalar(0.1))),
                  Vector3<Scalar>(Scalar(2), Scalar(0), Scalar(-7))) *
      SE3<Scalar>::rotX(kPi) *
      SE3<Scalar>(SO3<Scalar>::exp(Vector3<Scalar>(Scalar(-0.3), Scalar(-0.5),
                                                   Scalar(-0.1))),
                  Vector3<Scalar>(Scalar(0), Scalar(6), Scalar(0))));
  return se3_vec;
}

template <class T>
std::vector<SE2<T>, Eigen::aligned_allocator<SE2<T>>> getTestSE2s() {
  T const kPi = Constants<T>::pi();
  std::vector<SE2<T>, Eigen::aligned_allocator<SE2<T>>> se2_vec;
  se2_vec.push_back(SE2<T>());
  se2_vec.push_back(SE2<T>(SO2<T>(0.2), Vector2<T>(10, 0)));
  se2_vec.push_back(SE2<T>::transY(100));
  se2_vec.push_back(SE2<T>::trans(Vector2<T>(1, 2)));
  se2_vec.push_back(SE2<T>(SO2<T>(-1.), Vector2<T>(20, -1)));
  se2_vec.push_back(
      SE2<T>(SO2<T>(0.00001), Vector2<T>(-0.00000001, 0.0000000001)));
  se2_vec.push_back(SE2<T>(SO2<T>(0.3), Vector2<T>(2, 0)) * SE2<T>::rot(kPi) *
                    SE2<T>(SO2<T>(-0.3), Vector2<T>(0, 6)));
  return se2_vec;
}
}  // namespace Sophus
#endif  // TESTS_HPP
