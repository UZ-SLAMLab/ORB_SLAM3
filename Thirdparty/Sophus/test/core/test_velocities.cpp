#include <iostream>

#include "tests.hpp"

#include <sophus/velocities.hpp>

namespace Sophus {
namespace experimental {

template <class Scalar>
bool tests_linear_velocities() {
  bool passed = true;
  std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> bar_Ts_baz;

  for (size_t i = 0; i < 10; ++i) {
    bar_Ts_baz.push_back(SE3<Scalar>::rotX(i * 0.001) *
                         SE3<Scalar>::rotY(i * 0.001) *
                         SE3<Scalar>::transX(0.01 * i));
  }

  SE3<Scalar> foo_T_bar =
      SE3<Scalar>::rotX(0.5) * SE3<Scalar>::rotZ(0.2) * SE3<Scalar>::transY(2);

  std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> foo_Ts_baz;
  for (auto const& bar_T_baz : bar_Ts_baz) {
    foo_Ts_baz.push_back(foo_T_bar * bar_T_baz);
  }

  auto gen_linear_vels =
      [](std::vector<SE3<Scalar>, Eigen::aligned_allocator<SE3<Scalar>>> const&
             a_Ts_b) {
        std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
            linearVels_a;
        for (size_t i = 0; i < a_Ts_b.size() - 1; ++i) {
          linearVels_a.push_back(a_Ts_b[i + 1].translation() -
                                 a_Ts_b[i].translation());
        }
        return linearVels_a;
      };

  // linear velocities in frame bar
  std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
      linearVels_bar = gen_linear_vels(bar_Ts_baz);
  // linear velocities in frame foo
  std::vector<Vector3<Scalar>, Eigen::aligned_allocator<Vector3<Scalar>>>
      linearVels_foo = gen_linear_vels(foo_Ts_baz);

  for (size_t i = 0; i < linearVels_bar.size(); ++i) {
    SOPHUS_TEST_APPROX(passed, linearVels_foo[i],
                       transformVelocity(foo_T_bar, linearVels_bar[i]),
                       sqrt(Constants<Scalar>::epsilon()));
  }
  return passed;
}

template <class Scalar>
bool tests_rotational_velocities() {
  bool passed = true;

  SE3<Scalar> foo_T_bar =
      SE3<Scalar>::rotX(0.5) * SE3<Scalar>::rotZ(0.2) * SE3<Scalar>::transY(2);

  // One parameter subgroup of SE3, motion through space given time t.
  auto bar_T_baz = [](Scalar t) -> SE3<Scalar> {
    return SE3<Scalar>::rotX(t * Scalar(0.01)) *
           SE3<Scalar>::rotY(t * Scalar(0.0001)) *
           SE3<Scalar>::transX(t * Scalar(0.0001));
  };

  std::vector<Scalar> ts = {Scalar(0), Scalar(0.3), Scalar(1)};

  Scalar h = Constants<Scalar>::epsilon();
  for (Scalar t : ts) {
    // finite difference approximiation of instantanious velocity in frame bar
    Vector3<Scalar> rotVel_in_frame_bar =
        finiteDifferenceRotationalVelocity<Scalar>(bar_T_baz, t, h);

    // finite difference approximiation of instantanious velocity in frame foo
    Vector3<Scalar> rotVel_in_frame_foo =
        finiteDifferenceRotationalVelocity<Scalar>(
            [&foo_T_bar, bar_T_baz](Scalar t) -> SE3<Scalar> {
              return foo_T_bar * bar_T_baz(t);
            },
            t, h);

    Vector3<Scalar> rotVel_in_frame_bar2 =
        transformVelocity(foo_T_bar.inverse(), rotVel_in_frame_foo);
    SOPHUS_TEST_APPROX(
        passed, rotVel_in_frame_bar, rotVel_in_frame_bar2,
        // not too tight threshold, because of finit difference approximation
        std::sqrt(Constants<Scalar>::epsilon()));

    // The rotational velocities rotVel_in_frame_foo and rotVel_in_frame_bar
    // should not be equal since they are in different frames (foo != bar).
    SOPHUS_TEST_NOT_APPROX(passed, rotVel_in_frame_foo, rotVel_in_frame_bar,
                           Scalar(1e-3));

    // Expect same result when using adjoint instead since:
    //  vee(bar_R_foo * hat(vel_foo) * bar_R_foo^T = bar_R_foo 8 vel_foo.
    SOPHUS_TEST_APPROX(
        passed, transformVelocity(foo_T_bar.inverse(), rotVel_in_frame_foo),
        SO3<Scalar>::vee(foo_T_bar.so3().inverse().matrix() *
                         SO3<Scalar>::hat(rotVel_in_frame_foo) *
                         foo_T_bar.so3().matrix()),
        Constants<Scalar>::epsilon());
  }
  return passed;
}

int test_velocities() {
  using std::cerr;
  using std::endl;

  cerr << "Test Velocities" << endl << endl;
  cerr << "Double tests: " << endl;
  bool passed = tests_linear_velocities<double>();
  passed &= tests_rotational_velocities<double>();
  processTestResult(passed);

  cerr << "Float tests: " << endl;
  passed = tests_linear_velocities<float>();
  passed &= tests_rotational_velocities<float>();
  processTestResult(passed);

  return 0;
}
}  // namespace experimental
}  // namespace Sophus

int main() { return Sophus::experimental::test_velocities(); }
