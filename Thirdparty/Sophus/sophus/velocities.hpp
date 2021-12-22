#ifndef SOPHUS_VELOCITIES_HPP
#define SOPHUS_VELOCITIES_HPP

#include <functional>

#include "num_diff.hpp"
#include "se3.hpp"

namespace Sophus {
namespace experimental {
// Experimental since the API will certainly change drastically in the future.

// Transforms velocity vector by rotation ``foo_R_bar``.
//
// Note: vel_bar can be either a linear or a rotational velocity vector.
//
template <class Scalar>
Vector3<Scalar> transformVelocity(SO3<Scalar> const& foo_R_bar,
                                  Vector3<Scalar> const& vel_bar) {
  // For rotational velocities note that:
  //
  //   vel_bar = vee(foo_R_bar * hat(vel_bar) * foo_R_bar^T)
  //           = vee(hat(Adj(foo_R_bar) * vel_bar))
  //           = Adj(foo_R_bar) * vel_bar
  //           = foo_R_bar * vel_bar.
  //
  return foo_R_bar * vel_bar;
}

// Transforms velocity vector by pose ``foo_T_bar``.
//
// Note: vel_bar can be either a linear or a rotational velocity vector.
//
template <class Scalar>
Vector3<Scalar> transformVelocity(SE3<Scalar> const& foo_T_bar,
                                  Vector3<Scalar> const& vel_bar) {
  return transformVelocity(foo_T_bar.so3(), vel_bar);
}

// finite difference approximation of instantanious velocity in frame foo
//
template <class Scalar>
Vector3<Scalar> finiteDifferenceRotationalVelocity(
    std::function<SO3<Scalar>(Scalar)> const& foo_R_bar, Scalar t,
    Scalar h = Constants<Scalar>::epsilon()) {
  // https://en.wikipedia.org/w/index.php?title=Angular_velocity&oldid=791867792#Angular_velocity_tensor
  //
  // W = dR(t)/dt * R^{-1}(t)
  Matrix3<Scalar> dR_dt_in_frame_foo = curveNumDiff(
      [&foo_R_bar](Scalar t0) -> Matrix3<Scalar> {
        return foo_R_bar(t0).matrix();
      },
      t, h);
  // velocity tensor
  Matrix3<Scalar> W_in_frame_foo =
      dR_dt_in_frame_foo * (foo_R_bar(t)).inverse().matrix();
  return SO3<Scalar>::vee(W_in_frame_foo);
}

// finite difference approximation of instantanious velocity in frame foo
//
template <class Scalar>
Vector3<Scalar> finiteDifferenceRotationalVelocity(
    std::function<SE3<Scalar>(Scalar)> const& foo_T_bar, Scalar t,
    Scalar h = Constants<Scalar>::epsilon()) {
  return finiteDifferenceRotationalVelocity<Scalar>(
      [&foo_T_bar](Scalar t) -> SO3<Scalar> { return foo_T_bar(t).so3(); }, t,
      h);
}

}  // namespace experimental
}  // namespace Sophus

#endif  // SOPHUS_VELOCITIES_HPP
