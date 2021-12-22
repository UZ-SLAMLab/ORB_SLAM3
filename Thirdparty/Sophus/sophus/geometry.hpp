/// @file
/// Transformations between poses and hyperplanes.

#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include "se2.hpp"
#include "se3.hpp"
#include "so2.hpp"
#include "so3.hpp"
#include "types.hpp"

namespace Sophus {

/// Takes in a rotation ``R_foo_plane`` and returns the corresponding line
/// normal along the y-axis (in reference frame ``foo``).
///
template <class T>
Vector2<T> normalFromSO2(SO2<T> const& R_foo_line) {
  return R_foo_line.matrix().col(1);
}

/// Takes in line normal in reference frame foo and constructs a corresponding
/// rotation matrix ``R_foo_line``.
///
/// Precondition: ``normal_foo`` must not be close to zero.
///
template <class T>
SO2<T> SO2FromNormal(Vector2<T> normal_foo) {
  SOPHUS_ENSURE(normal_foo.squaredNorm() > Constants<T>::epsilon(), "%",
                normal_foo.transpose());
  normal_foo.normalize();
  return SO2<T>(normal_foo.y(), -normal_foo.x());
}

/// Takes in a rotation ``R_foo_plane`` and returns the corresponding plane
/// normal along the z-axis
/// (in reference frame ``foo``).
///
template <class T>
Vector3<T> normalFromSO3(SO3<T> const& R_foo_plane) {
  return R_foo_plane.matrix().col(2);
}

/// Takes in plane normal in reference frame foo and constructs a corresponding
/// rotation matrix ``R_foo_plane``.
///
/// Note: The ``plane`` frame is defined as such that the normal points along
///       the positive z-axis. One can specify hints for the x-axis and y-axis
///       of the ``plane`` frame.
///
/// Preconditions:
/// - ``normal_foo``, ``xDirHint_foo``, ``yDirHint_foo`` must not be close to
///   zero.
/// - ``xDirHint_foo`` and ``yDirHint_foo`` must be approx. perpendicular.
///
template <class T>
Matrix3<T> rotationFromNormal(Vector3<T> const& normal_foo,
                              Vector3<T> xDirHint_foo = Vector3<T>(T(1), T(0),
                                                                   T(0)),
                              Vector3<T> yDirHint_foo = Vector3<T>(T(0), T(1),
                                                                   T(0))) {
  SOPHUS_ENSURE(xDirHint_foo.dot(yDirHint_foo) < Constants<T>::epsilon(),
                "xDirHint (%) and yDirHint (%) must be perpendicular.",
                xDirHint_foo.transpose(), yDirHint_foo.transpose());
  using std::abs;
  using std::sqrt;
  T const xDirHint_foo_sqr_length = xDirHint_foo.squaredNorm();
  T const yDirHint_foo_sqr_length = yDirHint_foo.squaredNorm();
  T const normal_foo_sqr_length = normal_foo.squaredNorm();
  SOPHUS_ENSURE(xDirHint_foo_sqr_length > Constants<T>::epsilon(), "%",
                xDirHint_foo.transpose());
  SOPHUS_ENSURE(yDirHint_foo_sqr_length > Constants<T>::epsilon(), "%",
                yDirHint_foo.transpose());
  SOPHUS_ENSURE(normal_foo_sqr_length > Constants<T>::epsilon(), "%",
                normal_foo.transpose());

  Matrix3<T> basis_foo;
  basis_foo.col(2) = normal_foo;

  if (abs(xDirHint_foo_sqr_length - T(1)) > Constants<T>::epsilon()) {
    xDirHint_foo.normalize();
  }
  if (abs(yDirHint_foo_sqr_length - T(1)) > Constants<T>::epsilon()) {
    yDirHint_foo.normalize();
  }
  if (abs(normal_foo_sqr_length - T(1)) > Constants<T>::epsilon()) {
    basis_foo.col(2).normalize();
  }

  T abs_x_dot_z = abs(basis_foo.col(2).dot(xDirHint_foo));
  T abs_y_dot_z = abs(basis_foo.col(2).dot(yDirHint_foo));
  if (abs_x_dot_z < abs_y_dot_z) {
    // basis_foo.z and xDirHint are far from parallel.
    basis_foo.col(1) = basis_foo.col(2).cross(xDirHint_foo).normalized();
    basis_foo.col(0) = basis_foo.col(1).cross(basis_foo.col(2));
  } else {
    // basis_foo.z and yDirHint are far from parallel.
    basis_foo.col(0) = yDirHint_foo.cross(basis_foo.col(2)).normalized();
    basis_foo.col(1) = basis_foo.col(2).cross(basis_foo.col(0));
  }
  T det = basis_foo.determinant();
  // sanity check
  SOPHUS_ENSURE(abs(det - T(1)) < Constants<T>::epsilon(),
                "Determinant of basis is not 1, but %. Basis is \n%\n", det,
                basis_foo);
  return basis_foo;
}

/// Takes in plane normal in reference frame foo and constructs a corresponding
/// rotation matrix ``R_foo_plane``.
///
/// See ``rotationFromNormal`` for details.
///
template <class T>
SO3<T> SO3FromNormal(Vector3<T> const& normal_foo) {
  return SO3<T>(rotationFromNormal(normal_foo));
}

/// Returns a line (wrt. to frame ``foo``), given a pose of the ``line`` in
/// reference frame ``foo``.
///
/// Note: The plane is defined by X-axis of the ``line`` frame.
///
template <class T>
Line2<T> lineFromSE2(SE2<T> const& T_foo_line) {
  return Line2<T>(normalFromSO2(T_foo_line.so2()), T_foo_line.translation());
}

/// Returns the pose ``T_foo_line``, given a line in reference frame ``foo``.
///
/// Note: The line is defined by X-axis of the frame ``line``.
///
template <class T>
SE2<T> SE2FromLine(Line2<T> const& line_foo) {
  T const d = line_foo.offset();
  Vector2<T> const n = line_foo.normal();
  SO2<T> const R_foo_plane = SO2FromNormal(n);
  return SE2<T>(R_foo_plane, -d * n);
}

/// Returns a plane (wrt. to frame ``foo``), given a pose of the ``plane`` in
/// reference frame ``foo``.
///
/// Note: The plane is defined by XY-plane of the frame ``plane``.
///
template <class T>
Plane3<T> planeFromSE3(SE3<T> const& T_foo_plane) {
  return Plane3<T>(normalFromSO3(T_foo_plane.so3()), T_foo_plane.translation());
}

/// Returns the pose ``T_foo_plane``, given a plane in reference frame ``foo``.
///
/// Note: The plane is defined by XY-plane of the frame ``plane``.
///
template <class T>
SE3<T> SE3FromPlane(Plane3<T> const& plane_foo) {
  T const d = plane_foo.offset();
  Vector3<T> const n = plane_foo.normal();
  SO3<T> const R_foo_plane = SO3FromNormal(n);
  return SE3<T>(R_foo_plane, -d * n);
}

/// Takes in a hyperplane and returns unique representation by ensuring that the
/// ``offset`` is not negative.
///
template <class T, int N>
Eigen::Hyperplane<T, N> makeHyperplaneUnique(
    Eigen::Hyperplane<T, N> const& plane) {
  if (plane.offset() >= 0) {
    return plane;
  }

  return Eigen::Hyperplane<T, N>(-plane.normal(), -plane.offset());
}

}  // namespace Sophus

#endif  // GEOMETRY_HPP
