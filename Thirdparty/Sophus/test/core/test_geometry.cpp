#include <iostream>

#include <sophus/geometry.hpp>
#include <sophus/test_macros.hpp>
#include "tests.hpp"

namespace Sophus {

namespace {

template <class T>
bool test2dGeometry() {
  bool passed = true;
  T const eps = Constants<T>::epsilon();

  for (int i = 0; i < 20; ++i) {
    // Roundtrip test:
    Vector2<T> normal_foo = Vector2<T>::Random().normalized();
    Sophus::SO2<T> R_foo_plane = SO2FromNormal(normal_foo);
    Vector2<T> resultNormal_foo = normalFromSO2(R_foo_plane);
    SOPHUS_TEST_APPROX(passed, normal_foo, resultNormal_foo, eps);
  }

  for (int i = 0; i < 20; ++i) {
    // Roundtrip test:
    Line2<T> line_foo = makeHyperplaneUnique(
        Line2<T>(Vector2<T>::Random().normalized(), Vector2<T>::Random()));
    Sophus::SE2<T> T_foo_plane = SE2FromLine(line_foo);
    Line2<T> resultPlane_foo = lineFromSE2(T_foo_plane);
    SOPHUS_TEST_APPROX(passed, line_foo.normal().eval(),
                       resultPlane_foo.normal().eval(), eps);
    SOPHUS_TEST_APPROX(passed, line_foo.offset(), resultPlane_foo.offset(),
                       eps);
  }

  std::vector<SE2<T>, Eigen::aligned_allocator<SE2<T>>> Ts_foo_line =
      getTestSE2s<T>();

  for (SE2<T> const& T_foo_line : Ts_foo_line) {
    Line2<T> line_foo = lineFromSE2(T_foo_line);
    SE2<T> T2_foo_line = SE2FromLine(line_foo);
    Line2<T> line2_foo = lineFromSE2(T2_foo_line);
    SOPHUS_TEST_APPROX(passed, line_foo.normal().eval(),
                       line2_foo.normal().eval(), eps);
    SOPHUS_TEST_APPROX(passed, line_foo.offset(), line2_foo.offset(), eps);
  }

  return passed;
}

template <class T>
bool test3dGeometry() {
  bool passed = true;
  T const eps = Constants<T>::epsilon();

  Vector3<T> normal_foo = Vector3<T>(1, 2, 0).normalized();
  Matrix3<T> R_foo_plane = rotationFromNormal(normal_foo);
  SOPHUS_TEST_APPROX(passed, normal_foo, R_foo_plane.col(2).eval(), eps);
  // Just testing that the function normalizes the input normal and hint
  // direction correctly:
  Matrix3<T> R2_foo_plane = rotationFromNormal((T(0.9) * normal_foo).eval());
  SOPHUS_TEST_APPROX(passed, normal_foo, R2_foo_plane.col(2).eval(), eps);
  Matrix3<T> R3_foo_plane =
      rotationFromNormal(normal_foo, Vector3<T>(T(0.9), T(0), T(0)),
                         Vector3<T>(T(0), T(1.1), T(0)));
  SOPHUS_TEST_APPROX(passed, normal_foo, R3_foo_plane.col(2).eval(), eps);

  normal_foo = Vector3<T>(1, 0, 0);
  R_foo_plane = rotationFromNormal(normal_foo);
  SOPHUS_TEST_APPROX(passed, normal_foo, R_foo_plane.col(2).eval(), eps);
  SOPHUS_TEST_APPROX(passed, Vector3<T>(0, 1, 0), R_foo_plane.col(1).eval(),
                     eps);

  normal_foo = Vector3<T>(0, 1, 0);
  R_foo_plane = rotationFromNormal(normal_foo);
  SOPHUS_TEST_APPROX(passed, normal_foo, R_foo_plane.col(2).eval(), eps);
  SOPHUS_TEST_APPROX(passed, Vector3<T>(1, 0, 0), R_foo_plane.col(0).eval(),
                     eps);

  for (int i = 0; i < 20; ++i) {
    // Roundtrip test:
    Vector3<T> normal_foo = Vector3<T>::Random().normalized();
    Sophus::SO3<T> R_foo_plane = SO3FromNormal(normal_foo);
    Vector3<T> resultNormal_foo = normalFromSO3(R_foo_plane);
    SOPHUS_TEST_APPROX(passed, normal_foo, resultNormal_foo, eps);
  }

  for (int i = 0; i < 20; ++i) {
    // Roundtrip test:
    Plane3<T> plane_foo = makeHyperplaneUnique(
        Plane3<T>(Vector3<T>::Random().normalized(), Vector3<T>::Random()));
    Sophus::SE3<T> T_foo_plane = SE3FromPlane(plane_foo);
    Plane3<T> resultPlane_foo = planeFromSE3(T_foo_plane);
    SOPHUS_TEST_APPROX(passed, plane_foo.normal().eval(),
                       resultPlane_foo.normal().eval(), eps);
    SOPHUS_TEST_APPROX(passed, plane_foo.offset(), resultPlane_foo.offset(),
                       eps);
  }

  std::vector<SE3<T>, Eigen::aligned_allocator<SE3<T>>> Ts_foo_plane =
      getTestSE3s<T>();

  for (SE3<T> const& T_foo_plane : Ts_foo_plane) {
    Plane3<T> plane_foo = planeFromSE3(T_foo_plane);
    SE3<T> T2_foo_plane = SE3FromPlane(plane_foo);
    Plane3<T> plane2_foo = planeFromSE3(T2_foo_plane);
    SOPHUS_TEST_APPROX(passed, plane_foo.normal().eval(),
                       plane2_foo.normal().eval(), eps);
    SOPHUS_TEST_APPROX(passed, plane_foo.offset(), plane2_foo.offset(), eps);
  }

  return passed;
}

void runAll() {
  std::cerr << "Geometry (Lines/Planes) tests:" << std::endl;
  std::cerr << "Double tests: " << std::endl;
  bool passed = test2dGeometry<double>();
  passed = test3dGeometry<double>();
  processTestResult(passed);
  std::cerr << "Float tests: " << std::endl;
  passed = test2dGeometry<float>();
  passed = test3dGeometry<float>();
  processTestResult(passed);
}

}  // namespace
}  // namespace Sophus

int main() { Sophus::runAll(); }
