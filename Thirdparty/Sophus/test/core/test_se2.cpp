#include <iostream>

#include <sophus/se2.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include "tests.hpp"

// Explicit instantiate all class templates so that all member methods
// get compiled and for code coverage analysis.
namespace Eigen {
template class Map<Sophus::SE2<double>>;
template class Map<Sophus::SE2<double> const>;
}  // namespace Eigen

namespace Sophus {

template class SE2<double, Eigen::AutoAlign>;
template class SE2<double, Eigen::DontAlign>;
#if SOPHUS_CERES
template class SE2<ceres::Jet<double, 3>>;
#endif

template <class Scalar>
class Tests {
 public:
  using SE2Type = SE2<Scalar>;
  using SO2Type = SO2<Scalar>;
  using Point = typename SE2<Scalar>::Point;
  using Tangent = typename SE2<Scalar>::Tangent;
  Scalar const kPi = Constants<Scalar>::pi();

  Tests() {
    se2_vec_.push_back(
        SE2Type(SO2Type(Scalar(0.0)), Point(Scalar(0), Scalar(0))));
    se2_vec_.push_back(
        SE2Type(SO2Type(Scalar(0.2)), Point(Scalar(10), Scalar(0))));
    se2_vec_.push_back(
        SE2Type(SO2Type(Scalar(0.)), Point(Scalar(0), Scalar(100))));
    se2_vec_.push_back(
        SE2Type(SO2Type(Scalar(-1.)), Point(Scalar(20), -Scalar(1))));
    se2_vec_.push_back(
        SE2Type(SO2Type(Scalar(0.00001)),
                Point(Scalar(-0.00000001), Scalar(0.0000000001))));
    se2_vec_.push_back(
        SE2Type(SO2Type(Scalar(0.2)), Point(Scalar(0), Scalar(0))) *
        SE2Type(SO2Type(kPi), Point(Scalar(0), Scalar(0))) *
        SE2Type(SO2Type(Scalar(-0.2)), Point(Scalar(0), Scalar(0))));
    se2_vec_.push_back(
        SE2Type(SO2Type(Scalar(0.3)), Point(Scalar(2), Scalar(0))) *
        SE2Type(SO2Type(kPi), Point(Scalar(0), Scalar(0))) *
        SE2Type(SO2Type(Scalar(-0.3)), Point(Scalar(0), Scalar(6))));

    Tangent tmp;
    tmp << Scalar(0), Scalar(0), Scalar(0);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(1), Scalar(0), Scalar(0);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(0), Scalar(1), Scalar(1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(-1), Scalar(1), Scalar(0);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(20), Scalar(-1), Scalar(-1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(30), Scalar(5), Scalar(20);
    tangent_vec_.push_back(tmp);

    point_vec_.push_back(Point(1, 2));
    point_vec_.push_back(Point(1, -3));
  }

  void runAll() {
    bool passed = testLieProperties();
    passed &= testRawDataAcces();
    passed &= testMutatingAccessors();
    passed &= testConstructors();
    passed &= testFit();
    processTestResult(passed);
  }

 private:
  bool testLieProperties() {
    LieGroupTests<SE2Type> tests(se2_vec_, tangent_vec_, point_vec_);
    return tests.doAllTestsPass();
  }

  bool testRawDataAcces() {
    bool passed = true;
    Eigen::Matrix<Scalar, 4, 1> raw;
    raw << Scalar(0), Scalar(1), Scalar(0), Scalar(3);
    Eigen::Map<SE2Type const> const_se2_map(raw.data());
    SOPHUS_TEST_APPROX(passed, const_se2_map.unit_complex().eval(),
                       raw.template head<2>().eval(),
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, const_se2_map.translation().eval(),
                       raw.template tail<2>().eval(),
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_EQUAL(passed, const_se2_map.unit_complex().data(), raw.data());
    SOPHUS_TEST_EQUAL(passed, const_se2_map.translation().data(),
                      raw.data() + 2);
    Eigen::Map<SE2Type const> const_shallow_copy = const_se2_map;
    SOPHUS_TEST_EQUAL(passed, const_shallow_copy.unit_complex().eval(),
                      const_se2_map.unit_complex().eval());
    SOPHUS_TEST_EQUAL(passed, const_shallow_copy.translation().eval(),
                      const_se2_map.translation().eval());

    Eigen::Matrix<Scalar, 4, 1> raw2;
    raw2 << Scalar(1), Scalar(0), Scalar(3), Scalar(1);
    Eigen::Map<SE2Type> map_of_se3(raw.data());
    map_of_se3.setComplex(raw2.template head<2>());
    map_of_se3.translation() = raw2.template tail<2>();
    SOPHUS_TEST_APPROX(passed, map_of_se3.unit_complex().eval(),
                       raw2.template head<2>().eval(),
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, map_of_se3.translation().eval(),
                       raw2.template tail<2>().eval(),
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_EQUAL(passed, map_of_se3.unit_complex().data(), raw.data());
    SOPHUS_TEST_EQUAL(passed, map_of_se3.translation().data(), raw.data() + 2);
    SOPHUS_TEST_NEQ(passed, map_of_se3.unit_complex().data(), raw2.data());
    Eigen::Map<SE2Type> shallow_copy = map_of_se3;
    SOPHUS_TEST_EQUAL(passed, shallow_copy.unit_complex().eval(),
                      map_of_se3.unit_complex().eval());
    SOPHUS_TEST_EQUAL(passed, shallow_copy.translation().eval(),
                      map_of_se3.translation().eval());
    Eigen::Map<SE2Type> const const_map_of_se2 = map_of_se3;
    SOPHUS_TEST_EQUAL(passed, const_map_of_se2.unit_complex().eval(),
                      map_of_se3.unit_complex().eval());
    SOPHUS_TEST_EQUAL(passed, const_map_of_se2.translation().eval(),
                      map_of_se3.translation().eval());

    SE2Type const const_se2(raw2.template head<2>().eval(),
                            raw2.template tail<2>().eval());
    for (int i = 0; i < 4; ++i) {
      SOPHUS_TEST_EQUAL(passed, const_se2.data()[i], raw2.data()[i]);
    }

    SE2Type se2(raw2.template head<2>().eval(), raw2.template tail<2>().eval());
    for (int i = 0; i < 4; ++i) {
      SOPHUS_TEST_EQUAL(passed, se2.data()[i], raw2.data()[i]);
    }

    for (int i = 0; i < 4; ++i) {
      SOPHUS_TEST_EQUAL(passed, se2.data()[i], raw.data()[i]);
    }

    SE2Type trans = SE2Type::transX(Scalar(0.2));
    SOPHUS_TEST_APPROX(passed, trans.translation().x(), Scalar(0.2),
                       Constants<Scalar>::epsilon());
    trans = SE2Type::transY(Scalar(0.7));
    SOPHUS_TEST_APPROX(passed, trans.translation().y(), Scalar(0.7),
                       Constants<Scalar>::epsilon());

    Eigen::Matrix<Scalar, 4, 1> data1, data2;
    data1 << Scalar(0), Scalar(1), Scalar(1), Scalar(2);
    data1 << Scalar(1), Scalar(0),  Scalar(2), Scalar(1);

    Eigen::Map<SE2Type> map1(data1.data()), map2(data2.data());

    // map -> map assignment
    map2 = map1;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), map2.matrix());

    // map -> type assignment
    SE2Type copy;
    copy = map1;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), copy.matrix());

    // type -> map assignment
    copy = SE2Type::trans(Scalar(4), Scalar(5)) * SE2Type::rot(Scalar(0.5));
    map1 = copy;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), copy.matrix());

    return passed;
  }

  bool testMutatingAccessors() {
    bool passed = true;
    SE2Type se2;
    SO2Type R(Scalar(0.2));
    se2.setRotationMatrix(R.matrix());
    SOPHUS_TEST_APPROX(passed, se2.rotationMatrix(), R.matrix(),
                       Constants<Scalar>::epsilon());

    Eigen::Matrix<Scalar, 4, 1> raw;
    raw << Scalar(1), Scalar(0), Scalar(3), Scalar(1);
    Eigen::Map<SE2Type> map_of_se2(raw.data());
    map_of_se2.setRotationMatrix(R.matrix());
    SOPHUS_TEST_APPROX(passed, map_of_se2.rotationMatrix(), R.matrix(),
                       Constants<Scalar>::epsilon());

    return passed;
  }

  bool testConstructors() {
    bool passed = true;
    Matrix3<Scalar> I = Matrix3<Scalar>::Identity();
    SOPHUS_TEST_EQUAL(passed, SE2Type().matrix(), I);

    SE2Type se2 = se2_vec_.front();
    Point translation = se2.translation();
    SO2Type so2 = se2.so2();

    SOPHUS_TEST_APPROX(passed, SE2Type(so2.log(), translation).matrix(),
                       se2.matrix(), Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, SE2Type(so2, translation).matrix(), se2.matrix(),
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, SE2Type(so2.matrix(), translation).matrix(),
                       se2.matrix(), Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed,
                       SE2Type(so2.unit_complex(), translation).matrix(),
                       se2.matrix(), Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, SE2Type(se2.matrix()).matrix(), se2.matrix(),
                       Constants<Scalar>::epsilon());

    return passed;
  }

  template <class S = Scalar>
  enable_if_t<std::is_floating_point<S>::value, bool> testFit() {
    bool passed = true;
    for (int i = 0; i < 100; ++i) {
      Matrix3<Scalar> T = Matrix3<Scalar>::Random();
      SE2Type se2 = SE2Type::fitToSE2(T);
      SE2Type se2_2 = SE2Type::fitToSE2(se2.matrix());

      SOPHUS_TEST_APPROX(passed, se2.matrix(), se2_2.matrix(),
                         Constants<Scalar>::epsilon());
    }
    return passed;
  }

  template <class S = Scalar>
  enable_if_t<!std::is_floating_point<S>::value, bool> testFit() {
    return true;
  }

  std::vector<SE2Type, Eigen::aligned_allocator<SE2Type>> se2_vec_;
  std::vector<Tangent, Eigen::aligned_allocator<Tangent>> tangent_vec_;
  std::vector<Point, Eigen::aligned_allocator<Point>> point_vec_;
};

int test_se2() {
  using std::cerr;
  using std::endl;

  cerr << "Test SE2" << endl << endl;
  cerr << "Double tests: " << endl;
  Tests<double>().runAll();
  cerr << "Float tests: " << endl;
  Tests<float>().runAll();

#if SOPHUS_CERES
  cerr << "ceres::Jet<double, 3> tests: " << endl;
  Tests<ceres::Jet<double, 3>>().runAll();
#endif

  return 0;
}
}  // namespace Sophus

int main() { return Sophus::test_se2(); }
