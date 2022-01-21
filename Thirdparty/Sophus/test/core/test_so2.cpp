#include <iostream>

#include <sophus/so2.hpp>
#include "tests.hpp"

// Explicit instantiate all class templates so that all member methods
// get compiled and for code coverage analysis.
namespace Eigen {
template class Map<Sophus::SO2<double>>;
template class Map<Sophus::SO2<double> const>;
}  // namespace Eigen

namespace Sophus {

template class SO2<double, Eigen::AutoAlign>;
template class SO2<float, Eigen::DontAlign>;
#if SOPHUS_CERES
template class SO2<ceres::Jet<double, 3>>;
#endif

template <class Scalar>
class Tests {
 public:
  using SO2Type = SO2<Scalar>;
  using Point = typename SO2<Scalar>::Point;
  using Tangent = typename SO2<Scalar>::Tangent;
  Scalar const kPi = Constants<Scalar>::pi();

  Tests() {
    so2_vec_.push_back(SO2Type::exp(Scalar(0.0)));
    so2_vec_.push_back(SO2Type::exp(Scalar(0.2)));
    so2_vec_.push_back(SO2Type::exp(Scalar(10.)));
    so2_vec_.push_back(SO2Type::exp(Scalar(0.00001)));
    so2_vec_.push_back(SO2Type::exp(kPi));
    so2_vec_.push_back(SO2Type::exp(Scalar(0.2)) * SO2Type::exp(kPi) *
                       SO2Type::exp(Scalar(-0.2)));
    so2_vec_.push_back(SO2Type::exp(Scalar(-0.3)) * SO2Type::exp(kPi) *
                       SO2Type::exp(Scalar(0.3)));

    tangent_vec_.push_back(Tangent(Scalar(0)));
    tangent_vec_.push_back(Tangent(Scalar(1)));
    tangent_vec_.push_back(Tangent(Scalar(kPi / 2.)));
    tangent_vec_.push_back(Tangent(Scalar(-1)));
    tangent_vec_.push_back(Tangent(Scalar(20)));
    tangent_vec_.push_back(Tangent(Scalar(kPi / 2. + 0.0001)));

    point_vec_.push_back(Point(Scalar(1), Scalar(2)));
    point_vec_.push_back(Point(Scalar(1), Scalar(-3)));
  }

  void runAll() {
    bool passed = testLieProperties();
    passed &= testUnity();
    passed &= testRawDataAcces();
    passed &= testConstructors();
    passed &= testFit();
    processTestResult(passed);
  }

 private:
  bool testLieProperties() {
    LieGroupTests<SO2Type> tests(so2_vec_, tangent_vec_, point_vec_);
    return tests.doAllTestsPass();
  }

  bool testUnity() {
    bool passed = true;
    // Test that the complex number magnitude stays close to one.
    SO2Type current_q;
    for (std::size_t i = 0; i < 1000; ++i) {
      for (SO2Type const& q : so2_vec_) {
        current_q *= q;
      }
    }
    SOPHUS_TEST_APPROX(passed, current_q.unit_complex().norm(), Scalar(1),
                       Constants<Scalar>::epsilon(), "Magnitude drift");
    return passed;
  }

  bool testRawDataAcces() {
    bool passed = true;
    Vector2<Scalar> raw = {0, 1};
    Eigen::Map<SO2Type const> map_of_const_so2(raw.data());
    SOPHUS_TEST_APPROX(passed, map_of_const_so2.unit_complex().eval(), raw,
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_EQUAL(passed, map_of_const_so2.unit_complex().data(),
                      raw.data());
    Eigen::Map<SO2Type const> const_shallow_copy = map_of_const_so2;
    SOPHUS_TEST_EQUAL(passed, const_shallow_copy.unit_complex().eval(),
                      map_of_const_so2.unit_complex().eval());

    Vector2<Scalar> raw2 = {1, 0};
    Eigen::Map<SO2Type> map_of_so2(raw.data());
    map_of_so2.setComplex(raw2);
    SOPHUS_TEST_APPROX(passed, map_of_so2.unit_complex().eval(), raw2,
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_EQUAL(passed, map_of_so2.unit_complex().data(), raw.data());
    SOPHUS_TEST_NEQ(passed, map_of_so2.unit_complex().data(), raw2.data());
    Eigen::Map<SO2Type> shallow_copy = map_of_so2;
    SOPHUS_TEST_EQUAL(passed, shallow_copy.unit_complex().eval(),
                      map_of_so2.unit_complex().eval());

    SO2Type const const_so2(raw2);
    for (int i = 0; i < 2; ++i) {
      SOPHUS_TEST_EQUAL(passed, const_so2.data()[i], raw2.data()[i]);
    }

    SO2Type so2(raw2);
    for (int i = 0; i < 2; ++i) {
      so2.data()[i] = raw[i];
    }

    for (int i = 0; i < 2; ++i) {
      SOPHUS_TEST_EQUAL(passed, so2.data()[i], raw.data()[i]);
    }

    Vector2<Scalar> data1 = {1, 0}, data2 = {0, 1};
    Eigen::Map<SO2Type> map1(data1.data()), map2(data2.data());

    // map -> map assignment
    map2 = map1;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), map2.matrix());

    // map -> type assignment
    SO2Type copy;
    copy = map1;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), copy.matrix());

    // type -> map assignment
    copy = SO2Type(Scalar(0.5));
    map1 = copy;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), copy.matrix());

    return passed;
  }

  bool testConstructors() {
    bool passed = true;
    Matrix2<Scalar> R = so2_vec_.front().matrix();
    SO2Type so2(R);
    SOPHUS_TEST_APPROX(passed, R, so2.matrix(), Constants<Scalar>::epsilon());

    return passed;
  }

  template <class S = Scalar>
  enable_if_t<std::is_floating_point<S>::value, bool> testFit() {
    bool passed = true;

    for (int i = 0; i < 100; ++i) {
      Matrix2<Scalar> R = Matrix2<Scalar>::Random();
      SO2Type so2 = SO2Type::fitToSO2(R);
      SO2Type so2_2 = SO2Type::fitToSO2(so2.matrix());

      SOPHUS_TEST_APPROX(passed, so2.matrix(), so2_2.matrix(),
                         Constants<Scalar>::epsilon());
    }
    return passed;
  }

  template <class S = Scalar>
  enable_if_t<!std::is_floating_point<S>::value, bool> testFit() {
    return true;
  }

  std::vector<SO2Type, Eigen::aligned_allocator<SO2Type>> so2_vec_;
  std::vector<Tangent, Eigen::aligned_allocator<Tangent>> tangent_vec_;
  std::vector<Point, Eigen::aligned_allocator<Point>> point_vec_;
};

int test_so2() {
  using std::cerr;
  using std::endl;

  cerr << "Test SO2" << endl << endl;
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

int main() { return Sophus::test_so2(); }
