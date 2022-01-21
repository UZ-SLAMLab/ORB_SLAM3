#include <iostream>

#include <sophus/rxso2.hpp>
#include "tests.hpp"

// Explicit instantiate all class templates so that all member methods
// get compiled and for code coverage analysis.
namespace Eigen {
template class Map<Sophus::RxSO2<double>>;
template class Map<Sophus::RxSO2<double> const>;
}  // namespace Eigen

namespace Sophus {

template class RxSO2<double, Eigen::AutoAlign>;
template class RxSO2<float, Eigen::DontAlign>;
#if SOPHUS_CERES
template class RxSO2<ceres::Jet<double, 3>>;
#endif

template <class Scalar>
class Tests {
 public:
  using SO2Type = SO2<Scalar>;
  using RxSO2Type = RxSO2<Scalar>;
  using RotationMatrixType = typename SO2<Scalar>::Transformation;
  using Point = typename RxSO2<Scalar>::Point;
  using Tangent = typename RxSO2<Scalar>::Tangent;
  Scalar const kPi = Constants<Scalar>::pi();

  Tests() {
    rxso2_vec_.push_back(RxSO2Type::exp(Tangent(0.2, 1.)));
    rxso2_vec_.push_back(RxSO2Type::exp(Tangent(0.2, 1.1)));
    rxso2_vec_.push_back(RxSO2Type::exp(Tangent(0., 1.1)));
    rxso2_vec_.push_back(RxSO2Type::exp(Tangent(0.00001, 0.)));
    rxso2_vec_.push_back(RxSO2Type::exp(Tangent(0.00001, 0.00001)));
    rxso2_vec_.push_back(RxSO2Type::exp(Tangent(kPi, 0.9)));
    rxso2_vec_.push_back(RxSO2Type::exp(Tangent(0.2, 0)) *
                         RxSO2Type::exp(Tangent(kPi, 0.0)) *
                         RxSO2Type::exp(Tangent(-0.2, 0)));
    rxso2_vec_.push_back(RxSO2Type::exp(Tangent(0.3, 0)) *
                         RxSO2Type::exp(Tangent(kPi, 0.001)) *
                         RxSO2Type::exp(Tangent(-0.3, 0)));

    Tangent tmp;
    tmp << Scalar(0), Scalar(0);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(1), Scalar(0);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(1), Scalar(0.1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(0), Scalar(0.1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(0), Scalar(-0.1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(-1), Scalar(-0.1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(20), Scalar(2);
    tangent_vec_.push_back(tmp);

    point_vec_.push_back(Point(Scalar(1), Scalar(4)));
    point_vec_.push_back(Point(Scalar(1), Scalar(-3)));
  }

  template <class S = Scalar>
  enable_if_t<std::is_floating_point<S>::value, bool> testFit() {
    bool passed = true;
    for (int i = 0; i < 10; ++i) {
      Matrix2<Scalar> M = Matrix2<Scalar>::Random();
      for (Scalar scale : {Scalar(0.01), Scalar(0.99), Scalar(1), Scalar(10)}) {
        Matrix2<Scalar> R = makeRotationMatrix(M);
        Matrix2<Scalar> sR = scale * R;
        SOPHUS_TEST(passed, isScaledOrthogonalAndPositive(sR),
                    "isScaledOrthogonalAndPositive(sR): % *\n%", scale, R);
        Matrix2<Scalar> sR_cols_swapped;
        sR_cols_swapped << sR.col(1), sR.col(0);
        SOPHUS_TEST(passed, !isScaledOrthogonalAndPositive(sR_cols_swapped),
                    "isScaledOrthogonalAndPositive(-sR): % *\n%", scale, R);
      }
    }
    return passed;
  }

  template <class S = Scalar>
  enable_if_t<!std::is_floating_point<S>::value, bool> testFit() {
    return true;
  }

  void runAll() {
    bool passed = testLieProperties();
    passed &= testSaturation();
    passed &= testRawDataAcces();
    passed &= testConstructors();
    passed &= testFit();
    processTestResult(passed);
  }

 private:
  bool testLieProperties() {
    LieGroupTests<RxSO2Type> tests(rxso2_vec_, tangent_vec_, point_vec_);
    return tests.doAllTestsPass();
  }

  bool testSaturation() {
    bool passed = true;
    RxSO2Type small1(Scalar(1.1) * Constants<Scalar>::epsilon(), SO2Type());
    RxSO2Type small2(Scalar(1.1) * Constants<Scalar>::epsilon(),
                     SO2Type::exp(Constants<Scalar>::pi()));
    RxSO2Type saturated_product = small1 * small2;
    SOPHUS_TEST_APPROX(passed, saturated_product.scale(),
                       Constants<Scalar>::epsilon(),
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, saturated_product.so2().matrix(),
                       (small1.so2() * small2.so2()).matrix(),
                       Constants<Scalar>::epsilon());
    return passed;
  }

  bool testRawDataAcces() {
    bool passed = true;
    Eigen::Matrix<Scalar, 2, 1> raw = {0, 1};
    Eigen::Map<RxSO2Type const> map_of_const_rxso2(raw.data());
    SOPHUS_TEST_APPROX(passed, map_of_const_rxso2.complex().eval(), raw,
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_EQUAL(passed, map_of_const_rxso2.complex().data(), raw.data());
    Eigen::Map<RxSO2Type const> const_shallow_copy = map_of_const_rxso2;
    SOPHUS_TEST_EQUAL(passed, const_shallow_copy.complex().eval(),
                      map_of_const_rxso2.complex().eval());

    Eigen::Matrix<Scalar, 2, 1> raw2 = {1, 0};
    Eigen::Map<RxSO2Type> map_of_rxso2(raw2.data());
    SOPHUS_TEST_APPROX(passed, map_of_rxso2.complex().eval(), raw2,
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_EQUAL(passed, map_of_rxso2.complex().data(), raw2.data());
    Eigen::Map<RxSO2Type> shallow_copy = map_of_rxso2;
    SOPHUS_TEST_EQUAL(passed, shallow_copy.complex().eval(),
                      map_of_rxso2.complex().eval());

    RxSO2Type const const_so2(raw2);
    for (int i = 0; i < 2; ++i) {
      SOPHUS_TEST_EQUAL(passed, const_so2.data()[i], raw2.data()[i]);
    }

    RxSO2Type so2(raw2);
    for (int i = 0; i < 2; ++i) {
      so2.data()[i] = raw[i];
    }

    for (int i = 0; i < 2; ++i) {
      SOPHUS_TEST_EQUAL(passed, so2.data()[i], raw.data()[i]);
    }

    // regression: test that rotationMatrix API doesn't change underlying value
    // for non-const-map and compiles at all for const-map
    Eigen::Matrix<Scalar, 2, 1> raw3 = {Scalar(2), Scalar(0)};
    Eigen::Map<RxSO2Type> map_of_rxso2_3(raw3.data());
    Eigen::Map<const RxSO2Type> const_map_of_rxso2_3(raw3.data());
    RxSO2Type rxso2_copy3 = map_of_rxso2_3;
    const RotationMatrixType r_ref = map_of_rxso2_3.so2().matrix();

    const RotationMatrixType r = map_of_rxso2_3.rotationMatrix();
    SOPHUS_TEST_APPROX(passed, r_ref, r, Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, map_of_rxso2_3.complex().eval(),
                       rxso2_copy3.complex().eval(),
                       Constants<Scalar>::epsilon());

    const RotationMatrixType r_const = const_map_of_rxso2_3.rotationMatrix();
    SOPHUS_TEST_APPROX(passed, r_ref, r_const, Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, const_map_of_rxso2_3.complex().eval(),
                       rxso2_copy3.complex().eval(),
                       Constants<Scalar>::epsilon());

    Eigen::Matrix<Scalar, 2, 1> data1, data2;
    data1 << Scalar(.1), Scalar(.2);
    data2 << Scalar(.5), Scalar(.4);

    Eigen::Map<RxSO2Type> map1(data1.data()), map2(data2.data());

    // map -> map assignment
    map2 = map1;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), map2.matrix());

    // map -> type assignment
    RxSO2Type copy;
    copy = map1;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), copy.matrix());

    // type -> map assignment
    copy = RxSO2Type::exp(Tangent(Scalar(0.2), Scalar(0.5)));
    map1 = copy;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), copy.matrix());

    return passed;
  }

  bool testConstructors() {
    bool passed = true;
    RxSO2Type rxso2;
    Scalar scale(1.2);
    rxso2.setScale(scale);
    SOPHUS_TEST_APPROX(passed, scale, rxso2.scale(),
                       Constants<Scalar>::epsilon(), "setScale");
    Scalar angle(0.2);
    rxso2.setAngle(angle);
    SOPHUS_TEST_APPROX(passed, angle, rxso2.angle(),
                       Constants<Scalar>::epsilon(), "setAngle");
    SOPHUS_TEST_APPROX(passed, scale, rxso2.scale(),
                       Constants<Scalar>::epsilon(),
                       "setAngle leaves scale as is");

    auto so2 = rxso2_vec_[0].so2();
    rxso2.setSO2(so2);
    SOPHUS_TEST_APPROX(passed, scale, rxso2.scale(),
                       Constants<Scalar>::epsilon(), "setSO2");
    SOPHUS_TEST_APPROX(passed, RxSO2Type(scale, so2).matrix(), rxso2.matrix(),
                       Constants<Scalar>::epsilon(), "RxSO2(scale, SO2)");
    SOPHUS_TEST_APPROX(passed, RxSO2Type(scale, so2.matrix()).matrix(),
                       rxso2.matrix(), Constants<Scalar>::epsilon(),
                       "RxSO2(scale, SO2)");
    Matrix2<Scalar> R = SO2<Scalar>::exp(Scalar(0.2)).matrix();
    Matrix2<Scalar> sR = R * Scalar(1.3);
    SOPHUS_TEST_APPROX(passed, RxSO2Type(sR).matrix(), sR,
                       Constants<Scalar>::epsilon(), "RxSO2(sR)");
    rxso2.setScaledRotationMatrix(sR);
    SOPHUS_TEST_APPROX(passed, sR, rxso2.matrix(), Constants<Scalar>::epsilon(),
                       "setScaleRotationMatrix");
    rxso2.setScale(scale);
    rxso2.setRotationMatrix(R);
    SOPHUS_TEST_APPROX(passed, R, rxso2.rotationMatrix(),
                       Constants<Scalar>::epsilon(), "setRotationMatrix");
    SOPHUS_TEST_APPROX(passed, scale, rxso2.scale(),
                       Constants<Scalar>::epsilon(), "setScale");

    return passed;
  }

  std::vector<RxSO2Type, Eigen::aligned_allocator<RxSO2Type>> rxso2_vec_;
  std::vector<Tangent, Eigen::aligned_allocator<Tangent>> tangent_vec_;
  std::vector<Point, Eigen::aligned_allocator<Point>> point_vec_;
};

int test_rxso2() {
  using std::cerr;
  using std::endl;

  cerr << "Test RxSO2" << endl << endl;
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

int main() { return Sophus::test_rxso2(); }
