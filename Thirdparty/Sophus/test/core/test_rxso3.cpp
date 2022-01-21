#include <iostream>

#include <sophus/rxso3.hpp>
#include "tests.hpp"

// Explicit instantiate all class templates so that all member methods
// get compiled and for code coverage analysis.
namespace Eigen {
template class Map<Sophus::RxSO3<double>>;
template class Map<Sophus::RxSO3<double> const>;
}  // namespace Eigen

namespace Sophus {

template class RxSO3<double, Eigen::AutoAlign>;
template class RxSO3<float, Eigen::DontAlign>;
#if SOPHUS_CERES
template class RxSO3<ceres::Jet<double, 3>>;
#endif

template <class Scalar_>
class Tests {
 public:
  using Scalar = Scalar_;
  using SO3Type = SO3<Scalar>;
  using RxSO3Type = RxSO3<Scalar>;
  using RotationMatrixType = typename SO3<Scalar>::Transformation;
  using Point = typename RxSO3<Scalar>::Point;
  using Tangent = typename RxSO3<Scalar>::Tangent;
  Scalar const kPi = Constants<Scalar>::pi();

  Tests() {
    rxso3_vec_.push_back(RxSO3Type::exp(
        Tangent(Scalar(0.2), Scalar(0.5), Scalar(0.0), Scalar(1.))));
    rxso3_vec_.push_back(RxSO3Type::exp(
        Tangent(Scalar(0.2), Scalar(0.5), Scalar(-1.0), Scalar(1.1))));
    rxso3_vec_.push_back(RxSO3Type::exp(
        Tangent(Scalar(0.), Scalar(0.), Scalar(0.), Scalar(1.1))));
    rxso3_vec_.push_back(RxSO3Type::exp(
        Tangent(Scalar(0.), Scalar(0.), Scalar(0.00001), Scalar(0.))));
    rxso3_vec_.push_back(RxSO3Type::exp(
        Tangent(Scalar(0.), Scalar(0.), Scalar(0.00001), Scalar(0.00001))));
    rxso3_vec_.push_back(RxSO3Type::exp(
        Tangent(Scalar(0.), Scalar(0.), Scalar(0.00001), Scalar(0))));
    rxso3_vec_.push_back(
        RxSO3Type::exp(Tangent(kPi, Scalar(0), Scalar(0), Scalar(0.9))));
    rxso3_vec_.push_back(
        RxSO3Type::exp(
            Tangent(Scalar(0.2), Scalar(-0.5), Scalar(0), Scalar(0))) *
        RxSO3Type::exp(Tangent(kPi, Scalar(0), Scalar(0), Scalar(0))) *
        RxSO3Type::exp(
            Tangent(-Scalar(0.2), Scalar(-0.5), Scalar(0), Scalar(0))));
    rxso3_vec_.push_back(
        RxSO3Type::exp(
            Tangent(Scalar(0.3), Scalar(0.5), Scalar(0.1), Scalar(0))) *
        RxSO3Type::exp(Tangent(kPi, Scalar(0), Scalar(0), Scalar(0))) *
        RxSO3Type::exp(
            Tangent(Scalar(-0.3), Scalar(-0.5), Scalar(-0.1), Scalar(0))));

    Tangent tmp;
    tmp << Scalar(0), Scalar(0), Scalar(0), Scalar(0);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(1), Scalar(0), Scalar(0), Scalar(0);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(1), Scalar(0), Scalar(0), Scalar(0.1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(0), Scalar(1), Scalar(0), Scalar(0.1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(0), Scalar(0), Scalar(1), Scalar(-0.1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(-1), Scalar(1), Scalar(0), Scalar(-0.1);
    tangent_vec_.push_back(tmp);
    tmp << Scalar(20), Scalar(-1), Scalar(0), Scalar(2);
    tangent_vec_.push_back(tmp);

    point_vec_.push_back(Point(Scalar(1), Scalar(2), Scalar(4)));
    point_vec_.push_back(Point(Scalar(1), Scalar(-3), Scalar(0.5)));
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
    LieGroupTests<RxSO3Type> tests(rxso3_vec_, tangent_vec_, point_vec_);
    return tests.doAllTestsPass();
  }

  bool testSaturation() {
    bool passed = true;
    RxSO3Type small1(Constants<Scalar>::epsilon(), SO3Type());
    RxSO3Type small2(Constants<Scalar>::epsilon(),
                     SO3Type::exp(Vector3<Scalar>(Constants<Scalar>::pi(),
                                                  Scalar(0), Scalar(0))));
    RxSO3Type saturated_product = small1 * small2;
    SOPHUS_TEST_APPROX(passed, saturated_product.scale(),
                       Constants<Scalar>::epsilon(),
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, saturated_product.so3().matrix(),
                       (small1.so3() * small2.so3()).matrix(),
                       Constants<Scalar>::epsilon());
    return passed;
  }

  bool testRawDataAcces() {
    bool passed = true;
    Eigen::Matrix<Scalar, 4, 1> raw = {Scalar(0), Scalar(1), Scalar(0),
                                       Scalar(0)};
    Eigen::Map<RxSO3Type const> map_of_const_rxso3(raw.data());
    SOPHUS_TEST_APPROX(passed, map_of_const_rxso3.quaternion().coeffs().eval(),
                       raw, Constants<Scalar>::epsilon());
    SOPHUS_TEST_EQUAL(passed, map_of_const_rxso3.quaternion().coeffs().data(),
                      raw.data());
    Eigen::Map<RxSO3Type const> const_shallow_copy = map_of_const_rxso3;
    SOPHUS_TEST_EQUAL(passed, const_shallow_copy.quaternion().coeffs().eval(),
                      map_of_const_rxso3.quaternion().coeffs().eval());

    Eigen::Matrix<Scalar, 4, 1> raw2 = {Scalar(1), Scalar(0), Scalar(0),
                                        Scalar(0)};
    Eigen::Map<RxSO3Type> map_of_rxso3(raw.data());
    Eigen::Quaternion<Scalar> quat;
    quat.coeffs() = raw2;
    map_of_rxso3.setQuaternion(quat);
    SOPHUS_TEST_APPROX(passed, map_of_rxso3.quaternion().coeffs().eval(), raw2,
                       Constants<Scalar>::epsilon());
    SOPHUS_TEST_EQUAL(passed, map_of_rxso3.quaternion().coeffs().data(),
                      raw.data());
    SOPHUS_TEST_NEQ(passed, map_of_rxso3.quaternion().coeffs().data(),
                    quat.coeffs().data());
    Eigen::Map<RxSO3Type> shallow_copy = map_of_rxso3;
    SOPHUS_TEST_EQUAL(passed, shallow_copy.quaternion().coeffs().eval(),
                      map_of_rxso3.quaternion().coeffs().eval());

    RxSO3Type const const_so3(quat);
    for (int i = 0; i < 4; ++i) {
      SOPHUS_TEST_EQUAL(passed, const_so3.data()[i], raw2.data()[i]);
    }

    RxSO3Type so3(quat);
    for (int i = 0; i < 4; ++i) {
      so3.data()[i] = raw[i];
    }

    for (int i = 0; i < 4; ++i) {
      SOPHUS_TEST_EQUAL(passed, so3.data()[i], raw.data()[i]);
    }

    // regression: test that rotationMatrix API doesn't change underlying value
    // for non-const-map and compiles at all for const-map
    Eigen::Matrix<Scalar, 4, 1> raw3 = {Scalar(2), Scalar(0), Scalar(0),
                                        Scalar(0)};
    Eigen::Map<RxSO3Type> map_of_rxso3_3(raw3.data());
    Eigen::Map<const RxSO3Type> const_map_of_rxso3_3(raw3.data());
    RxSO3Type rxso3_copy3 = map_of_rxso3_3;
    const RotationMatrixType r_ref = map_of_rxso3_3.so3().matrix();

    const RotationMatrixType r = map_of_rxso3_3.rotationMatrix();
    SOPHUS_TEST_APPROX(passed, r_ref, r, Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(passed, map_of_rxso3_3.quaternion().coeffs().eval(),
                       rxso3_copy3.quaternion().coeffs().eval(),
                       Constants<Scalar>::epsilon());

    const RotationMatrixType r_const = const_map_of_rxso3_3.rotationMatrix();
    SOPHUS_TEST_APPROX(passed, r_ref, r_const, Constants<Scalar>::epsilon());
    SOPHUS_TEST_APPROX(
        passed, const_map_of_rxso3_3.quaternion().coeffs().eval(),
        rxso3_copy3.quaternion().coeffs().eval(), Constants<Scalar>::epsilon());

    Eigen::Matrix<Scalar, 4, 1> data1, data2;
    data1 << Scalar(.1), Scalar(.2), Scalar(.3), Scalar(.4);
    data2 << Scalar(.5), Scalar(.4), Scalar(.3), Scalar(.2);

    Eigen::Map<RxSO3Type> map1(data1.data()), map2(data2.data());

    // map -> map assignment
    map2 = map1;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), map2.matrix());

    // map -> type assignment
    RxSO3Type copy;
    copy = map1;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), copy.matrix());

    // type -> map assignment
    copy = RxSO3Type::exp(Tangent(Scalar(0.2), Scalar(0.5),
                                  Scalar(-1.0), Scalar(1.1)));
    map1 = copy;
    SOPHUS_TEST_EQUAL(passed, map1.matrix(), copy.matrix());

    return passed;
  }

  bool testConstructors() {
    bool passed = true;
    RxSO3Type rxso3;
    Scalar scale(1.2);
    rxso3.setScale(scale);
    SOPHUS_TEST_APPROX(passed, scale, rxso3.scale(),
                       Constants<Scalar>::epsilon(), "setScale");
    auto so3 = rxso3_vec_[0].so3();
    rxso3.setSO3(so3);
    SOPHUS_TEST_APPROX(passed, scale, rxso3.scale(),
                       Constants<Scalar>::epsilon(), "setScale");
    SOPHUS_TEST_APPROX(passed, RxSO3Type(scale, so3).matrix(), rxso3.matrix(),
                       Constants<Scalar>::epsilon(), "RxSO3(scale, SO3)");
    SOPHUS_TEST_APPROX(passed, RxSO3Type(scale, so3.matrix()).matrix(),
                       rxso3.matrix(), Constants<Scalar>::epsilon(),
                       "RxSO3(scale, SO3)");
    Matrix3<Scalar> R =
        SO3<Scalar>::exp(Point(Scalar(0.2), Scalar(0.5), Scalar(-1.0)))
            .matrix();
    Matrix3<Scalar> sR = R * Scalar(1.3);
    SOPHUS_TEST_APPROX(passed, RxSO3Type(sR).matrix(), sR,
                       Constants<Scalar>::epsilon(), "RxSO3(sR)");
    rxso3.setScaledRotationMatrix(sR);
    SOPHUS_TEST_APPROX(passed, sR, rxso3.matrix(), Constants<Scalar>::epsilon(),
                       "setScaleRotationMatrix");
    rxso3.setScale(scale);
    rxso3.setRotationMatrix(R);
    SOPHUS_TEST_APPROX(passed, R, rxso3.rotationMatrix(),
                       Constants<Scalar>::epsilon(), "setRotationMatrix");
    SOPHUS_TEST_APPROX(passed, scale, rxso3.scale(),
                       Constants<Scalar>::epsilon(), "setScale");

    return passed;
  }

  template <class S = Scalar>
  enable_if_t<std::is_floating_point<S>::value, bool> testFit() {
    bool passed = true;
    for (int i = 0; i < 10; ++i) {
      Matrix3<Scalar> M = Matrix3<Scalar>::Random();
      for (Scalar scale : {Scalar(0.01), Scalar(0.99), Scalar(1), Scalar(10)}) {
        Matrix3<Scalar> R = makeRotationMatrix(M);
        Matrix3<Scalar> sR = scale * R;
        SOPHUS_TEST(passed, isScaledOrthogonalAndPositive(sR),
                    "isScaledOrthogonalAndPositive(sR): % *\n%", scale, R);
        Matrix3<Scalar> sR_cols_swapped;
        sR_cols_swapped << sR.col(1), sR.col(0), sR.col(2);
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

  std::vector<RxSO3Type, Eigen::aligned_allocator<RxSO3Type>> rxso3_vec_;
  std::vector<Tangent, Eigen::aligned_allocator<Tangent>> tangent_vec_;
  std::vector<Point, Eigen::aligned_allocator<Point>> point_vec_;
};

int test_rxso3() {
  using std::cerr;
  using std::endl;

  cerr << "Test RxSO3" << endl << endl;
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

int main() { return Sophus::test_rxso3(); }
