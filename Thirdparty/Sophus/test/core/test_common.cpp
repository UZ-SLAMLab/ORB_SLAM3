#include <iostream>

#include <sophus/test_macros.hpp>
#include <sophus/formatstring.hpp>

namespace Sophus {

namespace {

bool testFormatString() {
  bool passed = true;
  SOPHUS_TEST_EQUAL(passed, details::FormatString(), std::string());
  std::string test_str = "Hello World!";
  SOPHUS_TEST_EQUAL(passed, details::FormatString(test_str.c_str()), test_str);
  SOPHUS_TEST_EQUAL(passed, details::FormatString("Number: %", 5),
                    std::string("Number: 5"));
  SOPHUS_TEST_EQUAL(passed,
                    details::FormatString("Real: % msg %", 1.5, test_str),
                    std::string("Real: 1.5 msg Hello World!"));
  SOPHUS_TEST_EQUAL(passed,
                    details::FormatString(
                        "vec: %", Eigen::Vector3f(0.f, 1.f, 1.5f).transpose()),
                    std::string("vec:   0   1 1.5"));
  SOPHUS_TEST_EQUAL(
      passed, details::FormatString("Number: %", 1, 2),
      std::string("Number: 1\nFormat-Warning: There are 1 args unused."));
  return passed;
}

bool testSmokeDetails() {
  bool passed = true;
  std::cout << details::pretty(4.2) << std::endl;
  std::cout << details::pretty(Vector2f(1, 2)) << std::endl;
  bool dummy = true;
  details::testFailed(dummy, "dummyFunc", "dummyFile", 99,
                      "This is just a pratice alarm!");
  SOPHUS_TEST_EQUAL(passed, dummy, false);

  double val = transpose(42.0);
  SOPHUS_TEST_EQUAL(passed, val, 42.0);
  Matrix<float, 1, 2> row = transpose(Vector2f(1, 7));
  Matrix<float, 1, 2> expected_row(1, 7);
  SOPHUS_TEST_EQUAL(passed, row, expected_row);

  optional<int> opt(nullopt);
  SOPHUS_TEST(passed, !opt);

  return passed;
}

void runAll() {
  std::cerr << "Common tests:" << std::endl;
  bool passed = testFormatString();
  passed &= testSmokeDetails();
  processTestResult(passed);
}

}  // namespace
}  // namespace Sophus

int main() { Sophus::runAll(); }
