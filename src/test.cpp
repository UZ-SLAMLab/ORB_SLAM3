#include "MLPnpsolver.h"
#include "gtest/gtest.h"

TEST(MLPnpsolverTest, TestSolvePNP) {
  // Generate random test data
  std::vector<cv::Point3f> pts3d;
  std::vector<cv::Point2f> pts2d;
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  int num_points = 10;
  for (int i = 0; i < num_points; i++) {
    pts3d.push_back(cv::Point3f(rand() % 100, rand() % 100, rand() % 100));
    pts2d.push_back(cv::Point2f(rand() % 100, rand() % 100));
  }

  // Call the function you want to test
  cv::Mat rvec, tvec;
  bool success = SolvePNP(pts3d, pts2d, K, rvec, tvec);

  // Check the result
  ASSERT_TRUE(success);
  ASSERT_FALSE(rvec.empty());
  ASSERT_FALSE(tvec.empty());
}
