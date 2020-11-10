
#include <vector>
#include <random>
#include "PnPProblem.h"
#include <opencv2/calib3d/calib3d.hpp>
// Custom constructor given the intrinsic camera parameters
PnPProblem::PnPProblem(const double params[])
{
    A_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
    A_matrix_.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
    A_matrix_.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
    A_matrix_.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
    A_matrix_.at<double>(1, 2) = params[3];
    A_matrix_.at<double>(2, 2) = 1;
    R_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
    t_matrix_ = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
    P_matrix_ = cv::Mat::zeros(3, 4, CV_64FC1);   // rotation-translation matrix

}

PnPProblem::~PnPProblem()
{
    // TODO Auto-generated destructor stub
}

void PnPProblem::set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix)
{
    // Rotation-Translation Matrix Definition
//    cout<<R_matrix_<<endl;
//    cout<<t_matrix_<<endl;
    P_matrix_.at<double>(0,0) = R_matrix.at<double>(0,0);
    P_matrix_.at<double>(0,1) = R_matrix.at<double>(0,1);
    P_matrix_.at<double>(0,2) = R_matrix.at<double>(0,2);
    P_matrix_.at<double>(1,0) = R_matrix.at<double>(1,0);
    P_matrix_.at<double>(1,1) = R_matrix.at<double>(1,1);
    P_matrix_.at<double>(1,2) = R_matrix.at<double>(1,2);
    P_matrix_.at<double>(2,0) = R_matrix.at<double>(2,0);
    P_matrix_.at<double>(2,1) = R_matrix.at<double>(2,1);
    P_matrix_.at<double>(2,2) = R_matrix.at<double>(2,2);
    P_matrix_.at<double>(0,3) = t_matrix.at<double>(0);
    P_matrix_.at<double>(1,3) = t_matrix.at<double>(1);
    P_matrix_.at<double>(2,3) = t_matrix.at<double>(2);
}

// Estimate the pose given a list of 2D/3D correspondences with RANSAC and the method to use

void PnPProblem::estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
                                     const std::vector<cv::Point2f> &list_points2d,     // list with scene 2D coordinates
                                     int flags, cv::Mat &inliers, int iterationsCount,  // PnP method; inliers container
                                     float reprojectionError, double confidence )    // Ransac parameters
{
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);    // output translation vector

    bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
    // initial approximations of the rotation and translation vectors

    cv::solvePnPRansac( list_points3d, list_points2d, A_matrix_, distCoeffs, rvec, tvec,
                        useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                        inliers, flags );

    Rodrigues(rvec, R_matrix_); // converts Rotation Vector to Matrix
    t_matrix_ = tvec;           // set translation matrix

    this->set_P_matrix(R_matrix_, t_matrix_); // set rotation-translation matrix

}