//
// Created by firas on 10/6/20.
//

#include "Projection.h"
#include <opencv2/core.hpp>
#include <iostream>




Projection::Projection(const cv::Mat& R, const cv::Mat& t, const cv::Mat& K) :     R_matrix_(R),t_matrix_(t) ,K_matrix_(K) { }

void Projection::set_P_matirx(cv::Mat R_matrix, cv::Mat t_matrix) {
//    std::cout<<R_matrix_<<std::endl;
    R_matrix_.convertTo(R_matrix_,CV_32FC1);
//    std::cout<<R_matrix_<<std::endl;
//    std::cout<<t_matrix_<<std::endl;
    t_matrix_.convertTo(t_matrix_,CV_32FC1);
//    std::cout<<t_matrix_<<std::endl;
    cv::hconcat(R_matrix_,t_matrix_,P_matrix_);
//    std::cout<<P_matrix_<<std::endl;
    int a=0;

}

// Given the mesh, backproject the 3D points to 2D to verify the pose estimation
std::vector<cv::Point2f> Projection::verify_points(std::vector <cv::Point3f> map)
{
    std::vector<cv::Point2f> verified_points_2d;
    for(const auto& point3d : map)
    {
        cv::Point2f point2d = this->backproject3DPoint(point3d);
        verified_points_2d.push_back(point2d);
    }

    return verified_points_2d;
}
cv::Point2f Projection::backproject3DPoint(const cv::Point3f &point3d)
{
    // 3D point vector [x y z 1]'
    cv::Mat point3d_vec = cv::Mat(4, 1, CV_32FC1);
    point3d_vec.at<float>(0) = point3d.x;
    point3d_vec.at<float>(1) = point3d.y;
    point3d_vec.at<float>(2) = point3d.z;
    point3d_vec.at<float>(3) = 1;

    // 2D point vector [u v 1]'
    cv::Mat point2d_vec = cv::Mat(4, 1, CV_32FC1);
//    std::cout<<K_matrix_<<std::endl;
//    std::cout<<P_matrix_<<std::endl;
//    std::cout<<point3d_vec<<std::endl;
    point2d_vec = K_matrix_ * P_matrix_ * point3d_vec;
//    std::cout<<point2d_vec<<std::endl;

    // Normalization of [u v]'
    cv::Point2f point2d;
    point2d.x = (float)(point2d_vec.at<float>(0) / point2d_vec.at<float>(2));
    point2d.y = (float)(point2d_vec.at<float>(1) / point2d_vec.at<float>(2));

    return point2d;
}