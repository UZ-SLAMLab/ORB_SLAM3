//
// Created by firas on 10/6/20.
//

#include "Projection.h"
#include <opencv2/core.hpp>
#include <utility>

Projection::Projection(cv::Mat  R, cv::Mat  t, cv::Mat  K) : R_matrix_(std::move(R)), t_matrix_(std::move(t)), K_matrix_(std::move(K))
{
}

void Projection::set_P_matirx(const cv::Mat& R_matrix, const cv::Mat& t_matrix)
{
//    std::cout<<R_matrix_<<std::endl;
	R_matrix_.convertTo(R_matrix_, CV_32FC1);
	t_matrix_.convertTo(t_matrix_, CV_32FC1);
	cv::hconcat(R_matrix_, t_matrix_, P_matrix_);
}

// Given the mesh, backproject the 3D points to 2D to verify the pose estimation
std::vector<cv::Point2f> Projection::verify_points(const std::vector<cv::Point3f>& map)
{
	std::vector<cv::Point2f> verified_points_2d;
	for (const auto& point3d : map)
	{
		cv::Point2f point2d = this->backproject3DPoint(point3d);
		verified_points_2d.push_back(point2d);
	}

	return verified_points_2d;
}
cv::Point2f Projection::backproject3DPoint(const cv::Point3f& point3d)
{
	// 3D point vector [x y z 1]'
	cv::Mat point3d_vec = cv::Mat(4, 1, CV_32FC1);
	point3d_vec.at<float>(0) = point3d.x;
	point3d_vec.at<float>(1) = point3d.y;
	point3d_vec.at<float>(2) = point3d.z;
	point3d_vec.at<float>(3) = 1;

	// 2D point vector [u v 1]'
	cv::Mat point2d_vec = cv::Mat(4, 1, CV_32FC1);
	point2d_vec = K_matrix_ * P_matrix_ * point3d_vec;

	// Normalization of [u v]'
	cv::Point2f point2d;
	point2d.x = (float)(point2d_vec.at<float>(0) / point2d_vec.at<float>(2));
	point2d.y = (float)(point2d_vec.at<float>(1) / point2d_vec.at<float>(2));

	return point2d;
}
