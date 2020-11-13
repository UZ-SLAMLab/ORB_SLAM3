//
// Created by firas on 9.4.2020.
//
#include "RobustMatcher.h"
#include "Utils.h"

#include "frame.h"

frame::frame() : n_correspondences_(0), list_keypoints_(0), list_points2d_(0), list_lines_(0), list_points3d_(0)
{
}

frame::~frame() = default;

void frame::loadframe(cv::Mat& current_frame, int& numKeyPoints2, float& ratiotest, cv::Mat& cameraMatrix)
{
	cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(numKeyPoints2);
	cv::Ptr<cv::FeatureDetector> descriptor = cv::ORB::create(numKeyPoints2);
	rmatcher_.setFeatureDetector(detector);                                // set feature detector
	rmatcher_.setDescriptorExtractor(descriptor);                          // set descriptor extractor
	rmatcher_.setDescriptorMatcher(createMatcher("ORB", true));     // set matcher
	rmatcher_.setRatio(ratiotest);
	rmatcher_.computeKeyPoints(current_frame, list_keypoints_);
	rmatcher_.calcdesriptorsperframe(current_frame, list_keypoints_, descriptors_);
	std::vector<cv::Point2f> temppoint2d;
	std::vector<cv::Point3f> templines;
	for (auto& list_keypoint : list_keypoints_)
	{
		cv::Point2f point2d = list_keypoint.pt; // 2D point from the scene
		temppoint2d.push_back(point2d);         // add 2D point
		cv::Mat pt = (cv::Mat_<float>(3, 1) << point2d.x, point2d.y, 1);
		cv::Mat invK = cameraMatrix.inv();
		cv::Mat templine = invK * pt;
		templine = templine / norm(templine);
		cv::Point3f v(templine);
		templines.push_back(v);

	}
	list_lines_ = templines;
	list_points2d_ = temppoint2d;
}
