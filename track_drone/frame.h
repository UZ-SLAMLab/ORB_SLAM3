//
// Created by firas on 9.4.2020.
//

#ifndef UNTITLED_FRAME_H
#define UNTITLED_FRAME_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "RobustMatcher.h"

class frame
{
 	public:
		frame();
		virtual ~frame();

		[[nodiscard]] std::vector<cv::Point2f> get_points2d() const
		{
			return list_points2d_;
		}
		[[nodiscard]] std::vector<cv::Point3f> get_points3d() const
		{
			return list_points3d_;
		}
		[[nodiscard]] std::vector<cv::Point3f> get_lines() const
		{
			return list_lines_;
		}
		[[nodiscard]] std::vector<cv::KeyPoint> get_keypoints() const
		{
			return list_keypoints_;
		}
		[[nodiscard]] cv::Mat get_descriptors() const
		{
			return descriptors_;
		}
		[[nodiscard]] std::vector<cv::Mat> get_lines4coreset() const
		{
			return list_lines_4coreset_;
		}
		[[nodiscard]] RobustMatcher get_matcher() const
		{
			return rmatcher_;
		}
	//    void calculate_line_matrix(std::vector<cv::Point3d> list_lines ,std::vector<cv::Mat> &list_lines_4coreset);
	// can be added to the coreset class (in full project its in the frame class)
	//    cv::Mat Null_space(cv::Point3d lin);
	//    void fi_SVD( cv::Mat A_Mat, double *vtt);
		void prep_for_coreset();
	//    static cv::Mat Null_line(cv::Point3d lin);
		int get_numDescriptors() const
		{
			return descriptors_.rows;
		}
		void loadframe(cv::Mat& current_frame, int& numKeyPoints2, float& ratiotest, cv::Mat& cameraMatrix);
	//    void prep_for_coreset3();
	//    cv::Mat Null_line3(cv::Point3d lin);
		cv::Mat trackprevframe;

		/** The current number of correspondences */
		int n_correspondences_;
		/** The list of 2D points on the model surface */
		std::vector<cv::KeyPoint> list_keypoints_;
		/** The list of 2D points on the model surface */
		std::vector<cv::Point2f> list_points2d_;
		/** The list of 3D points on the model surface */
		std::vector<cv::Point3f> list_lines_;
		std::vector<cv::Mat> list_lines_4coreset_;
		/** The list of 3D points on the model surface */
		std::vector<cv::Point3f> list_points3d_;
		/** The list of 2D points descriptors */
		cv::Mat descriptors_;
		/** The list of 2D points descriptors */
		RobustMatcher rmatcher_;

};

#endif //UNTITLED_FRAME_H
