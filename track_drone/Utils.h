/*
 * Utils.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>

#include <opencv2/features2d.hpp>
#include "PnPProblem.h"
#include "RobustMatcher.h"


void optimalpnp(std::vector<cv::Point3f> list_points3d_inliers,std::vector<cv::Point3f> list_lines_inliers,int modelindex,float *Rvect,float *tvect,std::string datapath);
void optimalpnp(std::vector<cv::Point3d> list_points3d_inliers,std::vector<cv::Point3f> list_lines_inliers,int modelindex,float *Rvect,float *tvect,std::string datapath);

void detect_object_per_frame( cv::Mat& frame,std::vector<cv::DMatch> *good_matches,std::vector<cv::KeyPoint> *keypoints_scene,
                        int modelindex,RobustMatcher *rmatcher, cv::Mat *frame_matching,std::vector<cv::Point3f> *list_points3d_model_match,
                        std::vector<cv::Point2f> *list_points2d_scene_match, cv::Point3f *point3d_model,cv::Point2f *point2d_scene,cv::Mat *descriptors_model
                        ,std::vector<cv::Point3f> *list_points3d_model, cv::Mat *inliers_idx,std::vector<cv::Point2f> *list_points2d_inliers,std::vector<cv::Point3f> *list_points3d_inliers,std::vector<cv::Point3f> *list_lines_inliers, int *inliers_int,
                        int *outliers_int,std::string *inliers_str, std::string *outliers_str,PnPProblem pnp_detection ,int pnpMethod,int iterationsCount,
                        float reprojectionError,double confidence,float *sumofcosts ,std::string datapath);

void split(std::string const &str, const char delim, std::vector<std::string> &out);

// Draw a text with the question point
void drawQuestion(cv::Mat image, cv::Point3f point, cv::Scalar color);

// Draw a text with the number of entered points
void drawText(cv::Mat image, std::string text, cv::Scalar color);

// Draw a text with the number of entered points
void drawText2(cv::Mat image, std::string text, cv::Scalar color);

// Draw a text with the number of entered points
void drawText3(cv::Mat image, std::string text, cv::Scalar color);

// Draw a text with the number of entered points
void drawText4(cv::Mat image, std::string text, cv::Scalar color);

// Draw a text with the number of entered points
void drawText5(cv::Mat image, std::string text, cv::Scalar color);
void drawText7(cv::Mat image, std::string text, cv::Scalar color);

// Draw a text with the number of entered points
void drawText6(cv::Mat image, std::string text, cv::Scalar color);

// Draw a text with the frame ratio
void drawFPS(cv::Mat image, double fps, cv::Scalar color);

// Draw a text with the frame ratio
void drawConfidence1(cv::Mat image, double confidence, cv::Scalar color);

// Draw a text with the frame ratio
void drawConfidence2(cv::Mat image, double confidence, cv::Scalar color);

// Draw a text with the number of entered points
void drawCounter(cv::Mat image, int n, int n_max, cv::Scalar color);

// Draw the points and the coordinates
void drawPoints(cv::Mat image, std::vector<cv::Point2f> &list_points_2d, std::vector<cv::Point3f> &list_points_3d, cv::Scalar color);

// Draw only the 2D points
void draw2DPoints(cv::Mat image, std::vector<cv::Point2f> &list_points, cv::Scalar color);

// Draw an arrow into the image
void drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude = 9, int thickness=1, int line_type=8, int shift=0);

// Draw the 3D coordinate axes
void draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2f> &list_points2d);

// Draw the object mesh

// Computes the norm of the translation error
double get_translation_error(const cv::Mat &t_true, const cv::Mat &t);

// Computes the norm of the rotation error
double get_rotation_error(const cv::Mat &R_true, const cv::Mat &R);

// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix);

// Converts a given Euler angles to Rotation Matrix
cv::Mat euler2rot(const cv::Mat & euler);

// Converts a given string to an integer
int StringToInt ( const std::string &Text );
double StringTodouble ( const std::string &Text );

// Converts a given float to a string
std::string FloatToString ( float Number );

// Converts a given integer to a string
std::string IntToString ( int Number );

void createFeatures(const std::string &featureName, int numKeypoints, cv::Ptr<cv::Feature2D> &detector, cv::Ptr<cv::Feature2D> &descriptor);

cv::Ptr<cv::DescriptorMatcher> createMatcher(const std::string &featureName, bool useFLANN);

#endif /* UTILS_H_ */
