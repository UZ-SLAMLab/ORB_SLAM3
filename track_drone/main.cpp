//
// Created by firas on 25.8.2020.
//
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <random>
#include "Utils.h"
#include "Product.h"
#include "frame.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "motion_vectors.h"
#include "PnpObjective.h"
#include "BarrierMethodSettings.h"
#include "PnpProblemSolver.h"
#include <Eigen/src/Core/Matrix.h>
#include "Projection.h"
#include <opencv2/core/eigen.hpp>
#include <thread>
#include <tuple>
#include <opencv2/calib3d.hpp>
#include <dirent.h>
#include "PnPProblem.h"
#include "Product.h"

using namespace cv;
using namespace std;
using Eigen::MatrixXd;
using namespace PnP;

// struct for the pixels we want to track
struct pixels_for_track
{
	cv::Point2d pixel;
	bool found;
	bool draw;
	Vec3b pix_color;

	pixels_for_track(
		const cv::Point2d& pixel,
		const bool found,
		bool draw, const Vec3b& pix_color
	) : pixel(pixel), found(found), draw(draw), pix_color(pix_color)
	{
	}
};

// PnP solver
std::tuple<double, cv::Mat, cv::Mat> new_pnp(
	std::vector<Eigen::Vector3d> points,
	std::vector<Eigen::Vector3d> lines,
	std::vector<double> weights,
	std::vector<int> indices
);

// feature to Model matching + cv Ransac to remove outliers
void Match(
	cv::Mat& frame,
	class frame& frame_object,
	Product* product,
	RobustMatcher rmatcher,
	PnPProblem pnp_detection,
	float reprojectionError
);

void PnP_and_cost(
	Product* product,
	cv::Mat frame,
	bool draw_projection_of_the_map,
	const cv::Mat& CameraMatrix
);

int main(int argc, char* argv[])
{
	cout << "example script of motion vectors tracking for \n USAGE: ./motion_vectors_for_trainer \n";

	frame frameobj;
	// Set true if you want to solve PnP (for alignment) and not just track motion using (match and ransac)
	bool SOLVE_PNP_PER_FRAME = true;
	//    Set true to project and draw Map points
	bool draw_projection_of_the_map = false;
	// Set true if you want to use the color of the original map . no real need for it
	bool withcolor = false;
	// Product obejct, loads map, and stores tracked features and 3d points(input name of folder for the descriptors and points , and optional colors)
	const std::string folder_path = "../Maps/";
	auto* Map_3d = new Product("new_drone_reg", withcolor, folder_path);

// params for orb feature detector and pnp ransac
	int numKeyPoints = 1000;
	float ratioTest = 0.8;
	float reprojectionError = 100;
	RobustMatcher rmatcher;
	Ptr<FeatureDetector> detector, descriptor;
	rmatcher.setFeatureDetector(detector);                                // set feature detector
	rmatcher.setDescriptorExtractor(descriptor);                      // set descriptor extractor
	bool useFLANN = true;
	rmatcher.setDescriptorMatcher(createMatcher("ORB", useFLANN));     // set matcher
	rmatcher.setRatio(ratioTest);

//    camera matrix for Opencv Ransac
	const double params_WEBCAM[] = { 498.81472778, 500.44668579, 326.65849033, 233.99551581 };
	PnPProblem pnp_detection(params_WEBCAM);


	//    motion vector matrix(ffmpeg) . all_MVS is the matrix tthat contains all the motion vectors for the full video .
//example usage : all_MVS[1][0][0]= movement in axis x of the first block  in the first frame, block sizes(grid step) are set to (16x16)
//                all_MVS[1][0+(frame_width/16)][0]= movement in axis y of the first block  in the first frame, block sizes(grid step) are set to (16x16)
// given pixel= (x,y) and frame i, then, delta_x=all_MVS[l][floor(pixel.y / 16)-1][floor(p.pixel.x / 16)-1]
//                                       delta_y=all_MVS[l][floor(p.pixel.y / 16) + (frame_width/16)-1][floor(p.pixel.x / 16)-1];
	auto start = std::chrono::steady_clock::now();

	const char* vid_path = "../drone_newtry.MOV";
	std::vector<std::vector<std::vector<int>>> all_MVS = get_optical_flow_matrix(vid_path);
	cout << all_MVS[2][10][10] << endl;
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed_seconds_extract_mv_matrix = end - start;
	cout << "time to extract MV Matrix = " << elapsed_seconds_extract_mv_matrix.count() << endl;

	int pix_size = 1;
	VideoCapture cap;
	cap.open(vid_path);
	if (!cap.isOpened())
	{
		cout << "Could not open the camera device" << endl;
		return -1;
	}
	cv::Mat frame;
	cap.read(frame);
	cv::namedWindow("image", cv::WINDOW_KEEPRATIO);
	cv::Mat greyimg;
	cv::cvtColor(frame, greyimg, cv::COLOR_BGR2GRAY);
	std::vector<pixels_for_track> pixels;
	// This might be a better way for you to chose the features
	// cv::goodFeaturesToTrack(greyimg, cornerstotrck, maxCorners, qulitylevel, minDistance, cv::noArray(), blocksize);

	// intrinsics Matrix
	float values[9] = { 498.81472778, 0, 326.65849033, 0, 500.44668579, 233.99551581, 0, 0, 1 };
	cv::Mat CameraMatrix(3, 3, CV_32FC1, values);

// frame obj has all data needed in the frame
	frameobj.loadframe(greyimg, numKeyPoints, ratioTest, CameraMatrix);
//    match 3d map with first frame
	Match(frame, frameobj, Map_3d, rmatcher, pnp_detection, reprojectionError);

	// store inliers for tracking, needs to be change or optimized
	std::vector<cv::Point2d> found;
	for (auto& i : Map_3d->tracked2dpoints[0])
	{
		Vec3b pixel_color = frame.at<Vec3b>(Point(i.x, i.y));
		pixel_color += Vec3b(10, 10, 10);
		pixels.emplace_back(i, false, false, pixel_color);
		found.push_back(i);
	}

	int frame_count = 0;
	int frame_count2 = 0;

//    for testing motion vectors from video compresion VS opencv optial flow (lK)
//    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
//    Size subPixWinSize(10,10), winSize(100,100);
//    if you Have good pnp results you can project the axis of the object on to the image
//    vector<Point3d> axis3D;
//    axis3D.emplace_back( Point3d(1,0,0));
//    axis3D.emplace_back( Point3d(1,0,0));
//    axis3D.emplace_back( Point3d(1,0,0));

	// pixels to draw
	std::vector<std::vector<cv::Point2d>> draw(pixels.size());
	vector<Point2f> all_centers;

	while (cap.read(frame))
	{
		cv::cvtColor(frame, greyimg, cv::COLOR_BGR2GRAY);

		// every n frames compute new features for tracking (chose n depending on your needs),
		// or if we couldn't solve pnp in previous frame (because we didn't have enough inliers)
		if (frame_count % 50 == 0 || isnan(Map_3d->sumofcosts))
		{
			// clean Draw vector
			for (int in = 0; in < pixels.size(); in++)
			{
				draw[in].clear();
			}
			//calculate and match new points
			frameobj.loadframe(greyimg, numKeyPoints, ratioTest, CameraMatrix);
			Match(frame, frameobj, Map_3d, rmatcher, pnp_detection, reprojectionError);
			// use this if you want opencv optical flow instead of motion vectors
			// cv::goodFeaturesToTrack(greyimg,cornerstotrck,maxCorners,qulitylevel,minDistance,cv::noArray(),blocksize);
			// clear all data
			pixels.clear();
			found.clear();
			std::vector<Point2f> newtrackedpoints;

			// update new data
			for (auto& i : Map_3d->tracked2dpoints[0])
			{
				Vec3b pixel_color = frame.at<Vec3b>(Point(i.x, i.y));
				pixel_color += Vec3b(10, 10, 10);
				pixels.emplace_back(i, false, false, pixel_color);
				found.push_back(i);
			}
			draw.resize((pixels.size()));
			frame_count = 0;
		}

		// HARD CODED PART FOR MY example , remove for other examples or use the logic to draw what you need
		// START    this part is not interesting at all .. its just for drawing
		float xes = 0;
		float yes = 0;
		int height = frame.rows, width = frame.cols;
		for (int j = 0; j < pixels.size(); ++j)
		{
			if (draw[j].size() != 0)
			{
				xes += draw[j][draw[j].size() - 1].x;
				yes += draw[j][draw[j].size() - 1].y;
			}
			else
			{
				xes += found[j].x;
				yes += found[j].y;
			}
			if (pixels[j].pixel.x > (width - pix_size) || (pixels[j].pixel.x < pix_size) ||
				(pixels[j].pixel.y > (height - pix_size)) || (pixels[j].pixel.y < pix_size))
				pixels[j].draw = true;
			if (!pixels[j].draw)
				draw[j].emplace_back(pixels[j].pixel);

		}
		if (frame_count2 >= 200)
		{
			Point2f massCenter(xes / pixels.size(), yes / pixels.size());

			all_centers.emplace_back(massCenter);
			// Point2f massCenter(xes / pixels.size(), yes / pixels.size());
			// circle(frame, massCenter, 6, Scalar(255, 0, 0), 1, -5, 0);
			cv::line(frame, massCenter + Point2f(-80, 40), massCenter + Point2f(90, 40), Scalar(99, 210, 114),
				2);
			cv::line(frame, massCenter + Point2f(-80, 40), massCenter + Point2f(-80, -70), Scalar(99, 210, 114),
				2);
			cv::line(frame, massCenter + Point2f(-80, -70), massCenter + Point2f(90, -70), Scalar(99, 210, 114),
				2);
			cv::line(frame, massCenter + Point2f(90, 40), massCenter + Point2f(90, -70), Scalar(99, 210, 114),
				2);

			for (int j = 0; j < pixels.size(); ++j)
			{
				for (int i = 0; i < all_centers.size(); ++i)
				{
					if (i < all_centers.size() - 1 && all_centers.size() > 1)
					{
						cv::line(frame, all_centers[i], all_centers[i + 1], Scalar(161, 135, 63), 4);

					}
				}
				cv::line(frame, all_centers[all_centers.size() - 1],
					all_centers[all_centers.size() - 1] + Point2f(70, -15),
					Scalar(161, 135, 63), 4);
				cv::drawMarker(frame, all_centers[all_centers.size() - 1] + Point2f(70, -15),
					Scalar(51, 51, 255), 1, 16);
			}
		}
// ***************************************    END    this part is not interesting at all .. its just for drawing*************************************8

//  all pixels assumed to be not found in next frame
		for (auto p:pixels)
		{
			p.found = false;
		}
		vector<Point3f> temp3d;
		vector<Point2f> temp2d;
		vector<Point3f> templines;
		int count_pix = 0;
//        track each pixel if possible
		for (auto& p : pixels)
		{
			if (floor(p.pixel.y / 16) + floor(height / 16.) - 1 <= floor(height / 16.) * 2 - 1 &&
				floor(p.pixel.x / 16) - 1 <= floor(width / 16.) * 2 - 1)
			{
				int xforXmotion = floor(p.pixel.y / 16) - 1;
				int xforYmotion = floor(p.pixel.y / 16) + floor(height / 16.) - 1;
				int y = floor(p.pixel.x / 16) - 1;
				if (xforXmotion < 0 || xforYmotion < 0 || y < 0)
				{ continue; }
				int delta_x = all_MVS[frame_count2][xforXmotion][y];
				int delta_y = all_MVS[frame_count2][xforYmotion][y];
//                still features ,  pixels that move in the x axis are automaticaly considered outliers (training videos)
				if (((abs(delta_x) == 0 && abs(delta_y) == 0)))
				{
					count_pix++;
					continue;
				}
				p.pixel.x += delta_x;
				p.pixel.y += delta_y;
				temp2d.emplace_back(cv::Point2d(p.pixel.x, p.pixel.y));
				temp3d.push_back(Map_3d->tracked3dpoints[0][count_pix]);
//            Update the line of the pixel for PnP
				cv::Mat pt = (cv::Mat_<float>(3, 1) << p.pixel.x, p.pixel.y, 1);
				cv::Mat invK = CameraMatrix.inv();
				cv::Mat templine = invK * pt;
				templine = templine / norm(templine);
				cv::Point3f v(templine);
				templines.push_back(v);
				p.found = true;
				count_pix++;
			}
		}
//     if not first frame
		if (frame_count != 0)
		{
			Map_3d->tracked2dpoints[0] = temp2d;
			Map_3d->tracked3dpoints[0] = temp3d;
			Map_3d->trackedlines[0] = templines;
		}
		frame_count++;
//        solve PnP problem and update cost per frame if number of inliers larger than 10
		if (SOLVE_PNP_PER_FRAME)
		{
			if (Map_3d->tracked2dpoints[0].size() > 10)
			{
				PnP_and_cost(Map_3d, frame, draw_projection_of_the_map, CameraMatrix);
			}
			cout << "cost for frame" << frame_count2 << " == " << Map_3d->sumofcosts << endl;
		}

		imshow("image", frame);
		waitKey(30);
		frame_count2++;
	}

	cap.release();
	cv::destroyAllWindows();

	return 0;
}

// PnP Solver
std::tuple<double, cv::Mat, cv::Mat> new_pnp(std::vector<Eigen::Vector3d> points,
	std::vector<Eigen::Vector3d> lines,
	std::vector<double> weights,
	std::vector<int> indices)
{
	auto pnp = PnpProblemSolver::init();
	auto barrier_method_settings = BarrierMethodSettings::init();
	barrier_method_settings->epsilon = 4E-8;
	barrier_method_settings->verbose = false;
	auto pnp_input = PnpInput::init(std::move(points), std::move(lines), std::move(weights), std::move(indices));
//    auto start = std::chrono::system_clock::now();
	auto pnp_objective = PnpObjective::init(pnp_input);
	auto pnp_res = pnp->solve_pnp(pnp_objective, barrier_method_settings);
//    auto end = std::chrono::system_clock::now();
//    std::chrono::duration<double> elapsed_seconds = end - start;
	auto R = pnp_res.rotation_matrix();
	auto t = pnp_res.translation_vector();
	cv::Mat cvR, cvt;
	cv::eigen2cv(R, cvR);
	cv::eigen2cv(t, cvt);
	auto cost = pnp_res.cost();
	return std::make_tuple(cost, cvR, cvt);

}

//THIS FUNCTION should be replaced
void Match(cv::Mat& frame,
	class frame& frame_object,
	Product* product,
	RobustMatcher rmatcher,
	PnPProblem pnp_detection,
	float reprojectionError)
{
//    RobustRansacScheme robust_ransac_scheme(0.2);
	vector<DMatch> good_matches;       // to obtain the 3D points of the model
	vector<Point3f> list_points3d_model_match; // container for the model 3D coordinates found in the scene
	vector<Point2f> list_points2d_scene_match; // container for the model 2D coordinates found in the scene
	vector<Point3f> list_lines_match;
	vector<cv::Mat> list_lines4coreset_match;
	cv::Mat inliers_idx;
	// ---------------------------------
	vector<Point2f> list_points2d_inliers;
	vector<Point3f> list_points3d_inliers;
	vector<Point3f> list_lines_inliers;
	vector<Mat> list_lines_for_coreset_inliers;
	int num_of_tracked = 0;
	rmatcher.fastRobustMatch2(good_matches, frame_object.descriptors_, product->products_descriptors_);
//        loop over matches
	for (auto& good_match : good_matches)
	{
		cv::Point3f point3d_model = product->products_3d_map_[good_match.trainIdx];  // 3D point from model
		cv::Point2f
			point2d_scene = frame_object.list_keypoints_[good_match.queryIdx].pt; // 2D point from the scene
//            if(frame_object.list_keypoints_[good_matches[match_index].queryIdx].)
		list_points3d_model_match.push_back(point3d_model);         // add 3D point
		list_points2d_scene_match.push_back(point2d_scene);         // add 2D point
		list_lines_match.push_back(frame_object.list_lines_[good_match.queryIdx]);
//        list_lines4coreset_match.push_back(frame_object.list_lines_4coreset_[good_matches[match_index].queryIdx].clone());
	}
	vector<int> inliers_indices;
	cv::Mat cv_inliers_idx;
	if (good_matches.size() >= 10)
	{
//            use solve pnp ransac to find inliers
		pnp_detection.estimatePoseRANSAC(list_points3d_model_match,
			list_points2d_scene_match,
			SOLVEPNP_ITERATIVE, inliers_idx,
			300, reprojectionError, 0.1);
	}
// update inliers
	for (int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index)
	{
		int n = inliers_idx.at<int>(inliers_index);// i-inlier
		Point2f point2d = list_points2d_scene_match[n];     // i-inlier point 2D
		Point3f point3d = list_points3d_model_match[n];     // i-inlier point 3D
		Point3f goodline = list_lines_match[n];
		list_points2d_inliers.push_back(point2d);           // add i-inlier to list
		list_points3d_inliers.push_back(point3d);
//        cv::Mat goodline4coreset = list_lines4coreset_match[n].clone();
		list_lines_inliers.push_back(goodline);
//        list_lines_for_coreset_inliers.push_back(goodline4coreset.clone());
		product->tracked2dpoints[0].push_back(point2d);
		product->tracked3dpoints[0].push_back(point3d);
		product->trackedlines[0].push_back(goodline);
//        product->trackedlines4coreset[0].push_back(goodline4coreset);

	}
}

void PnP_and_cost(Product* product, cv::Mat frame, bool draw_projection_of_the_map, const cv::Mat& CameraMatrix)
{
// stupid converter. had no time to change all  , basicaly nepnp needs vectors from Eigen library while the whol scrippt works on opencv Mat...
	vector<Point3d> points3d_double(product->tracked3dpoints[0].begin(), product->tracked3dpoints[0].end());
	vector<Point3d> lines_double(product->trackedlines[0].begin(), product->trackedlines[0].end());
	std::vector<Eigen::Vector3d> eigenpoints;
	std::vector<Eigen::Vector3d> eigenlines;
	vector<double> core_w;
	std::vector<int> Coreset_Indexes;
	int counter = 0;
	for (int p_indx = 0; p_indx < product->tracked3dpoints[0].size(); p_indx++)
	{
		eigenpoints.emplace_back(points3d_double[p_indx].x, points3d_double[p_indx].y, points3d_double[p_indx].z);
		eigenlines.emplace_back(lines_double[p_indx].x, lines_double[p_indx].y, lines_double[p_indx].z);
		core_w.push_back(1);
		Coreset_Indexes.push_back(counter);
		counter++;
	}
	auto pnp_res = new_pnp(eigenpoints, eigenlines, core_w, Coreset_Indexes);

	// update sum of costs per frame
	product->sumofcosts = std::get<0>(pnp_res);
	cv::Mat rot = std::get<1>(pnp_res);
	cv::Mat trans = std::get<2>(pnp_res);
	rot = rot.t();
	trans = trans;

	if (draw_projection_of_the_map)
	{
		Projection projector(rot, trans, CameraMatrix);
		projector.set_P_matirx(rot, trans);

		cv::Mat tmprojim = frame.clone();

		vector<Point2f> list_points2d_mesh = projector.verify_points(product->products_3d_map_);
		for (auto point_2d : list_points2d_mesh)
		{
			//  make sure projected points dont go out of frame boundaries
			if (point_2d.y < frame.rows && point_2d.x < frame.cols && point_2d.x > 0 && point_2d.y > 0)
			{
				// Draw Selected points
				Vec3b color = frame.at<Vec3b>(Point(point_2d.x, point_2d.y)) + Vec3b(15, 15, 15);
				cv::circle(tmprojim, point_2d, 5, cv::Scalar(255, 0, 0), 3);
			}
		}
		imshow("projection_of_the_map", tmprojim);
		cv::waitKey(30);
	}
}
