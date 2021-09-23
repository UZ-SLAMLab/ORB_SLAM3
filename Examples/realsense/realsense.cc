/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"
// A simple pose structure containing position vector and rotation matrix.
typedef struct _Pose {
    cv::Mat position;
    cv::Mat rotation;
} Pose;

int main(int argc, char * argv[]){
    if(argc < 3)
    {
        cerr << endl << "Usage: ./oakd path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    std::cout << "Realsense/ORB_SLAM3 Experiment" << std::endl;

	int width = 1280;
	int height = 720;
	int fps = 30;
	rs2::config config;
    // This Y8 format is rectified
    // https://github.com/IntelRealSense/librealsense/issues/3294
	config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

	// start pipeline
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile = pipeline.start(config);

    cv::Mat R1, R2, P1, P2, Q;

    // vocabulary, yaml, type, gui-enabled
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, true);

    // Formatter, for printing out matrices in a reasonable way.
    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    fmt->set64fPrecision(3);
    fmt->set32fPrecision(3);

    // We also want somewhere to store our pose data
    Pose pose;

    // The time of each frame is required for SLAM, so we take an epoch time
    // (i.e. our start time) now
    auto slam_epoch = std::chrono::steady_clock::now();

	while (true) {
		// wait for frames and get frameset
		rs2::frameset frameset = pipeline.wait_for_frames();
        auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
        double frame_timestamp_s = elapsed_time.count() / 1000000000.0;
        std::cout << std::setprecision(4) << frame_timestamp_s << ": ";

		// get left and right infrared frames from frameset
		rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

		cv::Mat rectif_left_frame = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
		cv::Mat rectif_right_frame = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());



        cv::Mat raw_pose = SLAM.TrackStereo(
            rectif_left_frame,
            rectif_right_frame,
            frame_timestamp_s
        );

        bool loc_fix_available = !raw_pose.empty();
        if (loc_fix_available) {
            // The pose matrix is a 4x4 extrinsic matrix, with the form:
            // [R_3x3 T_3x1; [0 0 0 1]], we can find the camera position with 
            // C = -R'T (R' = R transpose).
            pose.rotation = raw_pose(cv::Rect(0, 0, 3, 3));
            cv::Mat T = raw_pose(cv::Rect(3, 0, 1, 3));
            pose.position = -pose.rotation.t()*T;

            // Print the updated position, but transpose it so that instead of
            // a column vector we have a row vector, which is easier to read.
            std::cout << 
                "position: " << 
                fmt->format(pose.position.t()) << 
                std::endl;
        }
        else {
            // If we didn't get a pose update log it.
            std::cout << "no pose update" << std::endl;
        }

		cv::imshow("img_l", rectif_left_frame);
		cv::imshow("img_r", rectif_right_frame);
		char c = cv::waitKey(1);
		if (c == 's')
		{

		}
		else if (c == 'q')
			break;
	}

    SLAM.Shutdown();

	return EXIT_SUCCESS;
}