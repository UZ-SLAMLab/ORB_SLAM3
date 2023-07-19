/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>

#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
   cout << "Finishing session" << endl;
   b_continue_session = false;

}

int main(int argc, char **argv)
{

    if(argc < 3 || argc > 4)
    {
        cerr << endl << "Usage: ./mono_realsense_t265 path_to_vocabulary path_to_settings (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4)
    {
        file_name = string(argv[argc-1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;


    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Enable the left camera
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    rs2::pipeline_profile pipe_profile = pipe.start(cfg);

    cout.precision(17);

    /*cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "IMU data in the sequence: " << nImu << endl << endl;*/

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cv::Mat imCV;

    rs2::stream_profile fisheye_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1);
    rs2_intrinsics intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();
    int width_img = intrinsics.width;
    int height_img = intrinsics.height;

    double t_resize = 0.f;
    double t_track = 0.f;

    while(b_continue_session)
    {
        //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        // Get the stream from the device
        rs2::frameset frame_set = pipe.wait_for_frames();

        double timestamp_ms = frame_set.get_timestamp(); //RS2_FRAME_METADATA_SENSOR_TIMESTAMP
        // cout << "timestamp: " << timestamp_ms << endl;

        if(rs2::video_frame image_frame = frame_set.first_or_default(RS2_STREAM_FISHEYE))
        {
            rs2::video_frame frame = frame_set.get_fisheye_frame(1); // Left image
            imCV = cv::Mat(cv::Size(width_img, height_img), CV_8UC1, (void*)(frame.get_data()), cv::Mat::AUTO_STEP);
            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC14
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
                int width = imCV.cols * imageScale;
                int height = imCV.rows * imageScale;
                cv::resize(imCV, imCV, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC14
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

            // clahe
            //clahe->apply(imLeft,imLeft);
            //clahe->apply(imRight,imRight);

#ifdef REGISTER_TIMES
  #ifdef COMPILEDWITHC14
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
  #endif
#endif

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(imCV, timestamp_ms);

#ifdef REGISTER_TIMES
  #ifdef COMPILEDWITHC14
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
  #endif
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        }
    }

    pipe.stop();

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
