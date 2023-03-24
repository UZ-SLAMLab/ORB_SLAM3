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
#include <condition_variable>
#include "ImuTypes.h"

using namespace std;

bool b_continue_session;



void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./stereo_inertial_realsense_t265 path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 5) {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }

    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Enable both image streams (for some reason realsense does not allow to enable just one)
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8,30);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8,30);

    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    vector<double> v_accel_timestamp;
    vector<rs2_vector> v_accel_data;
    vector<double> v_gyro_timestamp;
    vector<rs2_vector> v_gyro_data;

    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data;
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data;
    vector<double> v_accel_timestamp_sync;
    vector<rs2_vector> v_accel_data_sync;

    cv::Mat imCV,imCV_right;
    int width_img = 848, height_img = 800;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames

    auto imu_callback = [&](const rs2::frame& frame){
        std::unique_lock<std::mutex> lock(imu_mutex);

        if(rs2::frameset fs = frame.as<rs2::frameset>()){
            count_im_buffer++;

            double new_timestamp_image = fs.get_timestamp()*1e-3;
            if(abs(timestamp_image-new_timestamp_image)<0.001){
                // cout << "Two frames with the same timeStamp!!!\n";
                count_im_buffer--;
                return;
            }

            rs2::video_frame color_frame = fs.get_fisheye_frame(1);
            rs2::video_frame color_frame_right = fs.get_fisheye_frame(2);
            imCV = cv::Mat(cv::Size(width_img, height_img), CV_8U, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
            imCV_right = cv::Mat(cv::Size(width_img, height_img), CV_8U, (void*)(color_frame_right.get_data()), cv::Mat::AUTO_STEP);

            timestamp_image = new_timestamp_image;
            double test = fs.get_timestamp()*1e-3;

            image_ready = true;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size()){

                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp, prev_accel_data, prev_accel_timestamp);
                v_accel_data_sync.push_back(interp_data);
                v_accel_timestamp_sync.push_back(target_time);
            }

            lock.unlock();
            cond_image_rec.notify_all();
        }
        else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()){
            if (m_frame.get_profile().stream_name() == "Gyro"){
                // It runs at 200Hz
                v_gyro_data.push_back(m_frame.get_motion_data());
                v_gyro_timestamp.push_back((m_frame.get_timestamp()+offset)*1e-3);
            }
            else if (m_frame.get_profile().stream_name() == "Accel"){
                // It runs at 60Hz
                prev_accel_timestamp = current_accel_timestamp;
                prev_accel_data = current_accel_data;

                current_accel_data = m_frame.get_motion_data();
                current_accel_timestamp = (m_frame.get_timestamp()+offset)*1e-3;

                while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
                {
                    int index = v_accel_timestamp_sync.size();
                    double target_time = v_gyro_timestamp[index];

                    rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp,
                                                                prev_accel_data, prev_accel_timestamp);

                    v_accel_data_sync.push_back(interp_data);
                    v_accel_timestamp_sync.push_back(target_time);

                }
            }
        }
    };

    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1);
    rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);
    float* Rbc = cam_stream.get_extrinsics_to(imu_stream).rotation;
    float* tbc = cam_stream.get_extrinsics_to(imu_stream).translation;
    std::cout << "Tbc = " << std::endl;
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++)
            std::cout << Rbc[i*3 + j] << ", ";
        std::cout << tbc[i] << "\n";
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    double timestamp;
    cv::Mat im,imright;

    // Clear IMU vectors
    v_gyro_data.clear();
    v_gyro_timestamp.clear();
    v_accel_data_sync.clear();
    v_accel_timestamp_sync.clear();

    double t_resize = 0.f;
    double t_track = 0.f;

    cv::Mat im_left, im_right;

    while (!SLAM.isShutDown()){
        std::vector<rs2_vector> vGyro;
        std::vector<double> vGyro_times;
        std::vector<rs2_vector> vAccel;
        std::vector<double> vAccel_times;

        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            while(!image_ready)
                cond_image_rec.wait(lk);

            if(count_im_buffer>1)
                cout << count_im_buffer -1 << " dropped frames\n";
            count_im_buffer = 0;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size()){
                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp, prev_accel_data, prev_accel_timestamp);

                v_accel_data_sync.push_back(interp_data);
                v_accel_timestamp_sync.push_back(target_time);
            }

            if(imageScale == 1.f)
            {
                im_left = imCV.clone();
                im_right = imCV_right.clone();
            }
            else
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
                cv::resize(imCV, im_left, cv::Size(width, height));
                cv::resize(imCV_right, im_right, cv::Size(width, height));
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

            // Copy the IMU data
            vGyro = v_gyro_data;
            vGyro_times = v_gyro_timestamp;
            vAccel = v_accel_data_sync;
            vAccel_times = v_accel_timestamp_sync;
            timestamp = timestamp_image;

            // Clear IMU vectors
            v_gyro_data.clear();
            v_gyro_timestamp.clear();
            v_accel_data_sync.clear();
            v_accel_timestamp_sync.clear();

            image_ready = false;
        }


        for(int i=0; i<vGyro.size(); ++i){
            ORB_SLAM3::IMU::Point lastPoint(vAccel[i].x, vAccel[i].y, vAccel[i].z,
                                            vGyro[i].x, vGyro[i].y, vGyro[i].z,
                                            vGyro_times[i]);
            vImuMeas.push_back(lastPoint);
            if(isnan(vAccel[i].x) || isnan(vAccel[i].y) || isnan(vAccel[i].z) ||
               isnan(vGyro[i].x) || isnan(vGyro[i].y) || isnan(vGyro[i].z) ||
               isnan(vGyro_times[i])){
                exit(-1);
            }
        }

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_Start_Track = std::chrono::monotonic_clock::now();
    #endif
#endif
        // Pass the image to the SLAM system
        SLAM.TrackStereo(im_left, im_right, timestamp, vImuMeas);
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_End_Track = std::chrono::monotonic_clock::now();
    #endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif

        // Clear the previous IMU measurements to load the new ones
        vImuMeas.clear();
    }

    SLAM.Shutdown();

    return 0;
}

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time){

    // If there are not previous information, the current data is propagated
    if(prev_time == 0){
        return current_data;
    }

    rs2_vector increment;
    rs2_vector value_interp;

    if(target_time > current_time) {
        value_interp = current_data;
    }
    else if(target_time > prev_time){
        increment.x = current_data.x - prev_data.x;
        increment.y = current_data.y - prev_data.y;
        increment.z = current_data.z - prev_data.z;

        double factor = (target_time - prev_time) / (current_time - prev_time);

        value_interp.x = prev_data.x + increment.x * factor;
        value_interp.y = prev_data.y + increment.y * factor;
        value_interp.z = prev_data.z + increment.z * factor;

        // zero interpolation
        value_interp = current_data;
    }
    else {
        value_interp = prev_data;
    }

    return value_interp;
}
