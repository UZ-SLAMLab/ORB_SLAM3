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
#include <iomanip>

#include <condition_variable>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char **argv) {

    if (argc != 2) {
        cerr << endl
             << "Usage: ./recorder_realsense_D435i path_to_saving_folder"
             << endl;
        return 1;
    }

    string directory = string(argv[argc - 1]);

    struct sigaction sigIntHandler;


    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    }
    else
        selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            if (index == 1) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,5000);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
            }
            // std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            get_sensor_option(sensor);
            if (index == 2){
                // RGB camera
                sensor.set_option(RS2_OPTION_EXPOSURE,100.f);
            }

            if (index == 3){
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION,0);
            }

        }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F); //, 250); // 63
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F); //, 400);

    // IMU callback
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    vector<double> v_gyro_timestamp;
    vector<rs2_vector> v_gyro_data;
    vector<double> v_acc_timestamp;
    vector<rs2_vector> v_acc_data;

    cv::Mat imCV;
    int width_img, height_img;
    double timestamp_image;
    bool image_ready = false;


    auto imu_callback = [&](const rs2::frame& frame)
    {
        std::unique_lock<std::mutex> lock(imu_mutex);

        if(rs2::frameset fs = frame.as<rs2::frameset>())
        {
            rs2::video_frame color_frame = fs.get_infrared_frame();
            imCV = cv::Mat(cv::Size(width_img, height_img), CV_8U, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);

            timestamp_image = fs.get_timestamp()*1e-3;
            image_ready = true;

            lock.unlock();
            cond_image_rec.notify_all();
        }
        else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>())
        {
            if (m_frame.get_profile().stream_name() == "Gyro")
            {
                // It runs at 200Hz
                v_gyro_data.push_back(m_frame.get_motion_data());
                v_gyro_timestamp.push_back((m_frame.get_timestamp()+offset)*1e-3);
            }
            else if (m_frame.get_profile().stream_name() == "Accel")
            {
                // It runs at 60Hz
                v_acc_data.push_back(m_frame.get_motion_data());
                v_acc_timestamp.push_back((m_frame.get_timestamp()+offset)*1e-3);
            }
        }
    };

    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);
    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
    rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);

    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;

    cv::Mat im;
    ofstream accFile, gyroFile, cam0TsFile;
    accFile.open (directory + "/IMU/acc.txt");
    gyroFile.open (directory + "/IMU/gyro.txt");
    cam0TsFile.open (directory + "/cam0/times.txt");

    // Clear IMU vectors
    v_gyro_data.clear();
    v_gyro_timestamp.clear();
    v_acc_data.clear();
    v_acc_timestamp.clear();

    cv::namedWindow("cam0",cv::WINDOW_AUTOSIZE);

    while (b_continue_session){
        std::vector<rs2_vector> vGyro;
        std::vector<double> vGyro_times;
        std::vector<rs2_vector> vAccel, vAccel_Sync;
        std::vector<double> vAccel_times;
        double imTs;
        {
            {
                std::unique_lock<std::mutex> lk(imu_mutex);
                if (!image_ready) // wait until image read from the other thread
                    cond_image_rec.wait(lk);
            }
            std::lock_guard<std::mutex> lk(imu_mutex);

            // Copy the IMU data to local single thread variables
            vGyro = v_gyro_data;
            vGyro_times = v_gyro_timestamp;
            vAccel = v_acc_data;
            vAccel_times = v_acc_timestamp;
            imTs = timestamp_image;
            im = imCV.clone();

            // Clear IMU vectors
            v_gyro_data.clear();
            v_gyro_timestamp.clear();
            v_acc_data.clear();
            v_acc_timestamp.clear();

            image_ready = false;
        }

        cv::imshow("cam0",im);

        // save image and IMU data
        long int imTsInt = (long int) (1e9*imTs);
        string imgRepo = directory + "/cam0/" + to_string(imTsInt) + ".png";
        if(!im.empty()) {
            cv::imwrite(imgRepo, im);
            cam0TsFile << imTsInt << endl;
        } else {
            cout << "image empty!! \n";
        }

        //assert(vAccel.size() == vAccel_times.size());
        //assert(vGyro.size() == vGyro_times.size());

        for(int i=0; i<vAccel.size(); ++i){
            accFile << std::setprecision(15) << vAccel_times[i] << "," << vAccel[i].x << "," << vAccel[i].y << "," << vAccel[i].z << endl;
        }

        for(int i=0; i<vGyro.size(); ++i){
            gyroFile << std::setprecision(15) << vGyro_times[i] << "," << vGyro[i].x << "," << vGyro[i].y << "," << vGyro[i].z << endl;
        }

        cv::waitKey(10);
    }

    accFile.close();
    gyroFile.close();
    cam0TsFile.close();

    cout << "System shutdown!\n";
}