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
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <depthai/depthai.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"
// A simple pose structure containing position vector and rotation matrix.
typedef struct _Pose {
    cv::Mat position;
    cv::Mat rotation;
} Pose;

// WLS parameters, taken from the OpenCV WLS filter docs recommended values.
#define WLS_LAMBDA (8000)
#define WLS_SIGMA (1.0)

// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static std::atomic<bool> downscaleColor{ true };
static constexpr int fps = 30;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./oakd path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    std::cout << "OAK-D/ORB_SLAM3 Experiment" << std::endl;

    // Create pipeline
    dai::Pipeline pipeline;
    std::vector<std::string> queueNames;

    // Define sources and outputs
    // auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto mono_left = pipeline.create<dai::node::MonoCamera>();
    auto mono_right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto xout_rectif_left = pipeline.create<dai::node::XLinkOut>();
    auto xout_rectif_right = pipeline.create<dai::node::XLinkOut>();
    // auto xout_disp = pipeline.create<dai::node::XLinkOut>();
    // auto rgbOut = pipeline.create<dai::node::XLinkOut>();

    xout_rectif_left->setStreamName("rectified_left");
    xout_rectif_right->setStreamName("rectified_right");
    // xout_disp->setStreamName("disparity");
    // rgbOut->setStreamName("rgb");


    // Properties
    mono_left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    mono_left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    mono_left->setFps(20.0);
    mono_right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    mono_right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    mono_right->setFps(20.0);
    // camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    // camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    // camRgb->setFps(fps);
    // if (downscaleColor) camRgb->setIspScale(2, 3);
    // // For now, RGB needs fixed focus to properly align with depth.
    // // This value was used during calibration
    // camRgb->initialControl.setManualFocus(135);

    // left->setResolution(monoRes);
    // left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    // left->setFps(fps);
    // right->setResolution(monoRes);
    // right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    // right->setFps(fps);

    // stereo->setOutputRectified(true);
    // stereo->setOutputDepth(false);
    stereo->setRectifyEdgeFillColor(0);
    stereo->setRectifyMirrorFrame(false);
    // stereo->setExtendedDisparity(true); // normally true with other code

    // stereo->initialConfig.setConfidenceThreshold(230);
    // // LR-check is required for depth alignment
    // stereo->setLeftRightCheck(true);
    // //stereo->setExtendedDisparity(true);
    // stereo->initialConfig.setMedianFilter(dai::StereoDepthProperties::MedianFilter::KERNEL_7x7);
    // stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    // Linking
    // camRgb->isp.link(rgbOut->input);
    mono_left->out.link(stereo->left);
    mono_right->out.link(stereo->right);
    stereo->rectifiedLeft.link(xout_rectif_left->input);
    stereo->rectifiedRight.link(xout_rectif_right->input);
    // stereo->disparity.link(xout_disp->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Sets queues size and behavior
    auto rectif_left_queue = device.getOutputQueue("rectified_left", 8, false);
    auto rectif_right_queue = device.getOutputQueue("rectified_right", 8, false);
    // auto disp_queue = device.getOutputQueue("disparity", 8, false);
    // auto rgb_queue = device.getOutputQueue("rgb", 8, false);

    auto wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    wls_filter->setLambda(WLS_LAMBDA);
    wls_filter->setSigmaColor(WLS_SIGMA);

    cv::Mat R1, R2, P1, P2, Q;

    // vocabulary, yaml, type, gui-enabled
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, true);

    // Formatter, for printing out matrices in a reasonable way.
    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    fmt->set64fPrecision(3);
    fmt->set32fPrecision(3);

    // We also want somewhere to store our pose data
    Pose pose;

    cv::Mat imLeft, imRight, imLeftRect, imRightRect;

    // The time of each frame is required for SLAM, so we take an epoch time
    // (i.e. our start time) now
    auto slam_epoch = std::chrono::steady_clock::now();

    while(true){
        cv::Mat rectif_left_frame = rectif_left_queue->get<dai::ImgFrame>()->getCvFrame();
        cv::Mat rectif_right_frame = rectif_left_queue->get<dai::ImgFrame>()->getCvFrame();
        // cv::Mat disp_map_frame = disp_queue->get<dai::ImgFrame>()->getCvFrame();
        // cv::Mat rgb_frame = rgb_queue->get<dai::ImgFrame>()->getCvFrame();
        
        auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
        double frame_timestamp_s = elapsed_time.count() / 1000000000.0;
        std::cout << std::setprecision(4) << frame_timestamp_s << ": ";

        cv::Mat raw_pose = SLAM.TrackStereo(
            rectif_left_frame,
            rectif_right_frame,
            frame_timestamp_s
        );

        // The output pose may be empty if the system was unable to track the
        // movement, so only get position and rotation if pose isn't empty. We
        // also put this info an a localisation fix available flag for later
        // use. 
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

        // // The raw disparity map is flipped, since we flipped the rectified
        // // images, so we must flip it as well.
        // cv::flip(disp_map_frame, disp_map_frame, 1);

        // // Filter the disparity map
        // cv::Mat filtered_disp_map;
        // wls_filter->filter(disp_map_frame, rectif_right_frame, filtered_disp_map);

        // // Apply a colormap to the filtered disparity map, but don't normalise
        // // it. Normalising the map will mean that the color doesn't correspond
        // // directly with disparity.
        // cv::Mat colour_disp;
        // cv::applyColorMap(filtered_disp_map, colour_disp, cv::COLORMAP_JET);
        // cv::imshow("disparity", colour_disp);

        // // See if q pressed, if so quit
        // if (cv::waitKey(1) == 'q') {
        //     break;
        // }
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
